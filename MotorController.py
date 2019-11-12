import time
import datetime
import math
import odrive
from odrive.enums import *
from multiprocessing import Process, Value
import os

class MotorController:
    encoder_cpr = 8192                              # holds the counts per revolution of the encoder that is attached to the motor
    degree_per_count = 360 / encoder_cpr            # holds the degrees per one count of the encoder
    current_torque_factor = 2.75                    # holds a factor to internally convert from torque(Nm) to current(Ampere)

    motor_control_process = None                    # holds an instance of the Process-class for actually controlling the motor

    def __init__(self):
        self.terminator = Value('b', False)             # indicator for terminating the loop inside the function 'send_current_command'
        self.control_freq = Value('i', 360)             # target refresh rate for the motor-control process
        self.signal_in = Value('f', 0.0)                # raw force-feedback value coming from the game
        self.game_status_playing = Value('b', False)    # game status (playing/not-playing)
        self.motor_pause_mode = Value('b', False)       # if the motor should be idle when game is paused
        self.data_reconstruction = Value('b', False)    # if the data-reconstruction feature should be used
        self.data_smoothing_level = Value('i', 0)       # if the temproal smoothing feature should be used
        self.friction = Value('f', 0.0)                 # strength of the friction effect
        self.damping = Value('f', 0.0)                  # strength of the damping effect
        self.inertia = Value('f', 0.0)                  # strength of the inertia effect
        self.encoder_pos = Value('f', 0.0)              # holds the wheel position in encoder counts
        self.wheel_pos_deg = Value('f', 0.0)            # holds the wheel position in degrees
        self.wheel_offset = Value('f', 0.0)             # holds the wheel offset used to center the wheel for the virtual controller
        self.bump_to_bump = Value('f', 900.0)           # holds the physical limit of the wheel (used by the motor controller)
        self.motor_current = Value('f', 0.0)            # holds the current(Ampere) that is applied to the motor at any moment
        self.actual_max_force = Value('f', 0.0)         # holds the actual maximum torque(Nm) the motor should be limited to
        self.motor_connected = Value('b', False)        # motor status (connected/not-connected)
        self.due_for_restart = Value('b', False)        # is set to true when connection to odrive board is lost
        self.achieved_freq = Value('f', 0.0)            # actual refresh rate of the motor-control process



    def start(self):
        self.terminator.value = False
        self.motor_control_process = None
        self.motor_control_process = Process(target=self.send_current_command,
                                             args=(self.terminator,
                                                   self.control_freq,
                                                   self.signal_in,
                                                   self.game_status_playing,
                                                   self.motor_pause_mode,
                                                   self.data_reconstruction,
                                                   self.data_smoothing_level,
                                                   self.encoder_pos,
                                                   self.wheel_pos_deg,
                                                   self.wheel_offset,
                                                   self.bump_to_bump,
                                                   self.actual_max_force,
                                                   self.friction,
                                                   self.damping,
                                                   self.inertia,
                                                   self.motor_current,
                                                   self.motor_connected,
                                                   self.due_for_restart,
                                                   self.achieved_freq))
        self.motor_control_process.start()

    def send_current_command(self, terminator, freq, signal_in, game_status_playing, motor_pause_mode, reconstruction, smoothing, encoder_pos, wheel_pos_deg, wheel_offset, bump_to_bump, actual_max_force, friction, damping, inertia, motor_current, motor_connected, due_for_restart, achieved_freq):
        my_drive = self.setup_motor()
        motor_connected.value = True

        last_frame_overtime = 0.0

        signal_history = [0.0] * 2
        steps_sice_last_change = 0
        last_max_steps = 1.0
        smoothing_history = []

        # friction
        pos_diff_threshold = 2.5
        standstill_pos = 0.0
        # damping
        damping_threshold = 0.5
        # inertia
        inertia_threshold = 1.5

        vel_hist = [0.0, 0.0]
        old_vel = 0.0

        old_motor_current_mode = False
        my_drive.axis0.requested_state = AXIS_STATE_IDLE

        while not terminator.value:
            start = datetime.datetime.now()

            try:
                # get wheel position, velocity and acceleration
                encoder_pos.value = my_drive.axis0.encoder.pos_estimate * self.degree_per_count
                wheel_pos_deg.value = encoder_pos.value - wheel_offset.value
                velocity = my_drive.axis0.encoder.vel_estimate
                vel_in_rps = velocity / self.encoder_cpr
                vel_hist.append(vel_in_rps)
                if len(vel_hist) >= 20:
                    vel_hist.pop(0)
                vel_sum = 0.0
                for i in range(len(vel_hist)):
                    vel_sum += vel_hist[i]
                vel_smooth = vel_sum / len(vel_hist)
                acc_in_rpss = (vel_smooth - old_vel) * freq.value
                old_vel = vel_smooth

                # check if motor should be set to idle...
                motor_current_mode = True
                if motor_pause_mode.value:
                    if game_status_playing.value:
                        motor_current_mode = True
                    else:
                        motor_current_mode = False
                else:
                    motor_current_mode = True

                if motor_current_mode:
                    if motor_current_mode != old_motor_current_mode:
                        print('setting motor to closed_loop_control...')
                        my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                        old_motor_current_mode = motor_current_mode
                else:
                    if motor_current_mode != old_motor_current_mode:
                        print('setting motor to idle...')
                        my_drive.axis0.requested_state = AXIS_STATE_IDLE
                        old_motor_current_mode = motor_current_mode


                # if motor not idle, calculate effects and send current command to motor
                if motor_current_mode:
                    actual_max_current = actual_max_force.value*self.current_torque_factor
                    raw_signal = signal_in.value

                    # data reconstruction and smoothing
                    '''
                    # data reconstruction
                    if reconstruction.value:
                        steps_sice_last_change += 1

                        if raw_signal != signal_history[-1]:
                            signal_history.append(raw_signal)
                            signal_history.pop(0)
                            last_max_steps = steps_sice_last_change
                            steps_sice_last_change = 0

                        gradient = signal_history[-1] - signal_history[0]
                        processed_signal = signal_history[0] + (steps_sice_last_change / last_max_steps) * gradient
                    else:
                        processed_signal = raw_signal


                    # data smoothing
                    if smoothing.value != 0:
                        smoothing_history.append(processed_signal)
                        if len(smoothing_history) > smoothing.value:
                            smoothing_history.pop(0)
                        smoothed_signal = self.average_over(smoothing_history, len(smoothing_history))
                    else:
                        smoothed_signal = processed_signal
                    '''

                    final_signal_out = raw_signal   # essentially overriding the reconstruction and smoothing algorithm
                    current = final_signal_out*actual_max_current
                    if not game_status_playing.value:
                        current = 0.0

                    # calculate friction current...
                    if wheel_pos_deg.value != standstill_pos:
                        pos_diff = wheel_pos_deg.value - standstill_pos
                        if pos_diff >= 0.0:
                            direction = 1.0
                        else:
                            direction = -1.0
                        if math.fabs(pos_diff) >= pos_diff_threshold:
                            standstill_pos = wheel_pos_deg.value - pos_diff_threshold * direction
                            pos_diff = pos_diff_threshold
                        pos_diff = pos_diff / pos_diff_threshold
                        pos_diff = math.fabs(pos_diff * pos_diff * pos_diff)
                        pos_diff = pos_diff * direction
                        fric_cur = -(pos_diff) * friction.value * 5
                        current += fric_cur

                    # calculate damping current...
                    if math.fabs(vel_in_rps) >= damping_threshold:
                        if vel_in_rps >= 0.0:
                            vel = vel_in_rps - damping_threshold
                        else:
                            vel = vel_in_rps + damping_threshold
                        damp_cur = -(vel * damping.value * 10)
                        current += damp_cur

                    # calculate inertia current...
                    if math.fabs(acc_in_rpss) >= inertia_threshold:
                        if acc_in_rpss >= 0.0:
                            acc = acc_in_rpss - inertia_threshold
                        else:
                            acc = acc_in_rpss + inertia_threshold
                        inert_cur = -(acc * inertia.value)
                        current += inert_cur

                    # applying bump stop current...
                    if wheel_pos_deg.value > bump_to_bump.value / 2:
                        overshoot = wheel_pos_deg.value - (bump_to_bump.value / 2)
                        current = -overshoot*0.75
                    if wheel_pos_deg.value < -bump_to_bump.value / 2:
                        overshoot = wheel_pos_deg.value - (-bump_to_bump.value / 2)
                        current = -overshoot*0.75

                    # clamping and sending current to motor...
                    current = self.clamp(current, -actual_max_current, actual_max_current)
                    my_drive.axis0.controller.current_setpoint = current

                    motor_current.value = current
            except:
                motor_connected.value = False
                wheel_pos_deg.value = 0.0
                terminator.value = True
                print('Motor got disconnected... terminator set to true... process id:', os.getpid())
                due_for_restart.value = True


            # timing management for achieving target frequency
            end = datetime.datetime.now()
            real_delta_t = (end - start).microseconds / 1000
            sleep_time = 0.0
            combined_delta_t = real_delta_t + last_frame_overtime
            if combined_delta_t <= 1000 / freq.value:
                sleep_time = ((1000 / freq.value) - combined_delta_t)
                time.sleep(sleep_time / 1000)
                last_frame_overtime = 0.0
            else:
                last_frame_overtime += real_delta_t - (1000 / freq.value)
                #real_end = datetime.datetime.now()
                #print('art. delta_t:', (real_end - start).microseconds / 1000)
            achieved_freq.value = 1000/(combined_delta_t+sleep_time)

        try:
            my_drive.axis0.controller.current_setpoint = 0.0
            time.sleep(0.5)
            my_drive.axis0.requested_state = AXIS_STATE_IDLE
            #print('motor controller terminated... process id:', os.getpid())
        except:
            pass



    # establish connection to the ODrive board, execute the full calibration sequence
    def setup_motor(self):
        print('connecting to ODrive...')
        my_drive = odrive.find_any()
        print("starting calibration...")
        my_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while my_drive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)
        time.sleep(1)
        my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        my_drive.axis0.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        my_drive.axis0.controller.current_setpoint = 0.0
        print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")
        my_drive.axis0.motor.config.current_lim = 30.0
        #print('motor calibrated and ready...process id:', os.getpid())
        return my_drive




    # helper functions
    def set_frequency(self, value):
        self.control_freq.value = value

    def set_reconstruction(self, value):
        self.data_reconstruction.value = value

    def set_smoothing_level(self, value):
        self.data_smoothing_level.value = value

    def set_encoder_offset(self):
        self.wheel_offset.value = self.encoder_pos.value

    def terminate(self):
        self.terminator.value = True
        if not self.motor_connected.value:
            print('no motor conneceted, killing motor control process')
            try:
                self.motor_control_process.terminate()
            except:
                print('motor control process not running...')

    def average_over(self, liste, area):
        if area != 0:
            sum = 0.0
            for i in range(area - 1):
                sum += liste[-(i + 1)]
            avg = sum / area
            return avg
        elif len(liste) != 0:
            return liste[-1]
        else:
            return 0.0

    def clamp(self, value, min, max):
        if value <= min:
            value = min
        elif value >= max:
            value = max
        return value