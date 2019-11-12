import pyxinput
import time
import datetime
from multiprocessing import Process, Value

class VirtualController:
    send_input_process = None           # holds an instance of the Process-class for sending a virtual controller signal



    def __init__(self):
        self.terminator = Value('b', False)             # indicator for terminating the loop inside the function 'send_input'
        self.control_freq = Value('i', 120)             # target refresh rate for the virtual-controller process
        self.steering_value_deg = Value('f', 0.0)       # wheel position in degrees
        self.lock_to_lock = Value('f', 900.0)           # holds the digital limit of the wheel (used here to scale the
                                                        # wheel-position to the output range of the virtual controller)



    def start(self):
        self.send_input_process = Process(target=self.send_input,
                                              args=(self.terminator, self.control_freq, self.steering_value_deg, self.lock_to_lock))
        self.send_input_process.start()

    def send_input(self, terminator, freq, steering_value_deg, lock_to_lock):
        print('creating virtual xinput controller')
        virtualSteeringWheel = pyxinput.vController()
        virtualSteeringWheel.percent = True
        virtualSteeringWheel.set_value('AxisLx', 0.0)

        last_frame_overtime = 0.0
        while not terminator.value:
            start = datetime.datetime.now()

            steering = steering_value_deg.value/(lock_to_lock.value/2)
            steering = self.clamp(steering, -1.0, 1.0)
            virtualSteeringWheel.set_value('AxisLx', steering)

            # timing management in order to achieve target frequency
            end = datetime.datetime.now()
            real_delta_t = (end - start).microseconds / 1000
            combined_delta_t = real_delta_t + last_frame_overtime
            if combined_delta_t <= 1000 / freq.value:
                sleep_time = ((1000 / freq.value) - combined_delta_t)
                time.sleep(sleep_time / 1000)
                last_frame_overtime = 0.0
            else:
                last_frame_overtime += real_delta_t - (1000 / freq.value)
                real_end = datetime.datetime.now()
                # print('art. delta_t:', (real_end - start).microseconds / 1000)

        virtualSteeringWheel.set_value('AxisLx', 0.0)
        virtualSteeringWheel.UnPlug(True)
        print('virtual xinput controller unplugged')



    # helper functions
    def set_frequency(self, value):
        self.control_freq.value = value

    def terminate(self):
        self.terminator.value = True

    def clamp(self, value, min, max):
        if value <= min:
            value = min
        elif value >= max:
            value = max
        return value