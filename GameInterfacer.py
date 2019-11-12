import time
import datetime
from multiprocessing import Process, Value

# import the structs for Shared Memory Mapping for Assetto Corsa and Assetto Corsa Competizione
import AssettoCorsaMemoryMapping
import AssettoCorsaCompMemoryMapping




# class for handling the interfacing with different games
class GameInterfacer:
    games_available = ['None', 'Assetto Corsa', 'Assetto Corsa Competizione']       # holds list of all supported games
    game_interface_process = None                                                   # holds an instance of the Process-class
                                                                                    # for sending a virtual controller signal



    def __init__(self):
        self.terminator = Value('b', False)                 # indicator for terminating the loop inside the interfacing functions
        self.control_freq = Value('i', 30)                  # target refresh rate for the interfacing process
        self.wheel_force = Value('f', 0.0)                  # stores the raw force-feedback value coming from the game
        self.invert_force = Value('b', False)               # determines whether the signal from the game should be inverted or not
        self.game_status_playing = Value('b', False)        # game status (playing/not-playing)



    def start(self, game_index):
        if game_index == 1:
            print('start interfaceing process for:', self.games_available[game_index])
            self.game_interface_process = Process(target=self.interface_assetto_corsa,
                                                  args=(self.terminator,
                                                        self.control_freq,
                                                        self.wheel_force,
                                                        self.invert_force,
                                                        self.game_status_playing))
            self.terminator.value = False
            self.game_interface_process.start()
        elif game_index == 2:
            print('start interfaceing process for:', self.games_available[game_index])
            self.game_interface_process = Process(target=self.interface_assetto_corsa_comp,
                                                  args=(self.terminator,
                                                        self.control_freq,
                                                        self.wheel_force,
                                                        self.invert_force,
                                                        self.game_status_playing))
            self.terminator.value = False
            self.game_interface_process.start()




    # FUNCTIONS FOR INTERFACING WITH DIFFERENT GAMES
    # these functions are referenced in the start()-function of this class

    # function for interfacing with Assetto Corsa
    def interface_assetto_corsa(self, terminator, freq, wheel_force, invert_force, game_status_playing):
        assetto_corsa = AssettoCorsaMemoryMapping.SimInfo()
        print('connecting to Assetto Corsa')

        last_frame_overtime = 0.0
        while not terminator.value:
            start = datetime.datetime.now()
            #print('listening to assetto corsa')





            force = assetto_corsa.physics.finalFF/2
            if invert_force.value:
                wheel_force.value = -force
            else:
                wheel_force.value = force

            if assetto_corsa.graphics.status == 2:
                game_status_playing.value = True
            else:
                game_status_playing.value = False





            end = datetime.datetime.now()
            real_delta_t = (end - start).microseconds / 1000
            combined_delta_t = real_delta_t + last_frame_overtime
            if combined_delta_t <= 1000 / freq.value:
                sleep_time = ((1000 / freq.value) - combined_delta_t)
                time.sleep(sleep_time / 1000)
                last_frame_overtime = 0.0
            else:
                last_frame_overtime += real_delta_t - (1000 / freq.value)
                #real_end = datetime.datetime.now()
                # print('art. delta_t:', (real_end - start).microseconds / 1000)

        wheel_force.value = 0.0
        game_status_playing.value = False
        print('disconnecting from Assetto Corsa')

    # function for interfacing with Assetto Corsa Competizione
    def interface_assetto_corsa_comp(self, terminator, freq, wheel_force, invert_force, game_status_playing):
        assetto_corsa_comp = AssettoCorsaCompMemoryMapping.SimInfo()
        print('connecting to Assetto Corsa Competizione')

        ff_hist = []
        ff_hist_limit = 5

        last_frame_overtime = 0.0
        while not terminator.value:
            start = datetime.datetime.now()
            #print('listening to assetto corsa competizione')





            force = assetto_corsa_comp.physics.finalFF/2
            if invert_force.value:
                wheel_force.value = -force
            else:
                wheel_force.value = force

            ff_hist.append(force)
            if len(ff_hist) >= ff_hist_limit:
                ff_hist.pop(0)
            if sum(ff_hist) != 0.0:
                game_status_playing.value = True
            else:
                game_status_playing.value = False





            end = datetime.datetime.now()
            real_delta_t = (end - start).microseconds / 1000
            combined_delta_t = real_delta_t + last_frame_overtime
            if combined_delta_t <= 1000 / freq.value:
                sleep_time = ((1000 / freq.value) - combined_delta_t)
                time.sleep(sleep_time / 1000)
                last_frame_overtime = 0.0
            else:
                last_frame_overtime += real_delta_t - (1000 / freq.value)
                #real_end = datetime.datetime.now()
                # print('art. delta_t:', (real_end - start).microseconds / 1000)

        wheel_force.value = 0.0
        game_status_playing.value = False
        print('disconnecting from Assetto Corsa Competizione')






    # helper functions
    def set_frequency(self, value):
        self.control_freq.value = value

    def terminate(self):
        self.terminator.value = True
        try:
            print('trying to kill interfacing process')
            self.game_interface_process.terminate()
        except:
            print('interfacing process already terminated')