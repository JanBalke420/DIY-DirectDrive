# class for internal representation of a profile
class Profile:
    name = ''

    # all settings that can be stored in a profile
    lock_to_lock = 0
    bump_to_bump = 0
    invert_force = 0
    max_force = 0.0
    use_reconstruction = 0
    smoothing_level = 0
    friction = 0.0
    damping = 0.0
    inertia = 0.0

    def __init__(self, name):
        self.name = name

    def __str__(self):
        print_str = ''

        print_str += 'name: '+self.name
        print_str += '\n'

        print_str += 'lock_to_lock: '+str(self.lock_to_lock)
        print_str += '\n'
        print_str += 'bump_to_bump: '+str(self.bump_to_bump)
        print_str += '\n'

        print_str += 'invert_force: '+str(self.invert_force)
        print_str += '\n'

        print_str += 'max_force: '+str(self.max_force)
        print_str += '\n'

        print_str += 'use_reconstruction: '+str(self.use_reconstruction)
        print_str += '\n'
        print_str += 'smoothing_level: '+str(self.smoothing_level)
        print_str += '\n'

        print_str += 'friction: '+str(self.friction)
        print_str += '\n'
        print_str += 'damping: '+str(self.damping)
        print_str += '\n'
        print_str += 'inertia: ' + str(self.inertia)
        print_str += '\n'

        return print_str