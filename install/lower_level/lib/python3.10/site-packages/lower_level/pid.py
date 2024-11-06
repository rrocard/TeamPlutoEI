import time  # Use time module for float-based time
import numpy as np

class PID:
    def __init__(self, gains):
        '''
        Builds a PID controller

        Arguments:
            gains (dict): with keys Kp, Kd, Ki
        '''
        self.gains = gains.copy()
        self.errors = {'error': 0.,
                       'd_error': 0.,
                       'i_error': 0.,
                       'time': None
                      }
        if not ('Kp' in gains and 'Kd' in gains and 'Ki' in gains):
            raise RuntimeError('Some keys are missing in the gains, we expect Kp, Kd, and Ki')

    def reset(self):
        '''
        Reset the errors of the PID controller
        '''
        self.errors = {'error': 0.,
                       'd_error': 0.,
                       'i_error': 0.,
                       'time': None
                      }

    def update(self, current_time: float, error: float):
        '''
        Update the error, its derivative and integral.
        current_time should be a float (seconds since epoch).
        '''
        prev_error = self.errors['error']
        prev_time = self.errors['time']

        # If this is the first update (previous time is None), initialize it
        if prev_time is None:
            self.errors['error'], self.errors['time'] = error, current_time
            return

        self.errors['error'] = error
        self.errors['time'] = current_time

        dt = current_time - prev_time  # Time difference in seconds (float)
        
        # Avoid division by zero if dt is too small
        if dt <= 0:
            dt = 1e-6

        # Integral calculation using trapezoidal rule for better accuracy
        self.errors['i_error'] += dt * (prev_error + error) / 2.0
        self.errors['d_error'] = (error - prev_error) / dt

    @property
    def command(self):
        '''
        Compute the PID control output.
        '''
        return self.gains['Kp'] * self.errors['error'] + \
               self.gains['Kd'] * self.errors['d_error'] + \
               self.gains['Ki'] * self.errors['i_error']

# Example usage or simulation code (not necessary for ROS)
if __name__ == '__main__':
    # Example of using this PID class in a simple setup
    dt = 0.02  # Time step (seconds)
    gains = {'Kp': 50, 'Kd': 10, 'Ki': 50.0}
    pid = PID(gains)
    target_value = 1.0  # Example target
    current_value = 0.0  # Initial state
    
    # Simulate the PID loop for a few steps
    for i in range(100):
        current_time = time.time()  # Current time (float)
        
        # Compute the error (target - current state)
        error = target_value - current_value
        
        # Update the PID controller with the new error
        pid.update(current_time, error)
        
        # Get the control output (PID command)
        control_output = pid.command
        print(f"Control Output: {control_output}")
        
        # Simulate some state change (for example, moving towards the target)
        current_value += control_output * dt
        
        # Sleep for the time step to simulate real-time
        time.sleep(dt)
