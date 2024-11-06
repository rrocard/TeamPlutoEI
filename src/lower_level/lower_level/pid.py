# coding: utf-8

"""
PID class.

It computes the command given the gains and the estimated derivate and integral
of the error.

The equations of the cart pole example are taken from
"Correct equations for the dynamics of the cart-pole system" R.V. Florian(2005)
https://coneural.org/florian/papers/05_cart_pole.pdf
"""

import datetime
import numpy as np

class PID:

    def __init__(self, gains):
        '''
        Builds a PID

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
            raise RuntimeError('''Some keys are missing in the gains, '''
                               '''we expect Kp, Kd and Ki''')


    def reset(self):
        '''
        Reset the errors of the PID
        '''
        self.errors = {'error': 0.,
                       'd_error': 0.,
                       'i_error': 0.,
                       'time': None
                      }

    def update(self,
               time: datetime.datetime,
               error: float):
        '''
        Update the error, its derivative and integral
        '''
        prev_error = self.errors['error']
        prev_time = self.errors['time']
        if prev_time is None:
            self.errors['error'], self.errors['time'] = error, time
            return

        self.errors['error'] = error
        self.errors['time'] = time
        dt = (time - prev_time).total_seconds()
        self.errors['i_error'] += dt * (prev_error + error)/2.0
        self.errors['d_error'] = (error - prev_error)/dt

    @property
    def command(self):
        return self.gains['Kp'] * self.errors['error'] + \
                self.gains['Kd'] * self.errors['d_error'] + \
                self.gains['Ki'] * self.errors['i_error']


if __name__ == '__main__':
    
    import matplotlib.pyplot as plt

    # Illustration of the PID on the inverted pendulum
    dt = 0.02  # s.
    g = 9.81  # kg/s^-2
    l = 0.5   # m.
    mc = 1.0  # kg
    mp = 0.1  # kg
    mT = mc + mp
    muc = 5*1e-4
    mup = 2*1e-6

    # The equations for the dynamics of the system
    def f_Nc(theta, dtheta, ddtheta):
        ct, st = np.cos(theta), np.sin(theta)
        return mT*g-mp*l*(ddtheta*st+dtheta**2*ct)

    def f_ddtheta(F, theta, dtheta, Nc, dx):
        ct, st = np.cos(theta), np.sin(theta)
        return (g*st+ct*((-F-mp*l*dtheta**2*(st+muc*np.sign(Nc*dx)*ct))/mT+muc*g*np.sign(Nc*dx))-mup*dtheta/(mp*l))/(l*(4./3.-mp*ct/mT*(ct-muc*np.sign(Nc*dx))))

    def f_ddx(F, theta, dtheta, ddtheta, Nc, x, dx):
        ct, st = np.cos(theta), np.sin(theta)
        return (F+mp*l*(dtheta**2*st-ddtheta*ct)-muc*Nc*np.sign(Nc*dx))/mT

    def f_dX(F, X, ddtheta):
        """
        X is the state [theta, dtheta, x, dx]
        """
        theta, dtheta, x, dx = X
        Nc = f_Nc(theta, dtheta, ddtheta)
        ddtheta = f_ddtheta(F, theta, dtheta, Nc, dx)
        Nc_2 = f_Nc(theta, dtheta, ddtheta)
        if np.sign(Nc_2) != np.sign(Nc):
            Nc = Nc_2
            ddtheta = f_ddtheta(F, theta, dtheta, Nc, dx)

        ddx = f_ddx(F, theta, dtheta, ddtheta, Nc, x, dx)

        return np.array([
            dtheta,
            ddtheta,
            dx,
            ddx
        ])

    # Instantiate our PID with hand tuned gains
    gains = {'Kp': 50, 'Kd': 10, 'Ki': 50.0}
    pid = PID(gains)
    # Define the target for the angle of the pole
    target_theta = -0.6

    # Define the initial state
    X = np.array([0.1, 0.0, 1, 0])
    ddtheta = 0
    F = 0
    t = datetime.datetime.now()
    # We will simulate for 5 sec
    tmax = t + datetime.timedelta(seconds=5.0)
    dt = datetime.timedelta(milliseconds=dt*1000)
    history = np.append(X, [F, t]).reshape((1, 6))
    
    # Let us go for the simulation
    while t < tmax:

        # Update the PID
        theta = X[0]
        pid.update(t, theta - target_theta)

        # Get the force to apply
        F = pid.command

        # Integrate the dynamics with Euler
        dX = f_dX(F, X, ddtheta)
        X += dt.total_seconds() * dX
        ddtheta = dX[1]
        history = np.vstack([history, np.append(X, [F, t])])
        t += dt

    # Plot the results
    titles = [r'$\theta$', r'$\dot{\theta}$', r'$x$', r'$\dot{x}$', r'$F$']
    fig, axes = plt.subplots(1, 5, figsize=(20, 5))
    for i in range(5):
        ax = axes[i]
        ax.plot(history[:, 5], history[:, i])
        ax.set_title(titles[i])
        plt.gcf().autofmt_xdate()
    # On the first plot, plot the target
    ax = axes[0]
    ax.axhline(y=target_theta, linestyle='--', color='tab:red')

    plt.show()