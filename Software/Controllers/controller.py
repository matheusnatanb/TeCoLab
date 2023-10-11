'''
Copyright 2023 Leonardo Cabral

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

import math
import control

class Controller:
    """
    A simple controller class for a discrete-time control system.

    Parameters:
    - numerator (list): The numerator coefficients of the transfer function.
    - denominator (list): The denominator coefficients of the transfer function.
    - sampling_time (int): The sampling time for the discrete-time control system.
    - method (str): The method used for discretization, default is 'tustin'.

    Attributes:
    - sampling_time (int): The sampling time for the discrete-time control system.
    - counter (int): Counter for controlling when to perform a new control action.
    - last_state (list): The last state of the control system.
    - last_control_action (tuple): The last control action applied.
    - system (control.StateSpaceDiscrete): The discrete-time control system.

    Methods:
    - run_control(set_points, temperatures): Runs the control algorithm and returns the control action.
    """
    def __init__(self, numerator:list = [1], denominator:list = [1, 2, 1], sampling_time:int = 0.2, method='tustin'):
        """
        Initializes the Controller object.

        Parameters:
        - numerator (list): The numerator coefficients of the transfer function.
        - denominator (list): The denominator coefficients of the transfer function.
        - sampling_time (int): The sampling time for the discrete-time control system.
        - method (str): The method used for discretization, default is 'tustin'.
        """
        self.sampling_time = sampling_time

        self.counter = 0
        
        self.last_control_action = (0, 0, 0)
        self.system = self._get_system(numerator, denominator, sampling_time, method)
        self.last_state = [[0, 0]] * len(self.system.A)


    def run_control(self, set_points, temperatures):
        """
        Runs the control algorithm and returns the control action.

        Parameters:
        - set_points (list): List of set points for the control algorithm.
        - temperatures (list): List of temperature values for the control algorithm.

        Returns:
        Tuple: The control action and a flag indicating if a new control action was performed.
        """

        self.counter += 1
        if self.counter >= 0:
            self._reset_counter()
            self.last_control_action = self._control_action(set_points, temperatures)
            return self.last_control_action, +1 # +1 represents new control action
        else:
            return self.last_control_action, 0 # 0 represents no new control action

    def _get_system(self, numerator, denominator, sampling_time, method):
        """
        Returns the discrete-time control system.

        Parameters:
        - numerator (list): The numerator coefficients of the transfer function.
        - denominator (list): The denominator coefficients of the transfer function.
        - sampling_time (int): The sampling time for the discrete-time control system.
        - method (str): The method used for discretization.

        Returns:
        control.StateSpaceDiscrete: The discrete-time control system.
        """

        tf_system = control.TransferFunction(numerator, denominator)
        system_continuous_ss = control.tf2ss(tf_system.num, tf_system.den)
        system_continuous_ss.initial_state = [0,0]
        system_discrete = control.c2d(system_continuous_ss, sampling_time, method)

        return system_discrete

    def _reset_counter(self):
        """Resets the counter for controlling when to perform a new control action."""

        self.counter = -self.sampling_time

    def _control_action(self, set_points, temperatures):
        """
        Computes the control action based on set points and temperatures.

        Parameters:
        - set_points (list): List of set points for the control algorithm.
        - temperatures (list): List of temperature values for the control algorithm.

        Returns:
        Tuple: The computed control actions.
        """

        sp1_abs, sp2_abs, sp1_rel, sp2_rel = set_points[:4]
        temp_he1, temp_he2, temp_amb = temperatures[:3]

        sp1_abs, sp2_abs, sp1_rel, sp2_rel = (0 if math.isnan(sp) else sp for sp in set_points[:4])

        h1_pwm = h2_pwm = coller_pwm = 0

        error = sp1_rel - (temp_he1 - temp_amb)

        X0 = [ls[1] for ls in self.last_state]


        (_, output, self.last_state) = control.forced_response(self.system, T = [0, 1], U = [error, error], X0 = X0, return_x = True)

        h1_pwm = h2_pwm = output[0]
        coller_pwm = 0

        return h1_pwm, h2_pwm, coller_pwm
