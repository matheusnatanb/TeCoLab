import numpy as np
import control
import matplotlib.pyplot as plt
import math

class Controller:
    def __init__(self, T=1, sampling_time:int = 0.1):
        self.sampling_time = sampling_time
        self.counter = 0
        numerator = [1]
        denominator = [1, 2, 1]
        tf_system = control.TransferFunction(numerator, denominator)
        system_continuous_ss = control.tf2ss(tf_system.num, tf_system.den)
        system_continuous_ss.initial_state = [0,0]

        self.last_state = [[0, 0],[0,0]]
        self.system_discrete = control.c2d(system_continuous_ss, sampling_time, method='tustin')
        self.lastControlAction = (0, 0, 0)

    def simulate(self, setpoint: float, duration: int = 100):
        num_steps = int(duration / self.sampling_time)
        t = np.arange(num_steps + 1) * self.sampling_time

        u = np.ones(num_steps + 1) * setpoint  # Step input
        _, y_discrete, _ = control.forced_response(self.system_discrete, T=t, U=u, return_x=True)

        return t, y_discrete

    def plot_response(self, setpoint: float, duration: int = 100):
        t, y = self.simulate(setpoint, duration)

        plt.figure(figsize=(10, 6))
        plt.step(t, y, label=f'Controlled System (Setpoint={setpoint})', where='post')
        plt.xlabel('Time [s]')
        plt.ylabel('Output')
        plt.title(f'Discrete Controlled System Response to Step Input (Setpoint={setpoint})')
        plt.legend()
        plt.grid(True)
        plt.show()

    def _control_action(self, set_points, temperatures):
        sp1_abs, sp2_abs, sp1_rel, sp2_rel = set_points[:4]
        tem_He1, tem_He2, tem_Amb = temperatures[:3]

        sp1_abs = 0 if math.isnan(sp1_abs) else sp1_abs
        sp2_abs = 0 if math.isnan(sp2_abs) else sp2_abs
        sp1_rel = 0 if math.isnan(sp1_rel) else sp1_rel
        sp2_rel = 0 if math.isnan(sp2_rel) else sp2_rel

        H1PWM = 0
        H2PWM = 0
        CoolerPWM = 0

		# Control algorithm computation
        error = sp1_rel - ((tem_He1 + tem_He2)/2 - tem_Amb)
        (_, output, self.last_state) = control.forced_response(self.system_discrete, T = [0, 1], U = [error, error], X0 = [self.last_state[0][1], self.last_state[1][1]], return_x = True)

        H1PWM = output[0]
        H2PWM = output[0]
        CoolerPWM = 0

        # Control algorithm return
        return H1PWM, H2PWM, CoolerPWM

    def engine(self, set_points, temperatures):
        self.counter += 1
        if self.counter >= 0:
            self._reset_counter()
            self.last_control_action = self._control_action(set_points, temperatures)
            return self.last_control_action, +1 # +1 represents new control action
        else:
            return self.last_control_action, 0 # 0 represents no new control action

    def _reset_counter(self):
        self.counter = -self.sampling_time

ct = Controller()
ct.plot_response(30)