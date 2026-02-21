import time

class PID:
    def __init__(self, kp, ki, kd, target=0.0, tol = 0.1,
                 pwm_min=1100, pwm_max = 1900, pwm_trim = 100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.target = target
        self.tol = tol
        
        self.integral = 0.0
        self.prev_error = 0.0
        
        self.prev_time = time.monotonic()
        self.pwm_min = pwm_min
        self.pwm_max = pwm_max
        self.pwm_trim = pwm_trim

        self.integral_lim = 500

    def update_target(self, target):
        self.target = target
    
    def update_tol(self, tol):
        self.tol = tol

    def atTarget(self, measurement):
        if(measurement  == self.target + self.tol or measurement == self.target - self.tol):
            return True
        else:
            return False

    def update(self, measurement):
        # Time step
        current_time = time.monotonic()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        if dt <= 0.0:
            return 0.0

        # Error
        error = self.target - measurement

        # Integral
        self.integral += error * dt

        # Derivative
        derivative = (error - self.prev_error) / dt

        # PID output
        b = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )
        
        pwm = self.pwm_trim + b
        #Keep it between 1100, 1900
        pwm = max(min(pwm, self.pwm_max), self.pwm_min)

        # Store for next step
        self.prev_error = error
        # return output
        return int(pwm)