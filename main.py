from vex import *
import math

brain = Brain()
LM1 = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
LM2 = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
RM1 = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
RM2 = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False)

R1 = Rotation(Ports.PORT2, True)
I = Inertial(Ports.PORT4)
V = AiVision(Ports.PORT5)

C = Controller(PRIMARY)

x = 0
y = 0
sensitivity = 2
previous_angle = 0
objects = V.take_snapshot(V.ALL_TAGS)
def enum(iterable): #enumerate() disallowed in VEX Python
    index = 0
    for item in iterable:
        yield index, item
        index += 1

def screen_status(txt: str, row=1):
    C.screen.clear_row(row)
    C.screen.set_cursor(row,1)
    C.screen.print(txt)

def distance(prev):
    return (R1.position() - prev) * math.pi / 180

class PID:
    def __init__(self, accel: float, inertia: float, drag: float, max_integral: float = 50):
        self.kp = accel
        self.ki = inertia
        self.kd = drag
        self.max_integral = max_integral
        self.previous_error = 0
        self.integral = 0
    
    def calculate(self, error, dt=0.01):
        p_term = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.previous_error) / dt if dt > 0 else 0
        self.previous_error = error
        return p_term + i_term + d_term

    def reset(self):
        self.previous_error = 0
        self.integral = 0

Driver = PID(1.85, 0, 0.03)
Rotater = PID(0.325, 0, 0.05)

def drive(target_distance: float, max_voltage: float = 10, correction_rate: float = 0.25, timeout: int = 10000):
    Driver.reset()
    start_time = brain.timer.time(MSEC)
    start_pos = R1.position()
    start_rotation = I.rotation()
    while brain.timer.time(MSEC) - start_time < timeout:
        error = target_distance - distance(start_pos)
        if (error < 0.5 and target_distance > 0) or (error > -0.5 and target_distance < 0):
            break
        pid_output = Driver.calculate(error)
        motor_voltage = max(-max_voltage, min(max_voltage, pid_output))
        for LM in [LM1, LM2]:
            LM.spin(FORWARD, motor_voltage - correction_rate * (I.rotation() - start_rotation), VOLT)
        for RM in [RM1, RM2]:
            RM.spin(FORWARD, motor_voltage + correction_rate * (I.rotation() - start_rotation), VOLT)
        wait(10, MSEC)
        for motor in [LM1, LM2, RM1, RM2]:
            motor.stop()
    wait(150, MSEC)
    print("Final Error:", target_distance - distance(start_pos))

def rotate(target_degrees: float, max_voltage: float = 10, timeout: int = 10000):
    Rotater.reset()
    start_time = brain.timer.time(MSEC)
    start_rotation = I.rotation()
    while brain.timer.time(MSEC) - start_time < timeout:
        error = target_degrees + start_rotation - I.rotation()
        if (error < 7 and target_degrees > 0) or (error > -7 and target_degrees < 0):
            break
        pid_output = Rotater.calculate(error)
        motor_voltage = max(-max_voltage, min(max_voltage, pid_output))
        for LM in [LM1, LM2]:
            LM.spin(FORWARD, motor_voltage, VOLT)
        for RM in [RM1, RM2]:
            RM.spin(REVERSE, motor_voltage, VOLT)
        wait(10, MSEC)
        for motor in [LM1, LM2, RM1, RM2]:
            motor.stop()
    wait(150, MSEC)
    print("Final Error:", target_degrees + start_rotation - I.rotation())

def soften(x): #turn a float between -1 and 1 into a dumb thing
    return math.sin(x * math.pi / 2) ** 3

def control():
    global previous_angle, x, y
    speedl = (soften(C.axis3.position() / 100) + soften(C.axis1.position() / 100)) / (0.166 / sensitivity)
    speedr = (soften(C.axis3.position() / 100) - soften(C.axis1.position() / 100)) / (0.166 / sensitivity)
    for LM in [LM1, LM2]:
        LM.spin(FORWARD, speedl, VOLT)
    for RM in [RM1, RM2]:
        RM.spin(FORWARD, speedr, VOLT)
    if speedl == speedr:
        dist = distance(previous_angle)
        slope = math.tan(math.radians(I.rotation()))
        y += dist / math.sqrt(1 + slope ** 2)
        x += dist * slope / math.sqrt(1 + slope ** 2)
    previous_angle = R1.position()
    wait(10, MSEC)

def autonomous():
    brain.screen.clear_screen()
    drive(24, 6)
    rotate(90, 6)
    drive(6, 6)
    rotate(-90, 6)
    drive(24, 6)

def user_control():
    while True:
        control()
        screen_status(str(x), 1)
        screen_status(str(y), 2)
        wait(10, MSEC)

I.calibrate() #Calibrate Inertial sensor
while I.is_calibrating():
    screen_status("Calibrating")
    wait(100)
screen_status("Calibration Complete")

comp = Competition(user_control, autonomous)
brain.screen.clear_screen()
