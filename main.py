from vex import *
import math

brain = Brain()
LM1 = Motor(Ports.PORT11, GearSetting.RATIO_6_1, False)
LM2 = Motor(Ports.PORT12, GearSetting.RATIO_18_1, True)
LM3 = Motor(Ports.PORT13, GearSetting.RATIO_6_1, True)
RM1 = Motor(Ports.PORT14, GearSetting.RATIO_6_1, True)
RM2 = Motor(Ports.PORT15, GearSetting.RATIO_18_1, False)
RM3 = Motor(Ports.PORT16, GearSetting.RATIO_6_1, False)

L = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
R = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
LR = Rotation(Ports.PORT3, False)

C = Motor(Ports.PORT4, GearSetting.RATIO_36_1, False)

T = DigitalOut(brain.three_wire_port.a)

O = Rotation(Ports.PORT19, True)
I = Inertial(Ports.PORT20)
V = AiVision(Ports.PORT5)

CTRL = Controller(PRIMARY)

x = 0
y = 0
sensitivity = 1
previous_angle = 0
def enum(iterable): #enumerate() disallowed in VEX Python
    index = 0
    for item in iterable:
        yield index, item
        index += 1

def screen_status(txt: str, row=1):
    CTRL.screen.clear_row(row)
    CTRL.screen.set_cursor(row,1)
    CTRL.screen.print(txt)


#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


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
Lifter = PID(0, 0, 0)

def distance(prev):
    return (O.position() - prev) * math.pi / 180

def drive(target_distance: float, max_voltage: float = 10, correction_rate: float = 0.25, timeout: int = 10000):
    Driver.reset()
    start_time = brain.timer.time(MSEC)
    start_pos = O.position()
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

def lift(target_height: float, max_voltage: float = 10, timeout: int = 10000):
    Lifter.reset()
    start_time = brain.timer.time(MSEC)
    start_pos = LR.position()
    while brain.timer.time(MSEC) - start_time < timeout:
        error = target_height + start_pos - LR.position()
        if (error < 0.5 and target_height > 0) or (error > -0.5 and target_height < 0): # change error capacities later
            break
        pid_output = Lifter.calculate(error)
        motor_voltage = max(-max_voltage, min(max_voltage, pid_output))
        for M in [L, R]:
            M.spin(FORWARD, motor_voltage, VOLT)
        wait(10, MSEC)
        for M in [L, R]:
            M.stop()
    wait(150, MSEC)
    print("Final Error:", target_height + start_pos - LR.position())


#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



def toggle():
    T.set(not T.value())

def soften(x): #turn a float between -1 and 1 into a dumb thing
    return x
    #return math.sin(x * math.pi / 2) ** 3

def control():
    global previous_angle, x, y
    speedl = (soften(CTRL.axis3.position() / 100) + soften(CTRL.axis1.position() / 100)) / (0.166 / sensitivity)
    speedr = (soften(CTRL.axis3.position() / 100) - soften(CTRL.axis1.position() / 100)) / (0.166 / sensitivity)
    for LM in [LM1, LM2, LM3]:
        LM.spin(FORWARD, speedl, VOLT)
    for RM in [RM1, RM2, RM3]:
        RM.spin(FORWARD, speedr, VOLT)
    for M in [L, R]:
        if CTRL.buttonL1.pressing():
            M.spin(FORWARD, 12, VOLT)
        elif CTRL.buttonL2.pressing():
            M.spin(REVERSE, 3, VOLT)
        else:
            M.stop(HOLD)
    if CTRL.buttonR1.pressing():
        C.spin(FORWARD, 12, VOLT)
    elif CTRL.buttonR2.pressing():
        C.spin(REVERSE, 12, VOLT)
    else:
        C.stop()
    CTRL.buttonA.pressed(toggle)
    if speedl == speedr:
        dist = distance(previous_angle)
        slope = math.tan(math.radians(I.rotation()))
        y += dist / math.sqrt(1 + slope ** 2)
        x += dist * slope / math.sqrt(1 + slope ** 2)
    previous_angle = O.position()
    wait(10, MSEC)

def goal_nav():
    err_rotation = float('inf')
    while abs(err_rotation) > 8:
        objects = V.take_snapshot(V.ALL_TAGS)
        if len(objects) == 0:
            return None
        target = max(objects, key=lambda obj: obj.area)
        err_rotation = target.centerX - 157
        #screen_status(f"ID:{target.id} X:{target.centerX} Y:{target.centerY}")
        for LM in [LM1, LM2, LM3]:
            LM.spin(FORWARD, 0.25 * err_rotation, VOLT)
        for RM in [RM1, RM2, RM3]:
            RM.spin(REVERSE, 0.25 * err_rotation, VOLT)
    while err_dist > 20:
        objects = V.take_snapshot(V.ALL_TAGS)
        if len(objects) == 0:
            return None
        target = max(objects, key=lambda obj: obj.area)
        err_dist = 100 - target.area
        #screen_status(f"ID:{target.id} X:{target.centerX} Y:{target.centerY}")
        for M in [LM1, LM2, LM3, RM1, RM2, RM3]:
            M.spin(FORWARD, 0.25 * err_dist, VOLT)




def autonomous():
    brain.screen.clear_screen()
    drive(6, 6)
    rotate(180, 6)
    drive(6, 6)
    drive(-6, 6)
    drive(6, 6)
    drive(-6, 6)
    rotate(90, 6)
    drive(12, 6)

def user_control():
    while True:
        control()
        screen_status(str(x), 1)
        screen_status(str(y), 2)
        objects = V.take_snapshot(V.ALL_TAGS)
        for tag in objects:
            pass
            #screen_status(f"ID:{tag.id} X:{tag.centerX} Y:{tag.centerY}")
        wait(10, MSEC)

I.calibrate() #Calibrate Inertial sensor
while I.is_calibrating():
    screen_status("Calibrating")
    wait(100)
screen_status("Calibration Complete")

comp = Competition(user_control, autonomous)
brain.screen.clear_screen()
