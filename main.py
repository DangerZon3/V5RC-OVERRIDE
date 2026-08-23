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

T = DigitalOut(brain.three_wire_port.a)
C = DigitalOut(brain.three_wire_port.b)

O = Rotation(Ports.PORT19, True)
I = Inertial(Ports.PORT20)
V = AiVision(Ports.PORT5)

CTRL = Controller(PRIMARY)

x = 0
y = 0
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

Driver = PID(1.3, 0, 0.05)
Rotater = PID(0.5, 0, 0.05)
Lifter = PID(0.1, 0, 0.01)

def distance(prev):
    return (O.position() - prev) * math.pi / 180

def drive(target_distance: float, max_voltage: float = 10, correction_rate: float = 0.1, timeout: int = 10000):
    Driver.reset()
    start_time = brain.timer.time(MSEC)
    start_pos = O.position()
    start_rotation = I.rotation()
    while brain.timer.time(MSEC) - start_time < timeout:
        error = target_distance - distance(start_pos)
        if -0.2 < error < 0.2:
            break
        pid_output = Driver.calculate(error)
        motor_voltage = max(-max_voltage, min(max_voltage, pid_output))
        for LM in [LM1, LM2, LM3]:
            LM.spin(FORWARD, motor_voltage - correction_rate * (I.rotation() - start_rotation), VOLT)
        for RM in [RM1, RM2, RM3]:
            RM.spin(FORWARD, motor_voltage + correction_rate * (I.rotation() - start_rotation), VOLT)
        wait(10, MSEC)
        for motor in [LM1, LM2, RM1, RM2]:
            motor.stop()
        print("DRIVE voltage:", motor_voltage)
    wait(150, MSEC)
    print("Final Error:", target_distance - distance(start_pos))

def rotate(target_degrees: float, max_voltage: float = 10, timeout: int = 10000):
    Rotater.reset()
    start_time = brain.timer.time(MSEC)
    start_rotation = I.rotation()
    while brain.timer.time(MSEC) - start_time < timeout:
        error = target_degrees + start_rotation - I.rotation()
        if -3 < error < 3:
            break
        pid_output = Rotater.calculate(error)
        motor_voltage = max(-max_voltage, min(max_voltage, pid_output))
        for LM in [LM1, LM2, LM3]:
            LM.spin(FORWARD, motor_voltage, VOLT)
        for RM in [RM1, RM2, RM3]:
            RM.spin(REVERSE, motor_voltage, VOLT)
        wait(10, MSEC)
        for motor in [LM1, LM2, RM1, RM2]:
            motor.stop()
        print("ROTATE voltage:", motor_voltage)
    wait(150, MSEC)
    print("Final Error:", target_degrees + start_rotation - I.rotation())

def lift(target_height: float, max_voltage: float = 7, timeout: int = 10000):
    Lifter.reset()
    start_time = brain.timer.time(MSEC)
    start_pos = L.position()
    while brain.timer.time(MSEC) - start_time < timeout:
        error = target_height + start_pos - L.position()
        if -40 < error < 40: # change error capacities later
            break
        pid_output = Lifter.calculate(error)
        motor_voltage = max(-max_voltage, min(max_voltage, pid_output))
        for M in [L, R]:
            M.spin(FORWARD, motor_voltage, VOLT)
        wait(10, MSEC)
        for M in [L, R]:
            M.stop(HOLD)
        print("LIFT voltage:", motor_voltage)
    wait(150, MSEC)
    print("Final Error:", target_height + start_pos - L.position())


#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


class POI:
    def __init__(self, cx: float, cy: float, width: float, height: float):
        self.cx, self.cy = cx, cy
        self.x1, self.y1, self.x2, self.y2 = cx - (width / 2), cy - (height / 2), cx + (width / 2), cy + (height / 2)

    def within_bounds(self, mode="CENTER"):
        if mode == "BOUNDS":
            corners = [(-9, -9), (-9, 9), (9, -9), (9, 9)]
        elif mode == "CENTER":
            corners = [(0, 0)]
        elif mode == "NEAR":
            corners = [(-10, -10), (-10, 10), (10, -10), (10, 10)]
        for corner in corners:
            if ((self.x1 <= x + corner[0] <= self.x2) and (self.y1 <= y + corner[1] <= self.y2)):
                return True
        return False

SelfMatchLoader1 = POI(12, 6, 24, 12)
SelfMatchLoader2 = POI(132, 6, 24, 12)
OppMatchLoader1 = POI(12, 138, 24, 12)
OppMatchLoader2 = POI(132, 138, 24, 12)

LGoal = POI(24, 48, 5.6, 5.6)
DGoal = POI(48, 24, 5.6, 5.6)
UGoal = POI(96, 120, 5.6, 5.6)
RGoal = POI(120, 96, 5.6, 5.6)

LOpp = POI(24, 96, 5.6, 5.6)
ROpp = POI(48, 120, 5.6, 5.6)

RSelf = POI(120, 48, 5.6, 5.6)
DSelf = POI(96, 24, 5.6, 5.6)

CenterGoal = POI(72, 72, 5.6, 5.6)

LToggle = POI(3, 72, 6, 26)
RToggle = POI(141, 72, 6, 26)
DToggle = POI(72, 3, 26, 6)
UToggle = POI(72, 141, 26, 6)


def toggle():
    T.set(not T.value())

def claw():
    C.set(not C.value())

def soften_bad(x): #turn a float between -1 and 1 into a dumb thing
    return math.tan(math.pi * x / 2.828) / 2

def soften(x):
    if x > 0.5:
        return (1.5 * x) - 0.5
    elif x > 0.1:
        return 0.25
    elif x < -0.5:
        return (1.5 * x) + 0.5
    elif x < -0.1:
        return -0.25
    else:
        return 2.5 * x


def control():
    global previous_angle, x, y
    speedl = (soften(CTRL.axis3.position() / 100) + soften(CTRL.axis1.position() / 100)) * 12
    speedr = (soften(CTRL.axis3.position() / 100) - soften(CTRL.axis1.position() / 100)) * 12
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
        for M in [LM1, LM2, LM3, RM1, RM2, RM3]:
            M.spin(FORWARD, 0.25 * err_dist, VOLT)

def poi_nav(poi):
    target_x = poi.x1 if x < poi.x1 else poi.x2 if x > poi.x2 else x
    target_y = poi.y1 if y < poi.y1 else poi.y2 if y > poi.y2 else y
    target_angle = math.atan((target_y - y) / (target_x - x)) * 180 / math.pi
    rotate(target_angle - I.rotation())
    target_distance = math.sqrt(((target_y - y) ** 2) + ((target_x - x) ** 2))
    drive(target_distance, max_voltage = 2)

def autonomous():
    brain.screen.clear_screen()
    claw()
    toggle()
    wait(800)
    drive(-13.5)
    rotate(-90)
    lift(800)
    drive(10)
    lift(-700)
    claw()
    drive(-8)

def user_control():
    CTRL.buttonR1.pressed(toggle)
    CTRL.buttonR2.pressed(claw)
    while True:
        control()
        """
        screen_status(str(x), 1)
        screen_status(str(y), 2)
        objects = V.take_snapshot(V.ALL_TAGS)
        for tag in objects:
            pass
        """
        wait(10, MSEC)

toggle()
I.calibrate() #Calibrate Inertial sensor
while I.is_calibrating():
    screen_status("Calibrating")
    wait(100)
screen_status("Calibration Complete")

comp = Competition(user_control, autonomous)
brain.screen.clear_screen()
