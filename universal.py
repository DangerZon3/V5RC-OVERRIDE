from vex import *
import math

front = 0
right = 1
skills = 2

AUTON_SIDE = skills

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

O = Rotation(Ports.PORT18, True)
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
        self.timer = 0
    
    def calculate(self, error, dt=0.01):
        self.timer += dt
        p_term = self.kp * error
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        i_term = self.ki * self.integral
        d_term = self.kd * (error - self.previous_error) / dt if dt > 0 else 0
        if abs(self.previous_error - error) <= 0.02 and self.timer >= 1.25:
            return "end"
        self.previous_error = error
        return (p_term + i_term + d_term)

    def reset(self):
        self.previous_error = 0
        self.integral = 0
        self.timer = 0

Driver = PID(1.33, 0, 0.045)
Rotater = PID(0.53, 0, 0.045)
Lifter = PID(0.1, 0, 0.01)

def distance(prev):
    return (O.position() - prev) * math.pi / 180

def drivel(target_distance: float = 0, drive_max: float = 10, target_height: float = 0, lift_max: float = 5, correction_rate: float = 0.1, timeout: int = 10000, pause=150):
    Driver.reset()
    Lifter.reset()
    start_time = brain.timer.time(MSEC)
    start_pos, start_rotation, start_height = O.position(), I.rotation(), L.position()

    driving = True
    lifting = True

    if target_height > 0:
        lift_max *= 2.4
    while brain.timer.time(MSEC) - start_time < timeout:

        if driving:
            drive_error = target_distance - distance(start_pos)
            drive_output = Driver.calculate(drive_error)
            if drive_output == "end":
                driving = False
                drive_output = 0
            drive_voltage = max(-drive_max, min(drive_max, drive_output))

            for LM in [LM1, LM2, LM3]:
                LM.spin(FORWARD, drive_voltage - correction_rate * (I.rotation() - start_rotation), VOLT)
            for RM in [RM1, RM2, RM3]:
                RM.spin(FORWARD, drive_voltage + correction_rate * (I.rotation() - start_rotation), VOLT)
            print("DRIVE voltage:", drive_voltage)

        if lifting: 
            lift_error = target_height + start_height - L.position()
            lift_output = Lifter.calculate(lift_error)
            if lift_output == "end":
                lifting = False
                lift_output = 0
            lift_voltage = max(-lift_max, min(lift_max, lift_output))

            for M in [L, R]:
                M.spin(FORWARD, lift_voltage, VOLT)
            print("LIFT voltage:", lift_voltage)
        
        wait(10, MSEC)

        for motor in [LM1, LM2, RM1, RM2]:
            motor.stop()
        for M in [L, R]:
            M.stop(HOLD)

        if -0.3 < drive_error and drive_error < 0.3:
            driving = False
        if -35 < lift_error and lift_error < 35:
            lifting = False

        if not (driving or lifting):
            break

    wait(pause, MSEC)
    print("Final Drive Error:", target_distance - distance(start_pos))
    print("Final Lift Error:", target_height + start_pos - L.position())


def rotate(target_degrees: float, max_voltage: float = 10, timeout: int = 7000, pause=150):
    Rotater.reset()
    start_time = brain.timer.time(MSEC)
    start_rotation = I.rotation()
    while brain.timer.time(MSEC) - start_time < timeout:
        error = target_degrees + start_rotation - I.rotation()
        pid_output = Rotater.calculate(error)
        if pid_output == "end" and -4.4 < error < 4.4:
            break
        elif pid_output == "end":
            pid_output = 0
        motor_voltage = max(-max_voltage, min(max_voltage, pid_output))
        for LM in [LM1, LM2, LM3]:
            LM.spin(FORWARD, motor_voltage, VOLT)
        for RM in [RM1, RM2, RM3]:
            RM.spin(REVERSE, motor_voltage, VOLT)
        wait(10, MSEC)
        for motor in [LM1, LM2, RM1, RM2]:
            motor.stop()
        print("ROTATE voltage:", motor_voltage)
        if -2.2 < error < 2.2:
            break
    wait(pause, MSEC)
    print("Final Error:", target_degrees + start_rotation - I.rotation())

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
UOpp = POI(48, 120, 5.6, 5.6)

RSelf = POI(120, 48, 5.6, 5.6)
DSelf = POI(96, 24, 5.6, 5.6)

CenterGoal = POI(72, 72, 5.6, 5.6)

LToggle = POI(3, 72, 6, 26)
RToggle = POI(141, 72, 6, 26)
DToggle = POI(72, 3, 26, 6)
UToggle = POI(72, 141, 26, 6)

def poi_nav(poi):
    target_x = poi.x1 if x < poi.x1 else poi.x2 if x > poi.x2 else x
    target_y = poi.y1 if y < poi.y1 else poi.y2 if y > poi.y2 else y
    target_angle = math.atan((target_y - y) / (target_x - x)) * 180 / math.pi
    rotate(target_angle - I.rotation())
    target_distance = math.sqrt(((target_y - y) ** 2) + ((target_x - x) ** 2))
    drivel(target_distance, drive_max=2)

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

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def toggle(pause=0):
    T.set(not T.value())
    wait(pause)

def claw(pause=0):
    C.set(not C.value())
    wait(pause)

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
            M.spin(FORWARD, 8, VOLT)
        elif CTRL.buttonL2.pressing():
            M.spin(REVERSE, 5, VOLT)
        else:
            M.stop(HOLD)
    if speedl == speedr:
        dist = distance(previous_angle)
        slope = math.tan(math.radians(I.rotation()))
        y += dist / math.sqrt(1 + slope ** 2)
        x += dist * slope / math.sqrt(1 + slope ** 2)
    previous_angle = O.position()
    wait(10, MSEC)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def autonomous():
    brain.screen.clear_screen()
    if AUTON_SIDE == 0:
        toggle(250)
        drivel(target_height=200)
        drivel(target_distance=-13.5, drive_max=7)
        rotate(target_degrees=-90, max_voltage=8)
        drivel(target_distance=11.5, drive_max=4, target_height=450, pause=0)
        drivel(target_height=-100)
        claw(250)
        drivel(target_distance=-16.4)
        rotate(target_degrees=-45)
        drivel(target_distance=25.6, drive_max=5, target_height=-500, pause=0)
        claw(250)
        drivel(target_distance=7.4, target_height=400)
        rotate(135, max_voltage=4, pause=0)
        drivel(target_distance=13.5, drive_max=6, target_height=700, pause=0)
        drivel(target_height=-700)
        claw()
        drivel(target_distance=-6, drive_max=12, pause=0)
        rotate(target_degrees=-45, max_voltage=12, pause=0)
        drivel(target_distance=-24, drive_max=12)
    elif AUTON_SIDE == 1:
        toggle(250)
        drivel(target_height=200)
        drivel(target_distance=-13.5, drive_max=7)
        rotate(target_degrees=90, max_voltage=8)
        drivel(target_distance=11.5, drive_max=4, target_height=450, pause=0)
        drivel(target_height=-100)
        claw(250)
        drivel(target_distance=-16.4)
        rotate(target_degrees=45)
        drivel(target_distance=25.6, drive_max=5, target_height=-500, pause=0)
        claw(250)
        drivel(target_distance=7.4, target_height=400)
        rotate(target_degrees=-135, max_voltage=4, pause=0)
        drivel(target_distance=13.5, drive_max=6, target_height=700, pause=0)
        drivel(target_height=-700)
        claw()
        drivel(target_distance=-6, drive_max=12, pause=0)
        rotate(target_degrees=45, max_voltage=12, pause=0)
        drivel(target_distance=-24, drive_max=12)
    elif AUTON_SIDE == 2:
        drivel(target_height=200)
        toggle(250)
        drivel(target_distance=-13.5, drive_max=7, target_height=400)
        rotate(target_degrees=-90, max_voltage=8)
        drivel(target_distance=12, drive_max=4, target_height=200, pause=0)
        drivel(target_height=-350)
        claw(250) # Pin 1 Placed
        drivel(target_distance=-16.4, drive_max=7)
        rotate(target_degrees=-45)
        drivel(target_distance=26.5, drive_max=5, target_height=-550, pause=0)
        claw(250) # Pin 2 Grabbed
        drivel(target_distance=6.5, drive_max=5, target_height=800)
        rotate(target_degrees=135, max_voltage=4, pause=0)
        drivel(target_distance=13.5, drive_max=6, target_height=500, pause=0)
        drivel(target_height=-900)
        claw() # Pin 2 Placed
        drivel(target_distance=-16.5, drive_max=6, target_height=-400, pause=0)
        rotate(target_degrees=-45, max_voltage=6) # Revert to 45 if fixed
        drivel(target_distance=26.5, drive_max=6, pause=0)
        claw(300) # Pin 3 Grabbed
        drivel(target_distance=7.5, drive_max=6, target_height=1200)
        rotate(target_degrees=135, max_voltage=4)
        drivel(target_distance=15.5, drive_max=3, target_height=550)
        claw(500) # Pin 3 Placed

        drivel(target_distance=-12, target_height=-550)
        rotate(target_degrees=45)
        drivel(target_distance=40, target_height=-1200)

"""MATCH LOAD CODE""
        drivel(target_height=-500)
        drivel(target_distance=-9999, drive_max=5, target_height=-1000)
        drivel(target_distance=5)
        rotate(target_degrees=-90, max_voltage=6)
        drivel(target_distance=12, drive_max=6, pause=0)
        drivel(target_distance=-1, drive_max=12, pause=0)
        claw(300) # Match-load 1 Grabbed
        drivel(target_distance=-12, drive_max=6, target_height=500)
        rotate(target_degrees=90, max_voltage=8)
        drivel(target_distance=30, drive_max=5, target_height=1000)
"""


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

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

toggle()
claw()
I.calibrate() #Calibrate Inertial sensor
while I.is_calibrating():
    screen_status("Calibrating")
    wait(100)
screen_status("Calibration Complete")

comp = Competition(user_control, autonomous)
brain.screen.clear_screen()
