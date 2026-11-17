from vex import *
import math

front = 0
right = 1
skills = 2
yoink = 3

AUTON_SIDE = skills
EARLY_CUTOFF = False

brain = Brain()
TL = Motor(Ports.PORT13, GearSetting.RATIO_18_1, True)
L = Motor(Ports.PORT12, GearSetting.RATIO_18_1, False)
BL = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
TR = Motor(Ports.PORT16, GearSetting.RATIO_18_1, False)
R = Motor(Ports.PORT15, GearSetting.RATIO_18_1, True)
BR = Motor(Ports.PORT14, GearSetting.RATIO_18_1, False)

X = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
Y = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
LR = Rotation(Ports.PORT3, False)

C = DigitalOut(brain.three_wire_port.b)

OX = Rotation(Ports.PORT18, True)
OZ = Rotation(Ports.PORT18, True)
I = Inertial(Ports.PORT20)
V = AiVision(Ports.PORT5)

CTRL = Controller(PRIMARY)

x = 0
y = 0
previous_angle = 0


def screen_status(*txt, row=1):
    CTRL.screen.clear_row(row)
    CTRL.screen.set_cursor(row, 1)

    for t in txt:
        CTRL.screen.print(t, precision=0)
        CTRL.screen.print(" ")
        if t == "\n":
            CTRL.screen.new_line()

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

XDriver = PID(1.33, 0, 0.045)
ZDriver = PID(1.33, 0, 0.045)
Rotater = PID(0.53, 0, 0.045)
Lifter = PID(0.1, 0, 0.01)

def distance(prev):
    return (O.position() - prev) * math.pi / 180

def multifunction(motion: list[float], motion_cap: float = 10, drift_correction: float = 0.1,
                  rotate: float = 0, rotate_cap: float = 10,
                  lift: float = 0, lift_cap: float = 10,
                  timeout: int = 10000, pause: int = 150):
    ZDriver.reset()
    XDriver.reset()
    Lifter.reset()
    Rotater.reset()

    start_time = brain.timer.time(MSEC)
    start_z, start_x = OX.position(), OZ.position()
    start_rotation, start_height = I.rotation(), L.position()

    zdriving = True
    xdriving = True
    lifting = True
    rotating = True

    if lift > 0:
        lift_cap *= 2.4

    z = motion[0]
    if len(motion) > 1:
        x = motion[1]

    while brain.timer.time(MSEC) - start_time < timeout:

        z_voltage, x_voltage, r_voltage, l_voltage = 0, 0, 0, 0


        if zdriving:
            z_error = z - ((OZ.position() - start_z) * math.pi / 180)
            z_output = ZDriver.calculate(z_error)

            if -0.3 < z_error and z_error < 0.3:
                zdriving = False

            if z_output == "end":
                zdriving = False
                z_output = 0

            z_voltage = max(-motion_cap, min(motion_cap, z_output))
            print("Z", str(round(z_error)), "->", str(round(z_voltage, 2)))
        
        if xdriving:
            x_error = x - ((OX.position() - start_x) * math.pi / 180)
            x_output = XDriver.calculate(x_error)

            if -0.3 < x_error and x_error < 0.3:
                xdriving = False

            if x_output == "end":
                xdriving = False
                x_output = 0

            x_voltage = max(-motion_cap, min(motion_cap, x_output))
            print("X", str(round(x_error)), "->", str(round(x_voltage, 2)))


        if rotating:
            r_error = rotate + start_rotation - I.rotation()
            r_output = Rotater.calculate(r_error)

            if -2.2 < r_error and r_error < 2.2:
                rotating = False

            if r_output == "end" and -4.4 < r_error < 4.4:
                break
            elif r_output == "end":
                r_output = 0

            r_voltage = max(-rotate_cap, min(rotate_cap, r_output))
            print("R", str(round(r_error)), "->", str(round(r_voltage, 2)))

        if lifting: 
            l_error = lift + start_height - L.position()
            l_output = Lifter.calculate(l_error)

            if -35 < l_error and l_error < 35:
                lifting = False

            if l_output == "end":
                lifting = False
                l_output = 0

            l_voltage = max(-lift_cap, min(lift_cap, l_output))
            print("L", str(round(l_error)), "->", str(round(l_voltage, 2)))

        L.spin(FORWARD, z_voltage + r_voltage, VOLT)
        R.spin(FORWARD, z_voltage - r_voltage, VOLT)

        TL.spin(FORWARD, z_voltage + x_voltage + r_voltage, VOLT)
        BL.spin(FORWARD, z_voltage - x_voltage + r_voltage, VOLT)

        TR.spin(FORWARD, z_voltage - x_voltage - r_voltage, VOLT)
        BR.spin(FORWARD, z_voltage + x_voltage - r_voltage, VOLT)

        for M in [X, Y]:
            M.spin(FORWARD, l_voltage, VOLT)
        
        wait(10, MSEC)


        for motor in [TL, L, BL, TR, R, BR]:
            motor.stop()
        for M in [X, Y]:
            M.stop(HOLD)

        if not (zdriving or xdriving or rotating or lifting):
            break

    wait(pause, MSEC)
    print("Final Z Error:", z_error)
    print("Final X Error:", x_error)
    print("Final Rotation Error:", r_error)
    print("Final Lift Error:", l_error)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def claw(pause=0):
    C.set(not C.value())
    wait(pause)

def soften(x):
    if x > 0:
        return (x ** 1.5) / 10
    else:
        return ((-x) ** 1.5) / -10

def soften2(x):
    if x > 30:
        return ((x - 30) ** 2) / 49
    elif x < -30:
        return ((x + 30) ** 2) / -49
    else:
        return 0

def control():
    global previous_angle, x, y
    speedz = soften(CTRL.axis3.position())
    speedx = soften2(CTRL.axis4.position())
    speedr = CTRL.axis1.position()

    L.spin(FORWARD, speedz + speedr, PERCENT)
    R.spin(FORWARD, speedz - speedr, PERCENT)

    TL.spin(FORWARD, speedz + speedx + speedr, PERCENT)
    BL.spin(FORWARD, speedz - speedx + speedr, PERCENT)

    TR.spin(FORWARD, speedz - speedx - speedr, PERCENT)
    BR.spin(FORWARD, speedz + speedx - speedr, PERCENT)

    for M in [X, Y]:
        if CTRL.buttonL1.pressing():
            M.spin(FORWARD, 8, VOLT)
        elif CTRL.buttonL2.pressing():
            M.spin(REVERSE, 5, VOLT)
        else:
            M.stop(HOLD)
    """
    if speedl == speedr:
        dist = distance(previous_angle)
        slope = math.tan(math.radians(I.rotation()))
        y += dist / math.sqrt(1 + slope ** 2)
        x += dist * slope / math.sqrt(1 + slope ** 2)
    previous_angle = O.position()"""
    wait(10, MSEC)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

def autonomous():
    brain.screen.clear_screen()
"""
    if AUTON_SIDE == 0:
        drive(target_height=200, pause=200)
        drive(target_height=-200, pause=300)
        drive(target_distance=15, drive_max=7, target_height=400)
        rotate(target_degrees=90, max_voltage=8)
        drive(target_distance=999, drive_max=4, target_height=300, pause=0)
        drive(target_distance=-1.4, drive_max=6, target_height=-350)
        claw(250) # Pin 1 Placed
        if EARLY_CUTOFF:
            return
        drive(target_distance=-16, drive_max=7)
        rotate(target_degrees=-45)
        drive(target_distance=26.3, drive_max=5, target_height=-800, pause=0)
        claw(150) # Pin 2 Grabbed
        drive(target_height=200)
        drive(target_distance=6.7, drive_max=5, target_height=650)
        rotate(target_degrees=135, max_voltage=4, pause=0)
        drive(target_distance=13.5, drive_max=6, target_height=450, pause=0)
        drive(target_height=-900, pause=0)
        claw(250) # Pin 2 Placed
        drive(target_distance=-15.5, drive_max=8, target_height=-500)
        rotate(target_degrees=-45, max_voltage=6)
        drive(target_distance=26.5, drive_max=6, pause=0)
        claw(150) # Pin 3 Grabbed
        drive(target_distance=7, drive_max=6, target_height=1300)
        rotate(target_degrees=135, max_voltage=8)
        #drive(target_distance=999, drive_max=4, target_height=650, pause=0)
        #drive(target_distance=-1, drive_max=6, target_height=-100)

    elif AUTON_SIDE == 1:
        drive(target_height=200, pause=200)
        drive(target_height=-200, pause=300)
        drive(target_distance=15, drive_max=7, target_height=400)
        rotate(target_degrees=-90, max_voltage=8)
        drive(target_distance=999, drive_max=4, target_height=300, pause=0) 
        drive(target_distance=-1.4, drive_max=6, target_height=-350)
        claw(250) # Pin 1 Placed
        if EARLY_CUTOFF:
            return
        drive(target_distance=-16, drive_max=7)
        rotate(target_degrees=45)
        drive(target_distance=26.3, drive_max=5, target_height=-800, pause=0)
        claw(150) # Pin 2 Grabbed
        drive(target_height=200)
        drive(target_distance=6.7, drive_max=5, target_height=650)
        rotate(target_degrees=-135, max_voltage=4, pause=0)
        drive(target_distance=13.5, drive_max=6, target_height=450, pause=0)
        drive(target_height=-900, pause=0)
        claw(250) # Pin 2 Placed
        drive(target_distance=-15.5, drive_max=8, target_height=-500)
        rotate(target_degrees=45, max_voltage=6)
        drive(target_distance=26.5, drive_max=6, pause=0)
        claw(150) # Pin 3 Grabbed
        drive(target_distance=7, drive_max=6, target_height=1300)
        rotate(target_degrees=-135, max_voltage=8)
        #drive(target_distance=999, drive_max=4, target_height=650, pause=0)
        #drive(target_distance=-1, drive_max=6, target_height=-100)

    elif AUTON_SIDE == 2:
        drive(target_height=200, pause=200)
        drive(target_height=-200, pause=300)
        drive(target_distance=15, drive_max=7, target_height=400)
        rotate(target_degrees=90, max_voltage=8)
        drive(target_distance=999, drive_max=4, target_height=300, pause=0)
        drive(target_distance=-1.4, drive_max=6, target_height=-350)
        claw(250) # Pin 1 Placed
        drive(target_distance=-16, drive_max=7)
        rotate(target_degrees=-45)
        drive(target_distance=26.1, drive_max=4, target_height=-800, pause=0)
        claw(150) # Pin 2 Grabbed
        drive(target_height=200)
        drive(target_distance=6.9, drive_max=5, target_height=650)
        rotate(target_degrees=135, max_voltage=4, pause=0)
        drive(target_distance=9999, drive_max=6, target_height=450, pause=0)
        drive(target_distance=-1, drive_max=4)
        drive(target_height=-900, pause=0)
        claw(250) # Pin 2 Placed
        drive(target_distance=-16, drive_max=8, target_height=-500)
        rotate(target_degrees=-45, max_voltage=6)
        drive(target_distance=26.5, drive_max=6, pause=0)
        claw(150) # Pin 3 Grabbed
        drive(target_distance=7, drive_max=6, target_height=1300)
        rotate(target_degrees=135, max_voltage=8)
        drive(target_distance=999, drive_max=4, target_height=650, pause=0)
        drive(target_distance=-1, drive_max=6, target_height=-100)
        claw(500) # Pin 3 Placed

        for cycle in range(2):
            drive(target_distance=-9999, drive_max=5, target_height=-9999)
            drive(target_distance=5, drive_max=6)
            rotate(target_degrees=-90, max_voltage=5)
            drive(target_distance=18.5 - (cycle), drive_max=4)
            claw(500)
            drive(target_height=400, pause=0)
            drive(target_distance=-14, drive_max=6)
            rotate(target_degrees=90, max_voltage=5)
            drive(target_height=1050)
            drive(target_distance=9999, drive_max=4, target_height=1000 + (500 * cycle), lift_max=5)
            drive(target_height=-100, pause=0)
            claw(500)

        drive(target_distance=-12, drive_max=4, target_height=-750)
        rotate(target_degrees=45)
        drive(target_distance=40, target_height=-1100)
    """


def user_control():
    CTRL.buttonR2.pressed(claw)
    timer = 105
    
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
        timer -= 0.1
        if timer == 30:
            CTRL.rumble("-")
        elif timer == 20:
            CTRL.rumble("--")
        elif round(timer) == timer and timer <= 5:
            CTRL.rumble(".")


def temp():
    while True:
        mtemps = [TL.temperature(), L.temperature(), BL.temperature(),
                  TR.temperature(), R.temperature(), BR.temperature()]
        screen_status(".         FR MD BK", "\n",
                      "LEFT   ", mtemps[0], mtemps[1], mtemps[2], "\n",
                      "RIGHT ", mtemps[3], mtemps[4], mtemps[5])
        for temp in mtemps:
            if temp >= 55:
                CTRL.rumble("...")
                break
        wait(1000)

#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

claw()
I.calibrate() #Calibrate Inertial sensor
while I.is_calibrating():
    screen_status("Calibrating")
    wait(100)
screen_status("Calibration Complete")
Thread(temp)

comp = Competition(user_control, autonomous)
brain.screen.clear_screen()
