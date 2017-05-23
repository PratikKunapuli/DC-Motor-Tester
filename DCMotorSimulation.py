import math
import matplotlib.pyplot as plt
from matplotlib import style
style.use('ggplot')

class DCMotor(object):
    def __init__(self, motorName):
        if motorName == "RS775" :
            self.kt = .009
            self.kv = 1083.0 * (math.pi * 2.0) / 60.0;
            self.resistance = 18.0/330.0
            self.inertia = 1.20348237e-5
        elif motorName == "RS550" :
            self.kt = .004862
            self.kv = 1608.0 * (math.pi * 2.0) / 60.0;
            self.resistance = 12.0 / 85.0
            self.inertia = 0
        elif motorName == "775PRO" :
            self.kt = (0.71/134) #Stall torque/Stall current
            self.kv = (18730.0/12.0) * (math.pi * 2.0) / 60.0; #(RPM/V) * 2pi/60
            self.resistance = 12.0/134.0 #V / stall current 
            self.inertia = (0.362874 * 0.02215 * 0.02215) / 2 #(mass(in grams) * radius (in meters) ^2) / 2
        elif motorName == "CIM" :
            self.kt = (2.41/131) #Stall torque/Stall current
            self.kv = (5330.0/12.0) * (math.pi * 2.0) / 60.0; #(RPM/V) * 2pi/60
            self.resistance = 12.0/131.0 #V / stall current 
            self.inertia = (1.27006 * 0.03175 * 0.03175) / 2 #(mass(in grams) * radius (in meters) ^2) / 2

        self.m_motorName = motorName
        self.numberMotors = 1
        self.position = 0.0;
        self.velocity = 0.0;
        self.current = 0.0;

    def makeTransmission(self, motor, numMotors, gearReduction, efficiency):
        self.kt = motor.kt * numMotors * gearReduction * efficiency
        self.kv = motor.kv / gearReduction
        self.resistance = motor.resistance / numMotors
        self.inertia = motor.inertia * numMotors * gearReduction * gearReduction
        self.m_motorName = motor.m_motorName
        self.numberMotors = numMotors

    def reset(self, newPos, newVel, newCurrent):
        self.position = newPos
        self.velocity = newVel
        self.current = newCurrent

    def __str__(self):
        return "%d of %s" % (self.numberMotors, self.m_motorName)

    """
    Simulate applying a given voltage and load for a specified period of
    time.
    
    @param applied_voltage
        Voltage applied to the motor (V)
    @param load
        Load applied to the motor (kg*m^2)
    @param external_torque
        The external torque applied (ex. due to gravity) (N*m)
    @param timestep
        How long the input is applied (s)

    Using the 971-style first order system model. V = I * R + Kv * w
    torque = Kt * I

    V = torque / Kt * R + Kv * w torque = J * dw/dt + external_torque

    dw/dt = (V - Kv * w) * Kt / (R * J) - external_torque / J
    """
    def step(self, applied_voltage, load, external_torque, timestep):
        load += self.inertia
        acceleration = (applied_voltage - (self.velocity / self.kv)) * self.kt / (self.resistance * load) + (external_torque / load)
        self.velocity += acceleration * timestep
        self.position += self.velocity * timestep + 0.5 * acceleration * timestep * timestep
        self.current = load * acceleration * math.copysign(1,applied_voltage) / self.kt

testGearbox = DCMotor("RS775")
testGearbox.makeTransmission(DCMotor("RS775"), 2, 77.0, 0.8)
positions = []
velocities = []
currents = []
hook_position = []
print("254 CanGrabber Test Data")
testGearbox.reset(0,0,0)
battery_voltage = 12.8
arm_com = 0.82 #meters
arm_mass = 0.2 #kg
battery_r = .018 #battery resistance (ohms)
constant_spring_assist_torque = 0.0 #torque provided constantly over the entire range of motion
over_center_spring_assist_torque = 0.0 #torque provided varying with sin of angle (same effect as gravity) due to over-center.
total_time = 0.0
arm_inertia = arm_mass * arm_com * arm_com

while testGearbox.position < math.pi/2 and total_time < 1.0:
    gravity_torque = 9.8 * arm_com * arm_mass * math.sin(testGearbox.position)
    current = testGearbox.current * 2
    testGearbox.step(
                    battery_voltage - current * battery_r,
                    arm_inertia,
                    gravity_torque + constant_spring_assist_torque
                            + over_center_spring_assist_torque
                            * math.sin(testGearbox.position), 
                    0.0001)
    positions.append(testGearbox.position)
    velocities.append(testGearbox.velocity)
    hook_position.append((math.pi/2-math.sin(testGearbox.position)) * arm_com * 2)
    currents.append(testGearbox.current)
    total_time += .0001
total_time = 0
while testGearbox.position > 0 and total_time < 1.0:
    gravity_torque = 9.8 * arm_com * arm_mass * math.sin(testGearbox.position)
    current = testGearbox.current * 2
    testGearbox.step(
                    -1 * battery_voltage - current * battery_r,
                    arm_inertia,
                    gravity_torque + constant_spring_assist_torque
                            + over_center_spring_assist_torque
                            * math.sin(testGearbox.position), 
                    0.0001)
    positions.append(testGearbox.position)
    velocities.append(testGearbox.velocity)
    hook_position.append((math.pi/2-math.sin(testGearbox.position)) * arm_com * 2)
    currents.append(testGearbox.current)
    total_time += .0001
print("Total time: ", total_time)

"""
for i in range(1,1000):
    testGearbox.step(-12.0, 0.04, -9.8/.2, 0.001)
    if i%1 == 0:
        positions.append(testGearbox.position)
        velocities.append(testGearbox.velocity)
        currents.append(testGearbox.current)
"""
#plt.plot(positions)
#plt.plot(velocities)
#plt.plot(currents)
plt.plot(hook_position)
plt.show()
