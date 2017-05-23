import matplotlib.pyplot as plt
import math
from DCMotorSimulation import DCMotor
from matplotlib import style
style.use('ggplot')

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

"""
Arm Up test. This test uses data from FRC Team 254 as an example to verify simulation results.
This test simulates an arm moving from it's "down" position back to  vertical.
"""

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



"""
Arm down test. This test uses data from FRC Team 254 as an example to verify simulation results.
This test simulates an arm moving 90 degrees down from vertical.
"""

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
Simple test running for 1 second at 1 millisecond steps. 
"""
"""
for i in range(1,1000):
    testGearbox.step(-12.0, 0.04, -9.8/.2, 0.001)
    if i%1 == 0:
        positions.append(testGearbox.position)
        velocities.append(testGearbox.velocity)
        currents.append(testGearbox.current)
"""

#Uncomment for plotting selected data
#plt.plot(positions)
#plt.plot(velocities)
#plt.plot(currents)


plt.plot(hook_position)
plt.show()
