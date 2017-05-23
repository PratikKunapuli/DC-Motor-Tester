import math 

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

