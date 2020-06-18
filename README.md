# DJI C610 + M2006 interface library

## Usage

### C610Bus
Use this class if you want low-level access to up to 8 C610 controllers connected to a single CAN bus.

Example (still need to test these scripts actually):
```cpp
long last_command = 0;

void setup()
{
    C610Bus<CAN1> bus; // Initialization. Templated to either use CAN1 or CAN2.
}
void loop()
{
    bus.pollCAN(); // Check for messages from the motors.

    long now = millis();
    if (now - last_command >= 10) // Loop at 100Hz. You should limit the rate at which you call commandTorques to <1kHz to avoid saturating the CAN bus bandwidth
    {
        bus.commandTorques(100, 200, 300, 400, 0); // Command 100mA to motor 1, 200ma to motor 2, etc. The last parameter specifies to command the motors with IDs 1-4
        bus.commandTorques(500, 600, 700, 800, 1); // Command 500mA to motor 5, 600ma to motor 6, etc. The last parameter specifies to command the motors with IDs 5-8.
        int32_t m0_counts = bus.get(0).counts(); // Get the current encoder count reading for motor 0. Returns 0 - 8191 which covers one full rotation of the motor (not to be mistaken with the output shaft).
        int32_t m1_rpm = bus.get(1).rpm(); // Get the current rpm reading for motor 1. 
        int32_t m2_approx_torque = bus.get(2).torque(); // Get the current torque estimate for motor 2. Units are in mA (motor current is proportional to torque).

        last_command = now;
    }
}
```

### DriveSystem
Use this class if you want to control the 12 motors over two CAN buses (aka, you want to control a 12 DOF robot like Pupper). Includes some mid-level code to switch between operating modes like idle, PID, current-controler, etc.

Example:
```cpp
long last_command = 0;

void setup()
{
    DriveSystem d; // Initialization
    d.SetPosition(6, 0.5); // Set the setpoint for motor 6 (ID 1 on CAN2) to 0.5 radians. This setpoint is for the angle of the output shaft, not the motor.
    d.SetAllPositionKp(4.5); // Set the kp pid gain for all motors to kp=4.5 [A/rad]
    d.SetAllPositionKd(0.001); // Set the kp pid gain for all motors to kd=0.0003 [A/rad/s]
    d.ActivateActautor(6); // Activate motor 6 (ID 1 on CAN2), all other motors will be idling (zero voltage)
    // Other control options include current control and idle.
}
void loop()
{
    d.CheckForCANMessages(); // Checks for can messages from the motors
    
    long now = millis();
    if (now - last_command >= 2) // Loop at 500Hz. Rate should be < 1kHz to avoid saturation.
    {
        d.Update(); // Computes and sends current commands based off the current operating mode (pid, idle, etc)
        float shaft_angle = d.GetActuatorPosition(6); // Get the last measured angle (in radians) of motor 6's output shaft.
    }
}
```

### DJICANTest.cpp
This main file runs pid control on up to 12 motors. It supports commands over serial in JSON (or msgpack if you change the interpreter constructor parameter) to set the position or pid gains. 

Some example json messages you can send from the Arduino serial monitor:

Setting the positions to arbitrary values: ```<{"pos":[1,2,3,4,5,6,7,8,9,10,11,12]}>```

Setting the kp gain for all motors: ```<{"kp":2.0}>```

Setting the kd gain for all motors: ```<{"kd":0.001}>```