#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "C610Bus.h"
#include "DriveSystem.h"

////////////////////// CONFIG ///////////////////////
const int PRINT_DELAY = 2 * 1000;
const int CONTROL_DELAY = 1000;

const int32_t MAX_TORQUE = 6000;
////////////////////// END CONFIG ///////////////////////

DriveSystem drive;

int32_t torque_setting = 0;

long last_command_ts;
long last_print_ts;


void parseSerialInput(char c, int32_t &torque_setting)
{
    // Right now this code treats all inputs as constant torque requests
    // TODO: make the serial parsing more flexible

    // Enter 'x' to go to IDLE mode!
    if (c == 'x')
    {
        drive.SetIdle();
    }

    Serial.print("Received: ");
    Serial.print(c);
    Serial.println(" * 1000mA");

    uint8_t int_input = c - '0';
    if (int_input >= 0 && int_input <= 20)
    {
        torque_setting = int_input * 1000;
    }
    if (c == '`')
    {
        torque_setting = 0;
    }
    while (Serial.available())
    {
        Serial.read();
    }
}

void setup(void)
{
    Serial.begin(115200);
    delay(400);

    last_command_ts = micros();
    last_print_ts = micros();

 
    // drive.SetCANCallbacks();
    ////////////// Runtime config /////////////////////
    // Put it in PID mode
    for(uint8_t i=0;i<DriveSystem::kNumActuators;i++)
    {
        drive.SetPosition(i, 0.0);
    }
    drive.SetUniformPositionGains(4.5, 0.0003);
    drive.ActivateActautor(6);
    // drive.SetIdle();
}

void loop()
{
    drive.CheckForCANMessages();
    // back.pollCAN();

    // for(uint8_t i=0; i<8; i++)
    // {
    //     Serial.print(back.get(i).counts());
    // }
    // Serial.println();


    if (micros() - last_command_ts > CONTROL_DELAY)
    {
        drive.Update();
        last_command_ts = micros();
    }

    if (micros() - last_print_ts > PRINT_DELAY)
    {
        DrivePrintOptions options;
        options.time = false;
        options.current_references = false;
        options.currents = true;
        options.velocities = false;
        options.velocity_references = false;
        options.position_references = false;

        drive.PrintStatus(options);
        last_print_ts = micros();
    }

    while (Serial.available())
    {
        char c = Serial.read();
        parseSerialInput(c, torque_setting);
    }
}

// has a 3 second watchdog
// 200 is the min current required to turn the output shaft (0.2A)
// stalls at 5A when holding very still but if small turns made doesn't hit stall condition
// 5 min at 3000 is totally ok, maybe 40deg c
// 8 min at 4000 is warm, maybe 60-70c at the middle aluminum
// takes around 0.03 seconds to go from 0 to 7150, with command 8000

// Notes: PID worked at 1000 hz, also worked at 100hz!