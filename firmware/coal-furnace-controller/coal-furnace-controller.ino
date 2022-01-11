/*
 Controller for ash removal from coal furnace and central heating pump
 version 0.1

 Building blocks:
 - AVT5272 Arduino Duemilanove clone
 - AVT1666 Relay Board

 Required libraries:
 * - Time 1.6.1 https://www.arduino.cc/reference/en/libraries/time/


                                          +-----+
             +----[PWR]-------------------| USB |--+    AVT
             |                            +-----+  |    1666
             |         GND/RST2  [ ][ ]            |
             |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] | C5 PK2
             |          5V/MISO2 [ ][ ]  A4/SDA[ ] | C4 PK1
             |                             AREF[ ] |
             |                              GND[ ] |
             | [ ]N/C                    SCK/13[ ] | B5
             | [ ]IOREF                 MISO/12[ ] | .
             | [ ]RST                   MOSI/11[ ]~| .
             | [ ]3V3    +---+               10[ ]~| .
             | [ ]5v    -| A |-               9[ ]~| .
             | [ ]GND   -| R |-               8[ ] | B0
AVT          | [ ]GND   -| D |-                    |
1666         | [ ]Vin   -| U |-               7[ ] | D7
             |          -| I |-               6[ ]~| .
          C0 | [ ]A0    -| N |-               5[ ]~| .
          .  | [ ]A1    -| O |-               4[ ] | .
PK3       .  | [ ]A2     +---+           INT1/3[ ]~| .
PK4       .  | [ ]A3                     INT0/2[ ] | .
PK1       .  | [ ]A4/SDA  RST SCK MISO     TX>1[ ] | .
PK2       C5 | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] | D0
             |            [ ] [ ] [ ]              |
             |  UNO_R3    GND MOSI 5V  ____________/
              \_______________________/
      
      http://busyducks.com/ascii-art-arduinos
*/

// Arduino Standard Libraries
// AVR Standard C Libraries
// Other Libraries
#include <TimeLib.h>


/*******************************************************************************
 *                                                                             *
 * Hardware configuration                                                      *
 *                                                                             *
 ******************************************************************************/
#define MOTOR_CON1          RELAY1_PIN
#define MOTOR_CON2          RELAY4_PIN

/*******************************************************************************
 *                                                                             *
 * Default values                                                              *
 *                                                                             *
 ******************************************************************************/
#define DEF_TIME_TO_RUN     1800  // 30 minutes
#define DEF_TIME_TO_STOP    30    // 30 seconds

/*******************************************************************************
 *                                                                             *
 * AVT1666 board "AVTduino RELAY"                                              *
 * 4 single pole relays                                                        *
 *                                                                             *
 ******************************************************************************/
#define RELAY1_PIN          A4
#define RELAY2_PIN          A5
#define RELAY3_PIN          A2
#define RELAY4_PIN          A3


enum motor_state {
    MOS_STOPPED,
    MOS_RUNNING_FORWARD,
    MOS_RUNNING_BACKWARD
};

enum motor_command {
    FORWARD,
    BACKWARD,
    STOP
};


time_t              time_now;

/*******************************************************************************
 *                                                                             *
 * Motor control                                                               *
 *                                                                             *
 ******************************************************************************/
// wiper motor activation time
time_t              motor_start_time;
// interval between consecutive activations / time to switch on the wiper motor
time_t              time_to_run = DEF_TIME_TO_RUN;
// time of activation / time to switch off the wiper motor
time_t              time_to_stop = DEF_TIME_TO_STOP;
enum motor_state    motor_current_state;


/*******************************************************************************
 *                                                                             *
 * Arduino sketch functions                                                    *
 *                                                                             *
 ******************************************************************************/
void setup() {
    motor_init();

    motor_current_state = MOS_STOPPED;
}

void loop() {
    time_now = now();  // resolution in seconds

/*******************************************************************************
 *                                                                             *
 * Motor control                                                               *
 *                                                                             *
 ******************************************************************************/
    switch (motor_current_state)
    {
        case MOS_STOPPED:
            if (time_to_run_has_already_passed(time_now))
            {
                motor_control(FORWARD);
            }
            break;
        case MOS_RUNNING_FORWARD:
            if (time_to_stop_has_already_passed(time_now))
            {
                motor_control(STOP);
            }
            break;
        case MOS_RUNNING_BACKWARD:
            if (time_to_stop_has_already_passed(now()))
            {
                motor_control(STOP);
            }
            break;
        default:
            break;
    }
    delay(50);
}

/*******************************************************************************
 *                                                                             *
 * Motor control functions                                                     *
 *                                                                             *
 ******************************************************************************/
void motor_init(void)
{
    pinMode(MOTOR_CON1, OUTPUT);
    digitalWrite(MOTOR_CON1, HIGH);
    pinMode(MOTOR_CON2, OUTPUT);
    digitalWrite(MOTOR_CON2, HIGH);
}

void motor_control(enum motor_command command)
{
    switch(command)
    {
    case FORWARD:
        digitalWrite(MOTOR_CON1, HIGH);
        digitalWrite(MOTOR_CON2, LOW);
        motor_current_state = MOS_RUNNING_FORWARD;
        motor_start_time = time_now;
        break;
    case BACKWARD:
        digitalWrite(MOTOR_CON1, LOW);
        digitalWrite(MOTOR_CON2, HIGH);
        motor_current_state = MOS_RUNNING_BACKWARD;
        motor_start_time = time_now;
        break;
    case STOP:
        digitalWrite(MOTOR_CON1, HIGH);
        digitalWrite(MOTOR_CON2, HIGH);
        motor_current_state = MOS_STOPPED;
        break;
    default:
        break;
    }
}

boolean time_to_run_has_already_passed(time_t time)
{
    return (time - motor_start_time) >= time_to_run;
}

boolean time_to_stop_has_already_passed(time_t time)
{
    return (time - motor_start_time) >= time_to_stop;
}
