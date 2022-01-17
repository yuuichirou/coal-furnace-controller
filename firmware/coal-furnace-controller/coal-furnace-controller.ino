/*
 Controller for ash removal from coal furnace and central heating pump
 version 0.1

 Building blocks:
 - AVT5272 Arduino Duemilanove clone
 - AVT1666 Relay Board

 Required libraries:
 - OneWire 2.3.6 https://www.arduino.cc/reference/en/libraries/onewire/
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
#include <OneWire.h>
#include <TimeLib.h>


/*******************************************************************************
 *                                                                             *
 * Configuration                                                               *
 *                                                                             *
 ******************************************************************************/
#define SENSOR_NUMBER       3
// DS18B20 resolution 12bit (0.0625), 11bit (0.125), 10bit (0.250), 9bit (0.5)
#define SENSOR_RESOLUTION   10
#define CONVERT_INTERVAL    CONVERT_INTERVAL_9BIT

/*******************************************************************************
 *                                                                             *
 * Hardware configuration                                                      *
 *                                                                             *
 ******************************************************************************/
#define MOTOR_CON1          RELAY1_PIN
#define MOTOR_CON2          RELAY4_PIN

#define PUMP_CON            RELAY2_PIN

#define ONE_WIRE_BUS_PIN    13

/*******************************************************************************
 *                                                                             *
 * Default values                                                              *
 *                                                                             *
 ******************************************************************************/
#define DEF_TIME_TO_RUN     1800  // 30 minutes
#define DEF_TIME_TO_STOP    30    // 30 seconds
#define DEF_TEMP_TO_HALF    608   // 38 °C
#define DEF_TEMP_TO_START_PUMP  640     // 40 °C
#define DEF_TEMP_TO_STOP_PUMP   480     // 30 °C

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

/*******************************************************************************
 *                                                                             *
 * Dallas OneWire                                                              *
 *                                                                             *
 ******************************************************************************/
#define CONVERT_INTERVAL_12BIT  750  // for 12 bit resolution (0.0625)
#define CONVERT_INTERVAL_11BIT  375  // for 11 bit resolution (0.125)
#define CONVERT_INTERVAL_10BIT  188  // for 10 bit resolution (0.250)
#define CONVERT_INTERVAL_9BIT    94  // for  9 bit resolution (0.5)


enum motor_state {
    MOS_STOPPED,
    MOS_RUNNING_FORWARD,
    MOS_RUNNING_BACKWARD,
    MOS_NOT_ACTIVE
};

enum pump_state {
    PUS_STOPPED,
    PUS_RUNNING
    };

enum motor_command {
    FORWARD,
    BACKWARD,
    STOP
};

enum measuring_state {
    MES_IDLE,
    MES_CONVERTING_BUSY,
    MES_READY
};

time_t              time_now;
unsigned long       time_now_ms;

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
// temperature below which we reduce the waiting time for motor activation
int                 temperature_to_half_time = DEF_TEMP_TO_HALF;

/*******************************************************************************
 *                                                                             *
 * Pump control                                                                *
 *                                                                             *
 ******************************************************************************/
int                 pump_start_temperature = DEF_TEMP_TO_START_PUMP;
int                 pump_stop_temperature  = DEF_TEMP_TO_STOP_PUMP;
enum pump_state     pump_current_state;

/*******************************************************************************
 *                                                                             *
 * Temperature sensors                                                         *
 *                                                                             *
 ******************************************************************************/
OneWire                 one_wire_bus(ONE_WIRE_BUS_PIN);
byte                    one_wire_devices_count;
enum measuring_state    measuring_current_state;
unsigned long           measure_start_time;

byte                    sensor_addresses[SENSOR_NUMBER][8];
int                     temperatures[SENSOR_NUMBER];


byte scan_one_wire_bus(byte (*addresses)[8], byte array_size);
void set_resolution(byte (*addresses)[8], byte array_size, byte resolution);

/*******************************************************************************
 *                                                                             *
 * Arduino sketch functions                                                    *
 *                                                                             *
 ******************************************************************************/
void setup() {
    motor_init();
    pump_init();

    motor_current_state = MOS_STOPPED;
    pump_current_state  = PUS_STOPPED;
    measuring_current_state = MES_IDLE;

    count_one_wire_devices();
    if (one_wire_devices_count > 0) {
        scan_one_wire_bus(&sensor_addresses[0], SENSOR_NUMBER);
        set_resolution(&sensor_addresses[0],
          SENSOR_NUMBER > one_wire_devices_count ? one_wire_devices_count : SENSOR_NUMBER,
          SENSOR_RESOLUTION);
    }
}

void loop() {
    int t_min, t_max;

    time_now = now();  // resolution in seconds
    time_now_ms = millis();

/*******************************************************************************
 *                                                                             *
 * Read temperatures                                                           *
 *                                                                             *
 ******************************************************************************/
    if (one_wire_devices_count > 0) {
        switch (measuring_current_state) {
            case MES_IDLE:
                measure_temperatures();
                break;
            case MES_CONVERTING_BUSY:
                if (time_now_ms > (measure_start_time + CONVERT_INTERVAL)) {
                    measuring_current_state = MES_READY;
                }
                break;
            case MES_READY:
                read_temperatures(temperatures,
                  SENSOR_NUMBER > one_wire_devices_count ? one_wire_devices_count : SENSOR_NUMBER);
                break;
            default:
                break;
        }
    }
    else {
      for(int i = 0; i < SENSOR_NUMBER; i++) temperatures[i] = 0;
    }

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
    // if temperature drops below the threshold(the fire goes out), there
    // is no point in starting the motor
    t_max = 1440;
    for(byte i = 0; i < (SENSOR_NUMBER > one_wire_devices_count ? one_wire_devices_count : SENSOR_NUMBER); i++) {
        t_max = t_max > temperatures[i] ? t_max : temperatures[i];
    }
    if (t_max < pump_stop_temperature) {
        motor_current_state = MOS_NOT_ACTIVE;
    }
    // but if the temperature rises above the threshold (relighting of
    // the furnace) we resume operation
    if (t_max > pump_start_temperature) {
        if (motor_current_state == MOS_NOT_ACTIVE) {
            // don't start immediately after detecting a change
            // in temperature (it may have been a long time since
            // the motor was last stopped)          
            motor_current_state = MOS_STOPPED;
        }
    }

/*******************************************************************************
 *                                                                             *
 * Pump control                                                                *
 *                                                                             *
 ******************************************************************************/
    t_min = 0; t_max = 1440;
    for(byte i = 0; i < (SENSOR_NUMBER > one_wire_devices_count ? one_wire_devices_count : SENSOR_NUMBER); i++) {
        t_max = t_max > temperatures[i] ? t_max : temperatures[i];
        t_min = t_min < temperatures[i] ? t_min : temperatures[i];
    }
    // if temperature drops below the threshold(the fire goes out), there
    // is no point in starting the motor
    if (t_max < pump_stop_temperature) {
        pump_stop();
    }
    // but if the temperature rises above the threshold (relighting of
    // the furnace) we resume operation
    if (t_min > pump_start_temperature) {
        pump_start();
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

boolean time_to_run_has_already_passed(time_t time) {
    integer temperature = 0;
    byte i;

    for(i = 0; i < SENSOR_NUMBER; i++)
        temperature = temparature > temperatures[i] ? temparature : temperatures[i];
    if (temperature < temperature_to_half_time) {
        return (time - motor_start_time) >= (time_to_run/2);
    }
    else {
        return (time - motor_start_time) >= time_to_run;
    }
}

boolean time_to_stop_has_already_passed(time_t time)
{
    return (time - motor_start_time) >= time_to_stop;
}

/*******************************************************************************
 *                                                                             *
 * Pump control functions                                                      *
 *                                                                             *
 ******************************************************************************/
void pump_init(void) {
    pinMode(PUMP_CON, OUTPUT);
    digitalWrite(PUMP_CON, HIGH);
}

void pump_start(void) {
    digitalWrite(PUMP_CON, LOW);
    pump_current_state = PUS_RUNNING;
}

void pump_stop(void) {
    digitalWrite(PUMP_CON, HIGH);
    pump_current_state = PUS_STOPPED;
}

/*******************************************************************************
 *                                                                             *
 * Dallas OneWire temperature measurement functions                            *
 *                                                                             *
 ******************************************************************************/
byte count_one_wire_devices(void) {
    byte    i = 0;
    byte    dummy[8];

    one_wire_bus.reset_search();

    while (one_wire_bus.search(dummy)) {
        i++;
    }
    one_wire_devices_count = i;

    return 0;
}

byte scan_one_wire_bus(byte (*addresses)[8], byte array_size) {
    byte    i = 0;
    byte    dummy[8];

    one_wire_bus.reset_search();
    while (one_wire_bus.search(dummy)) {
        if (i < array_size) {
            memcpy(addresses[i], dummy, sizeof(dummy));
        }
        i++;
    }
    one_wire_devices_count = i;

    return 0;
}

void measure_temperatures (void) {
    one_wire_bus.reset();
    one_wire_bus.skip();
    one_wire_bus.write(0x44);
    measure_start_time = millis();
    measuring_current_state = MES_CONVERTING_BUSY;
}

void read_temperatures(int *temperatures, byte array_size) {
    byte scrachpad[9];
    byte i, j;

    for (i = 0; i < array_size; i++) {
        one_wire_bus.reset();
        one_wire_bus.select(sensor_addresses[i]);
        one_wire_bus.write(0xBE);
        for (j = 0; j < 9; j++) {
            scrachpad[j] = one_wire_bus.read();
        }
        temperatures[i] = (scrachpad[1] << 8) | scrachpad[0];      
    }
    measuring_current_state = MES_IDLE;
}

void set_resolution(byte (*addresses)[8], byte array_size, byte resolution) {
    byte scrachpad[9];
    byte i, j;

    for (i = 0; i < array_size; i++) {
        one_wire_bus.reset();
        one_wire_bus.select(addresses[i]);
        one_wire_bus.write(0xBE);
        for (j = 0; j < 9; j++) {
            scrachpad[j] = one_wire_bus.read();
        }
        if (resolution ==  9) scrachpad[4] = 0x1F;  // & 0x9F | 0x00
        if (resolution == 10) scrachpad[4] = 0x3F;  // & 0xBF | 0x20
        if (resolution == 11) scrachpad[4] = 0x5F;  // & 0xDF | 0x40
        if (resolution == 12) scrachpad[4] = 0x7F;  // & 0xFF | 0x60
        one_wire_bus.reset();
        one_wire_bus.select(addresses[i]);
        one_wire_bus.write(0x4E);
        one_wire_bus.write_bytes(&scrachpad[2], 3);
    }
    one_wire_bus.reset(); 
}
