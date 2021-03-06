/*
 Controller for ash removal from coal furnace and central heating pump
 version 0.4

 Building blocks:
 - AVT5272 Arduino Duemilanove clone
 - AVT1666 Relay Board
 - AVT1722 miniature operator panel for Arduino

 Required libraries:
 - Encoder 1.4.2 https://www.arduino.cc/reference/en/libraries/encoder
 - LiquidCrystal 1.0.7 https://www.arduino.cc/en/Reference/LiquidCrystal
 - OneWire 2.3.6 https://www.arduino.cc/reference/en/libraries/onewire/
 - Time 1.6.1 https://www.arduino.cc/reference/en/libraries/time/


                                          +-----+
             +----[PWR]-------------------| USB |--+    AVT  AVT
             |                            +-----+  |    1666 1722
             |         GND/RST2  [ ][ ]            |
             |       MOSI2/SCK2  [ ][ ]  A5/SCL[ ] | C5 PK2
             |          5V/MISO2 [ ][ ]  A4/SDA[ ] | C4 PK1  S4
             |                             AREF[ ] |
             |                              GND[ ] |
             | [ ]N/C                    SCK/13[ ] | B5      DS18B20
             | [ ]IOREF                 MISO/12[ ] | .       LED B
             | [ ]RST                   MOSI/11[ ]~| .       LED G
             | [ ]3V3    +---+               10[ ]~| .       LED R
             | [ ]5v    -| A |-               9[ ]~| .       LCD EN
             | [ ]GND   -| R |-               8[ ] | B0      LCD RS
AVT  AVT     | [ ]GND   -| D |-                    |
1666 1722    | [ ]Vin   -| U |-               7[ ] | D7      LCD DB7
             |          -| I |-               6[ ]~| .       LCD DB6
     PK1  C0 | [ ]A0    -| N |-               5[ ]~| .       LCD DB5
     S1   .  | [ ]A1    -| O |-               4[ ] | .       LCD DB4
PK3  S2   .  | [ ]A2     +---+           INT1/3[ ]~| .
PK4  S3   .  | [ ]A3                     INT0/2[ ] | .       ENC
PK1  S4   .  | [ ]A4/SDA  RST SCK MISO     TX>1[ ] | .       ENC
PK2       C5 | [ ]A5/SCL  [ ] [ ] [ ]      RX<0[ ] | D0      ENC PUSH BUTTON
             |            [ ] [ ] [ ]              |
             |  UNO_R3    GND MOSI 5V  ____________/
              \_______________________/

      http://busyducks.com/ascii-art-arduinos
*/

// Arduino Standard Libraries
#include <LiquidCrystal.h>
// AVR Standard C Libraries
#include <avr/eeprom.h>
#include <avr/wdt.h>
// Other Libraries
#include <Encoder.h>
#include <OneWire.h>
#include <TimeLib.h>


/*******************************************************************************
 *                                                                             *
 * Configuration                                                               *
 *                                                                             *
 ******************************************************************************/
#define SENSOR_NUMBER           3
// DS18B20 resolution 12bit (0.0625), 11bit (0.125), 10bit (0.250), 9bit (0.5)
#define SENSOR_RESOLUTION       9
#define CONVERT_INTERVAL        CONVERT_INTERVAL_9BIT
#define TEMP_INTEGER_DIGITS     2
#define TEMP_FRACTIONAL_DIGITS  1
#define TIME_TO_RUN_MAX_SETTING     5400    // 1,5 hour
#define TIME_TO_STOP_MAX_SETTING    90      // 1,5 minute
#define TEMPERATURE_MAX_SETTING     1520    // 95 ??C
#define ENCODER_PULSES_PER_MENU     8   // DEF_ENC_PULSES_PER_ROT

/*******************************************************************************
 *                                                                             *
 * Hardware configuration                                                      *
 *                                                                             *
 ******************************************************************************/
#define MOTOR_CON1          RELAY1_PIN
#define MOTOR_CON2          RELAY4_PIN
#define MOTOR_PARKING       3

#define PUMP_CON            RELAY2_PIN

/*******************************************************************************
 *                                                                             *
 * Default values                                                              *
 *                                                                             *
 ******************************************************************************/
#define DEF_TIME_TO_RUN         1800    // 30 minutes
#define DEF_TIME_TO_STOP        30      // 30 seconds
#define DEF_TEMP_TO_HALF        608     // 38 ??C
#define DEF_TEMP_TO_START_PUMP  640     // 40 ??C
#define DEF_TEMP_TO_STOP_PUMP   480     // 30 ??C
#define DEF_MENU_CLICK_TIMEOUT  500     // 2 clicks per second
#define DEF_MOTOR_PARKING       false
#define DEF_ENC_PULSES_PER_ROT  4

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

// conflict with switch S2 on AVT1722 board
#undef RELAY3_PIN

/*******************************************************************************
 *                                                                             *
 * AVT1722 board "AVTduino miniLCD"                                            *
 * 2x8 LCD, encoder switch, 4 push buttons, double relay, RGB LED,             *
 * OneWire connection                                                          *
 *                                                                             *
 ******************************************************************************/
// LCD display pins
#define LCD_RS              8
#define LCD_EN              9
#define LCD_D4              4
#define LCD_D5              5
#define LCD_D6              6
#define LCD_D7              7
// Double output relay
//#define RELAY_PIN           A0          // not connected
// One Wire connector
#define ONE_WIRE_BUS_PIN    13
// Encoder pins
#define ENC_A               1
#define ENC_B               2
#define ENC_BUTTON          0
// Switches
#define S1                  A1
#define S2                  A2
#define S3                  A3          // pull-up resistor not soldered
#define S4                  A4          // pull-up resistor not soldered
// Leds
#define LEDG                10
#define LEDR                11
#define LEDB                12
#define GREEN_LED           LEDG
#define RED_LED             LEDR
#define BLUE_LED            LEDB

// conflict with AVT1666 board
#undef S3
#undef S4

/*******************************************************************************
 *                                                                             *
 * Dallas OneWire                                                              *
 *                                                                             *
 ******************************************************************************/
#define CONVERT_INTERVAL_12BIT  750     // for 12 bit resolution (0.0625)
#define CONVERT_INTERVAL_11BIT  375     // for 11 bit resolution (0.125)
#define CONVERT_INTERVAL_10BIT  188     // for 10 bit resolution (0.250)
#define CONVERT_INTERVAL_9BIT    94     // for  9 bit resolution (0.5)
#define OW_CONVERT_T            0x44
#define OW_READ_SCRATCHPAD      0xBE
#define OW_WRITE_SCRATCHPAD     0x4E
#define DS18B20_FRACTIONAL_BITS 4

/*******************************************************************************
 *                                                                             *
 * LCD special characters                                                      *
 *                                                                             *
 ******************************************************************************/
#define RETURN_SIGN 126
#define DEGREE_SIGN 223

/*******************************************************************************
 *                                                                             *
 * Storing operating parameters in non-volatile memory (EEPROM)                *
 *                                                                             *
 ******************************************************************************/
// first 4 bytes empty
#define TIME_TO_RUN_EEPROM_ADDRESS              4    // 4 B
#define TIME_TO_STOP_EEPROM_ADDRESS             8    // 4 B
#define SENSOR_ADDRESSES_EEPROM_ADDRESS         12   // SENSOR_NUMBER * 8 B
#define PUMP_START_TEMPERATURE_EEPROM_ADDRESS   12 + SENSOR_NUMBER * 8  // 2 B
#define PUMP_STOP_TEMPERATURE_EEPROM_ADDRESS    14 + SENSOR_NUMBER * 8  // 2 B
#define ENABLE_MOTOR_PARKING_EEPROM_ADDRESS     16 + SENSOR_NUMBER * 8  // 1 B
#define NEXT_EEPROM_ADDRESS                     17 + SENSOR_NUMBER * 8 //41 //?B


enum motor_state {
    MOS_STOPPED,
    MOS_RUNNING_FORWARD,
    MOS_RUNNING_BACKWARD,
    MOS_PARKING,
    MOS_NOT_ACTIVE
};

enum pump_state {
    PUS_STOPPED,
    PUS_RUNNING
    };

enum motor_command {
    FORWARD,
    BACKWARD,
    PARK,
    STOP
};

enum measuring_state {
    MES_IDLE,
    MES_CONVERTING_BUSY,
    MES_READY
};

enum encoder_state {
    ENS_CW,
    ENS_CCW,
    ENS_NO_CHANGE
};

/*******************************************************************************
 *                                                                             *
 * Display menu definition                                                     *
 *                                                                             *
 ******************************************************************************/
enum menu_state {
    m_main_menu_first_pos,
    m_time_to_run,                      // time left till start the motor
    m_time_to_stop,                     // time left till stop the motor
    m_temperature,                      // temperatures display
    m_motor_state,                      // state of the motor, on/off
    m_pump_state,                       // central heating pump status, on/off
    m_settings,                         // open the settings menu
    m_main_menu_last_pos,

    m_settings_menu_first_pos,
    m_settings_time_to_run,
    m_settings_time_to_stop,
    m_settings_temperature,
    m_settings_pump_start_temperature,
    m_settings_pump_stop_temperature,
    m_settings_motor_parking,
    m_settings_one_wire_devices_count,
    m_settings_store_to_eeprom,
    m_settings_reset_to_factory,
    m_settings_return,
    m_settings_menu_last_pos
};


time_t              time_now;
unsigned long       time_now_ms;
LiquidCrystal       lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);
Encoder             encoder(ENC_B, ENC_A);
int                 enc;
int                 enc_last;
int                 main_menu_position;
int                 settings_menu_position;
int                 sensor_menu_position;
boolean             in_settings_menu;
char                *menu_titles[] = {
    NULL,
    "start za",
    "stop za",
    NULL,
    "silnik",
    "pompa",
    "ustawienia",
    NULL,

    NULL,
    "start za",
    "stop za",
    NULL,
    "pom start",
    "pom stop",
    "parkowanie",    
    "ilosc czujnikow",
    "zapisz",
    "ustawienia",
    "powrot",
    NULL
};
char                *sensor_names[] = {
    "temp wyj", "temp wej", "temp dom"
};
time_t              menu_last_click;
time_t              menu_click_timeout = DEF_MENU_CLICK_TIMEOUT;
boolean             editing_mode;
time_t              editing_time_value;
int                 editing_temperature;
byte                editing_sensor_index;
boolean             editing_parking;

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
boolean             enable_motor_parking = DEF_MOTOR_PARKING;

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

byte                    (*one_wire_addresses)[8];
int                     *one_wire_temperatures;

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
    encoder_init();
    lcd.begin(8, 2);
    lcd.setCursor(0, 0);
    lcd.print("Sterownik");
    lcd.setCursor(0, 1);
    lcd.print("v0.4");

    motor_current_state = MOS_STOPPED;
    pump_current_state = PUS_STOPPED;
    measuring_current_state = MES_IDLE;

    main_menu_position = m_time_to_run;
    settings_menu_position = m_settings_time_to_run;
    sensor_menu_position = 0;
    in_settings_menu = false;
    menu_last_click = 0;
    editing_mode = false;

    // set state from saved params
    eeprom_read_block(&time_to_run, TIME_TO_RUN_EEPROM_ADDRESS, sizeof(time_t));
    eeprom_read_block(&time_to_stop, TIME_TO_STOP_EEPROM_ADDRESS,
        sizeof(time_t));
    for(byte i = 0; i < SENSOR_NUMBER; i++)
        eeprom_read_block(sensor_addresses[i],
            SENSOR_ADDRESSES_EEPROM_ADDRESS + i * 8,
            sizeof(sensor_addresses[0]));
    eeprom_read_block(&pump_start_temperature,
        PUMP_START_TEMPERATURE_EEPROM_ADDRESS, sizeof(int));
    eeprom_read_block(&pump_stop_temperature,
        PUMP_STOP_TEMPERATURE_EEPROM_ADDRESS, sizeof(int));
    eeprom_read_block(&enable_motor_parking,
        ENABLE_MOTOR_PARKING_EEPROM_ADDRESS, sizeof(byte));

    count_one_wire_devices();
    if (one_wire_devices_count > 0) {
        set_resolution(&sensor_addresses[0],
            min(SENSOR_NUMBER, one_wire_devices_count), SENSOR_RESOLUTION);
    }

    delay(1000);
    wdt_enable(WDTO_2S);
}

void loop() {
    int t_max;

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
                  min(SENSOR_NUMBER, one_wire_devices_count));
                if (editing_mode && one_wire_temperatures != NULL)
                    read_temperatures(one_wire_temperatures,
                        one_wire_devices_count);
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
    switch (motor_current_state) {
        case MOS_STOPPED:
            if (time_to_run_has_already_passed(time_now)) {
                motor_control(FORWARD);
            }
            break;
        case MOS_RUNNING_FORWARD:
            if (time_to_stop_has_already_passed(time_now)) {
                if (enable_motor_parking)
                    motor_control(PARK);
                else
                    motor_control(STOP);
            }
            break;
        case MOS_RUNNING_BACKWARD:
            if (time_to_stop_has_already_passed(now())) {
                motor_control(STOP);
            }
            break;
        case MOS_PARKING:
            if (digitalRead(MOTOR_PARKING)) {
                motor_control(STOP);
            }
            break;
        default:
            break;
    }
    // if temperature drops below the threshold(the fire goes out), there
    // is no point in starting the motor
    t_max = 0;
    for(byte i = 0; i < (min(SENSOR_NUMBER, one_wire_devices_count)); i++) {
        t_max = max(t_max, temperatures[i]);
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
    t_max = 0;
    for(byte i = 0; i < (min(SENSOR_NUMBER, one_wire_devices_count)); i++) {
        t_max = max(t_max, temperatures[i]);
    }
    // if temperature drops below the threshold(the fire goes out), there
    // is no point in starting the motor
    if (t_max < pump_stop_temperature) {
        pump_stop();
    }
    // but if the temperature rises above the threshold (relighting of
    // the furnace) we resume operation
    if (t_max > pump_start_temperature) {
        pump_start();
    }

/*******************************************************************************
 *                                                                             *
 * Menu control                                                                *
 *                                                                             *
 ******************************************************************************/
    if ((!digitalRead(ENC_BUTTON))
        && ((time_now_ms - menu_last_click) > menu_click_timeout)) {
        menu_last_click = time_now_ms;
        if (!in_settings_menu) {
            switch (main_menu_position) {
                case m_settings:
                    in_settings_menu = true;
                    settings_menu_position = m_settings_time_to_run;
                    sensor_menu_position = 0;
                    break;
                default:
                    break;
            }
        }
        else {
            if (settings_menu_position == m_settings_time_to_run
                || settings_menu_position == m_settings_time_to_stop
                || settings_menu_position == m_settings_temperature
                || settings_menu_position == m_settings_pump_start_temperature
                || settings_menu_position == m_settings_pump_stop_temperature) {
                if (editing_mode) {
                    lcd.noBlink();
                }
                else {
                    lcd.blink();
                }
            }
            switch (settings_menu_position) {
                case m_settings_time_to_run:
                case m_settings_time_to_stop:
                    if (editing_mode) {
                        editing_mode = false;
                        if (settings_menu_position == m_settings_time_to_run) {
                            time_to_run = editing_time_value;
                        }
                        else {
                            time_to_stop = editing_time_value;
                        }
                    }
                    else {
                        editing_mode = true;
                        if (settings_menu_position == m_settings_time_to_run)
                            editing_time_value = time_to_run;
                        else
                            editing_time_value = time_to_stop;
                    }
                    break;
                case m_settings_temperature:
                    if (editing_mode) {
                        editing_mode = false;
                        memcpy(sensor_addresses[sensor_menu_position],
                            one_wire_addresses[editing_sensor_index],
                            sizeof(sensor_addresses[0]));
                        free(one_wire_addresses);
                        one_wire_addresses = NULL;
                        free(one_wire_temperatures);
                        one_wire_temperatures = NULL;
                    }
                    else {
                        editing_mode = true;
                        count_one_wire_devices();
                        if (one_wire_devices_count > 0) {
                            one_wire_addresses = (byte (*)[8]) malloc(
                                one_wire_devices_count
                                * sizeof(one_wire_addresses[0]));
                            one_wire_temperatures = (int*) malloc(
                                one_wire_devices_count * sizeof(int));
                            scan_one_wire_bus(one_wire_addresses,
                                one_wire_devices_count);
                            editing_sensor_index = 0;
                            for(byte i = 0; i < one_wire_devices_count; i++) {
                                if (memcmp(
                                    sensor_addresses[sensor_menu_position],
                                    one_wire_addresses[i],
                                    sizeof(sensor_addresses[0])) == 0) {
                                    editing_sensor_index = i;
                                    break;
                                }
                            }
                        }
                    }
                    break;
                case m_settings_pump_start_temperature:
                case m_settings_pump_stop_temperature:
                    if (editing_mode) {
                        editing_mode = false;
                        if (settings_menu_position ==
                            m_settings_pump_start_temperature) {
                            pump_start_temperature = editing_temperature;
                        }
                        else {
                            pump_stop_temperature = editing_temperature;
                        }
                    }
                    else {
                        editing_mode = true;
                        if (settings_menu_position ==
                             m_settings_pump_start_temperature)
                            editing_temperature = pump_start_temperature;
                        else
                            editing_temperature = pump_stop_temperature;
                    }
                    break;
                case m_settings_motor_parking:
                    if (editing_mode) {
                        editing_mode = false;
                        enable_motor_parking = editing_parking;
                    }
                    else {
                        editing_mode = true;
                        editing_parking = enable_motor_parking;
                    }
                    break;
                case m_settings_one_wire_devices_count:
                    count_one_wire_devices();
                    break;
                case m_settings_store_to_eeprom:
                    eeprom_update_block(&time_to_run,
                        TIME_TO_RUN_EEPROM_ADDRESS, sizeof(time_t));
                    eeprom_update_block(&time_to_stop,
                        TIME_TO_STOP_EEPROM_ADDRESS, sizeof(time_t));
                    for(int i = 0; i < SENSOR_NUMBER; i++)
                        eeprom_update_block(
                            sensor_addresses[i],
                            SENSOR_ADDRESSES_EEPROM_ADDRESS + i * 8,
                            sizeof(sensor_addresses[0]));
                    eeprom_update_block(&pump_start_temperature,
                        PUMP_START_TEMPERATURE_EEPROM_ADDRESS, sizeof(int));
                    eeprom_update_block(&pump_stop_temperature,
                        PUMP_STOP_TEMPERATURE_EEPROM_ADDRESS, sizeof(int));
                    eeprom_update_block(&enable_motor_parking,
                        ENABLE_MOTOR_PARKING_EEPROM_ADDRESS, sizeof(byte));
                    lcd.setCursor(0, 1);
                    lcd.print("zapisane");
                    delay(250);
                    break;                    
                case m_settings_reset_to_factory:
                    time_to_run = DEF_TIME_TO_RUN;
                    time_to_stop = DEF_TIME_TO_STOP;
                    pump_start_temperature = DEF_TEMP_TO_START_PUMP;
                    pump_stop_temperature = DEF_TEMP_TO_STOP_PUMP;
                    lcd.setCursor(0, 1);
                    lcd.print("przywrocone");
                    delay(250);
                    break;
                case m_settings_return:
                    in_settings_menu = false;
                    main_menu_position = m_time_to_run;
                    sensor_menu_position = 0;
                    break;
                default:
                    break;
            }
        }
    }
    switch (encoder_status()) {
        case ENS_CW:
            if (editing_mode) {
                if (settings_menu_position == m_settings_time_to_run) {
                    if (editing_time_value < TIME_TO_RUN_MAX_SETTING)
                        editing_time_value += 60;  // 1 minute resolution
                }
                else if (settings_menu_position == m_settings_time_to_stop) {
                    if (editing_time_value < TIME_TO_STOP_MAX_SETTING)
                        editing_time_value += 1;  // 1 second resolution
                }
                else if (settings_menu_position == m_settings_temperature) {
                    if (editing_sensor_index < one_wire_devices_count - 1)
                        editing_sensor_index += 1;
                }
                else if (settings_menu_position ==
                    m_settings_pump_start_temperature
                    || settings_menu_position ==
                    m_settings_pump_stop_temperature) {
                    if (editing_temperature < TEMPERATURE_MAX_SETTING)
                        editing_temperature += 1 << DS18B20_FRACTIONAL_BITS;
                                               // 1 degree resolution
                }
                else if (settings_menu_position == m_settings_motor_parking)
                    editing_parking = !editing_parking;
            }
            else if (!in_settings_menu) {
                if (main_menu_position == m_temperature) {
                    if (sensor_menu_position == (SENSOR_NUMBER - 1))
                        main_menu_position++;
                    else
                        sensor_menu_position++;
                }
                else if (main_menu_position < (m_main_menu_last_pos - 1))
                    main_menu_position++;
            }
            else {
                if (settings_menu_position == m_settings_temperature) {
                    if (sensor_menu_position == (SENSOR_NUMBER - 1))
                        settings_menu_position++;
                    else
                        sensor_menu_position++;
                }
                else if (settings_menu_position <
                    (m_settings_menu_last_pos - 1))
                    settings_menu_position++;
            }
            break;
        case ENS_CCW:
            if (editing_mode) {
                if (settings_menu_position == m_settings_time_to_run) {
                    if (editing_time_value > 0)
                        editing_time_value -= 60;  // 1 minute resolution
                }
                else if (settings_menu_position == m_settings_time_to_stop) {
                    if (editing_time_value > 0)
                        editing_time_value -= 1;  // 1 second resolution
                }
                else if (settings_menu_position == m_settings_temperature) {
                    if (editing_sensor_index > 0)
                        editing_sensor_index -= 1;
                }
                else if (settings_menu_position ==
                    m_settings_pump_start_temperature
                    || settings_menu_position ==
                    m_settings_pump_stop_temperature) {
                    if (editing_temperature > 0)
                        editing_temperature -= 1 << DS18B20_FRACTIONAL_BITS;
                                               // 1 degree resolution
                }
                else if (settings_menu_position == m_settings_motor_parking)
                    editing_parking = !editing_parking;
            }
            else if (!in_settings_menu) {
                if (main_menu_position == m_temperature) {
                    if (sensor_menu_position == 0)
                        main_menu_position--;
                    else
                        sensor_menu_position--;
                }
                else if (main_menu_position > (m_main_menu_first_pos + 1))
                    main_menu_position--;
            }
            else {
                if (settings_menu_position == m_settings_temperature) {
                    if (sensor_menu_position == 0)
                        settings_menu_position--;
                    else
                        sensor_menu_position--;
                }
                else if (settings_menu_position >
                    (m_settings_menu_first_pos + 1))
                    settings_menu_position--;
            }
            break;
        default:
            break;
    }

/*******************************************************************************
 *                                                                             *
 * Display menu                                                                *
 *                                                                             *
 ******************************************************************************/
    lcd.clear();
    lcd.setCursor(0, 0);
    if (main_menu_position == m_temperature
        || settings_menu_position == m_settings_temperature)
        lcd.print(sensor_names[sensor_menu_position]);
    else if (!in_settings_menu) {
        lcd.print(menu_titles[main_menu_position]);
    }
    else {
        lcd.print(menu_titles[settings_menu_position]);
    }
    lcd.setCursor(0, 1);
    if (!in_settings_menu) {
        time_t time_left;
        int min, sec;
        char text[9];

        switch (main_menu_position) {
            case m_time_to_run:
            case m_time_to_stop:
                text[0] = '-';
                text[1] = '-';
                text[2] = 'm';
                text[3] = '-';
                text[4] = '-';
                text[5] = 's';
                text[6] = 0;

                if (motor_current_state != MOS_NOT_ACTIVE) {
                    if (motor_current_state == MOS_STOPPED
                        || motor_current_state == MOS_PARKING) {
                        if (main_menu_position == m_time_to_run)
                            time_left = remaining_time_to_start(time_now);
                        else
                            time_left = 0;
                    }
                    else {
                        if (main_menu_position == m_time_to_run)
                            time_left = 0;
                        else
                            time_left = remaining_time_to_stop(time_now);
                    }

                    min = time_left / 60;
                    sec = time_left % 60;
                    if (min < 10) {
                        text[0] = ' ';
                    }
                    else {
                        text[0] = '0' + min / 10 % 10;
                    }
                    text[1] = '0' + min % 10;
                    text[3] = '0' + sec / 10;
                    text[4] = '0' + sec % 10;
                }
                lcd.print(text);
                break;
            case m_temperature:
                fixedp_to_str(temperatures[sensor_menu_position], text,
                    sizeof(text), TEMP_INTEGER_DIGITS, TEMP_FRACTIONAL_DIGITS,
                    DS18B20_FRACTIONAL_BITS);
                lcd.print(text);
                lcd.write(DEGREE_SIGN);
                lcd.print("C");
                break;
            case m_motor_state:
                switch (motor_current_state) {
                    case MOS_STOPPED:
                        lcd.print("stop");
                        break;
                    case MOS_RUNNING_FORWARD:
                        lcd.print("do przodu");
                        break;
                    case MOS_RUNNING_BACKWARD:
                        lcd.print("do tylu");
                        break;
                    case MOS_PARKING:
                        lcd.print("zatrzymywanie");
                        break;
                    case MOS_NOT_ACTIVE:
                        lcd.print("nieaktywny");
                        break;
                    default:
                        break;
                }
                break;
            case m_pump_state:
                if (pump_current_state == PUS_RUNNING)
                    lcd.print("wl.");
                else
                    lcd.print("wyl.");
                break;
            default:
                lcd.print(" ");
                break;
        }
    }
    else {
        char text[9];

        switch (settings_menu_position) {
            case m_settings_time_to_run:
                if (editing_mode) {
                    _00ita(editing_time_value / 60, text);
                }
                else {
                    _00ita(time_to_run / 60, text);
                }
                text[2] = ' ';
                text[3] = 'm';
                text[4] = 'i';
                text[5] = 'n';
                text[6] = 0;
                lcd.print(text);
                if (editing_mode) {
                    lcd.setCursor(1, 1);
                }
                break;
            case m_settings_time_to_stop:
                if (editing_mode) {
                    _00ita(editing_time_value, text);
                }
                else {
                    _00ita(time_to_stop, text);
                }
                text[2] = ' ';
                text[3] = 's';
                text[4] = 'e';
                text[5] = 'c';
                text[6] = 0;
                lcd.print(text);
                if (editing_mode) {
                    lcd.setCursor(1, 1);
                }
                break;
            case m_settings_temperature:
                if (editing_mode)
                    fixedp_to_str(one_wire_temperatures[editing_sensor_index],
                        text, sizeof(text), TEMP_INTEGER_DIGITS,
                        TEMP_FRACTIONAL_DIGITS, DS18B20_FRACTIONAL_BITS);
                else
                    fixedp_to_str(temperatures[sensor_menu_position],
                        text, sizeof(text), TEMP_INTEGER_DIGITS,
                        TEMP_FRACTIONAL_DIGITS, DS18B20_FRACTIONAL_BITS);
                lcd.print(text);
                lcd.write(DEGREE_SIGN);
                lcd.print("C");
                if (editing_mode) {
                    lcd.setCursor(3, 1);
                }
                break;
            case m_settings_pump_start_temperature:
                if (editing_mode) {
                    fixedp_to_str(editing_temperature, text, sizeof(text),
                        TEMP_INTEGER_DIGITS, TEMP_FRACTIONAL_DIGITS,
                        DS18B20_FRACTIONAL_BITS);
                }
                else {
                    fixedp_to_str(pump_start_temperature, text, sizeof(text),
                        TEMP_INTEGER_DIGITS, TEMP_FRACTIONAL_DIGITS,
                        DS18B20_FRACTIONAL_BITS);
                }
                lcd.print(text);
                lcd.write(DEGREE_SIGN);
                lcd.print("C");
                if (editing_mode) {
                    lcd.setCursor(2, 1);
                }
                break;
            case m_settings_pump_stop_temperature:
                if (editing_mode) {
                    fixedp_to_str(editing_temperature, text, sizeof(text),
                        TEMP_INTEGER_DIGITS, TEMP_FRACTIONAL_DIGITS,
                        DS18B20_FRACTIONAL_BITS);
                }
                else {
                    fixedp_to_str(pump_stop_temperature, text, sizeof(text),
                        TEMP_INTEGER_DIGITS, TEMP_FRACTIONAL_DIGITS,
                        DS18B20_FRACTIONAL_BITS);
                }
                lcd.print(text);
                lcd.write(DEGREE_SIGN);
                lcd.print("C");
                if (editing_mode) {
                    lcd.setCursor(2, 1);
                }
                break;
            case m_settings_motor_parking:
                if (editing_mode)
                    if (editing_parking)
                        lcd.print("tak*nie ");
                    else
                        lcd.print("tak nie*");
                else
                    if (enable_motor_parking)
                        lcd.print("tak");
                    else
                        lcd.print("    nie");
                break;
            case m_settings_one_wire_devices_count:
                _00ita(one_wire_devices_count, text);
                text[2] = 0;
                lcd.print(text);
                break;
            case m_settings_reset_to_factory:
                lcd.print("fabryczne");
                break;
            case m_settings_return:
                lcd.setCursor(5, 1);
                lcd.write(RETURN_SIGN);
                break;
            default:
                lcd.print(" ");
                break;
        }
    }

    wdt_reset();
    delay(50);
}

void _00ita(int val, char *dest) {
    if (val < 10) {
        *dest = '0';
        *(dest + 1) = '0' + val;
    }
    else {
        *dest = '0' + (val / 10) % 10;
        *(dest + 1) = '0' + val % 10;
    }
}

/*******************************************************************************
 *                                                                             *
 * Motor control functions                                                     *
 *                                                                             *
 ******************************************************************************/
void motor_init(void) {
    pinMode(MOTOR_CON1, OUTPUT);
    digitalWrite(MOTOR_CON1, HIGH);
    pinMode(MOTOR_CON2, OUTPUT);
    digitalWrite(MOTOR_CON2, HIGH);
    pinMode(MOTOR_PARKING, INPUT_PULLUP);
}

void motor_control(enum motor_command command) {
    switch(command) {
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
    case PARK:
        motor_current_state = MOS_PARKING;
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

time_t remaining_time_to_start(time_t time) {
    int temperature = 0;
    byte i;

    for(i = 0; i < SENSOR_NUMBER; i++)
        temperature = max(temperature, temperatures[i]);
    // mix more frequently at low temperatures
    if (temperature < temperature_to_half_time) {
        return motor_start_time + time_to_run / 2 - time;
    }
    else {
        return motor_start_time + time_to_run - time;
    }
}

time_t remaining_time_to_stop(time_t time) {
    return motor_start_time + time_to_stop - time;
}

boolean time_to_run_has_already_passed(time_t time) {
    int temperature = 0;
    byte i;

    for(i = 0; i < SENSOR_NUMBER; i++)
        temperature = max(temperature, temperatures[i]);
    if (temperature < temperature_to_half_time) {
        return (time - motor_start_time) >= (time_to_run / 2);
    }
    else {
        return (time - motor_start_time) >= time_to_run;
    }
}

boolean time_to_stop_has_already_passed(time_t time) {
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
    byte dummy[8];
    byte i = 0;

    one_wire_bus.reset_search();
    while (one_wire_bus.search(dummy)) {
        i++;
    }
    one_wire_devices_count = i;

    return 0;
}

byte scan_one_wire_bus(byte (*addresses)[8], byte array_size) {
    byte dummy[8];
    byte i = 0;

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
    one_wire_bus.write(OW_CONVERT_T);
    measure_start_time = millis();
    measuring_current_state = MES_CONVERTING_BUSY;
}

void read_temperatures(int *temperatures, byte array_size) {
    byte scrachpad[9];
    byte i, j;

    for(i = 0; i < array_size; i++) {
        one_wire_bus.reset();
        if (editing_mode && one_wire_addresses != NULL)
            one_wire_bus.select(one_wire_addresses[i]);
        else
            one_wire_bus.select(sensor_addresses[i]);
        one_wire_bus.write(OW_READ_SCRATCHPAD);
        for(j = 0; j < 9; j++) {
            scrachpad[j] = one_wire_bus.read();
        }
        temperatures[i] = (scrachpad[1] << 8) | scrachpad[0];
    }
    measuring_current_state = MES_IDLE;
}

void set_resolution(byte (*addresses)[8], byte array_size, byte resolution) {
    byte scrachpad[9];
    byte i, j;

    for(i = 0; i < array_size; i++) {
        one_wire_bus.reset();
        one_wire_bus.select(addresses[i]);
        one_wire_bus.write(OW_READ_SCRATCHPAD);
        for(j = 0; j < 9; j++) {
            scrachpad[j] = one_wire_bus.read();
        }
        if (resolution ==  9) scrachpad[4] = 0x1F;  // & 0x9F | 0x00
        if (resolution == 10) scrachpad[4] = 0x3F;  // & 0xBF | 0x20
        if (resolution == 11) scrachpad[4] = 0x5F;  // & 0xDF | 0x40
        if (resolution == 12) scrachpad[4] = 0x7F;  // & 0xFF | 0x60
        one_wire_bus.reset();
        one_wire_bus.select(addresses[i]);
        one_wire_bus.write(OW_WRITE_SCRATCHPAD);
        one_wire_bus.write_bytes(&scrachpad[2], 3);
    }
    one_wire_bus.reset();
}

void fixedp_to_str(int value, char *buffer, int buffer_size,
    byte integer_digits, byte fractional_digits, byte point_pos) {
    int integer_part, fractional_part;
    bool negative;
    byte i;
    char digits[5];

    if ((integer_digits + fractional_digits + 3) > buffer_size)
        return;

    negative = value < 0;
    if (negative) value = -value;

    if (integer_digits > 5) integer_digits = 5;
    if (fractional_digits > 5) fractional_digits = 5;

    integer_part = value >> point_pos;
    fractional_part = value & ~(0xFFFF << point_pos);
    // (.) >> 0  0000 0000   0xFF << 0  1111 1111   ~(.)  0000 0000
    // (.) >> 1  0000 000.   0xFF << 1  1111 1110   ~(.)  0000 0001
    // (.) >> 2  0000 00..   0xFF << 2  1111 1100   ~(.)  0000 0011
    // (.) >> 3  0000 0...   0xFF << 3  1111 1000   ~(.)  0000 0111
    // (.) >> 4  0000 ....   0xFF << 4  1111 0000   ~(.)  0000 1111
    //...

    digits[4] = (integer_part % 10) + '0';
    integer_part = integer_part / 10;
    for(i = 1; i < integer_digits; i++) {
        if (integer_part > 0) {
            digits[4 - i] = (integer_part % 10) + '0';
            integer_part = integer_part / 10;
        }
        else
            digits[4 - i] = ' ';
    }

    buffer[0] = negative ? '-': '+';

    for(i = 0; i < integer_digits; i++) {
        buffer[1 + i] = digits[5 - integer_digits + i];
    }

    buffer[1+integer_digits] = '.';

    for(i = 0; i < fractional_digits; i++) {
        digits[i] = (((fractional_part * 10) / (1 << point_pos)) % 10) + '0';
        fractional_part = fractional_part * 10;
    }

    for(i = 0; i < fractional_digits; i++) {
        buffer[1 + integer_digits + 1 + i] = digits[i];
    }

    buffer[1 + integer_digits + 1 + fractional_digits] = 0;
}

/*******************************************************************************
 *                                                                             *
 * AVT1722 board "AVTduino miniLCD" encoder                                    *
 *                                                                             *
 ******************************************************************************/
void encoder_init(void) {
    pinMode(ENC_A, INPUT);
    pinMode(ENC_B, INPUT);
    pinMode(ENC_BUTTON, INPUT);

    enc = 0;
    enc_last = enc;
}

uint8_t encoder_status(void) {
    enc = encoder.read();
    if ((enc / ENCODER_PULSES_PER_MENU)
        > (enc_last / ENCODER_PULSES_PER_MENU)) {
        enc_last = enc;
        return ENS_CW;
    }
    if ((enc / ENCODER_PULSES_PER_MENU)
        < (enc_last / ENCODER_PULSES_PER_MENU)) {
        enc_last = enc;
        return ENS_CCW;
    }
    return ENS_NO_CHANGE;
}

