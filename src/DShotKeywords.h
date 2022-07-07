/************************************************************************/
/*  DShot Commands                                                      */
/*                                                                      */
/*  Source: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/   */
/*                                                                      */
/************************************************************************/

// Commands below are only executed when motors are stopped
#define DIGITAL_CMD_MOTOR_STOP 0  // Currently not implemented
#define DIGITAL_CMD_BEEP1 1  // Wait at least length of beep (260ms) before next command
#define DIGITAL_CMD_BEEP2 2  // Wait at least length of beep (260ms) before next command
#define DIGITAL_CMD_BEEP3 3  // Wait at least length of beep (280ms) before next command
#define DIGITAL_CMD_BEEP4 4  // Wait at least length of beep (280ms) before next command
#define DIGITAL_CMD_BEEP5 5  // Wait at least length of beep (1020ms) before next command
#define DIGITAL_CMD_ESC_INFO 6  // Wait at least 12ms before next command
#define DIGITAL_CMD_SPIN_DIRECTION_1 7   // Need 6x, no wait required
#define DIGITAL_CMD_SPIN_DIRECTION_2 8   // Need 6x, no wait required
#define DIGITAL_CMD_3D_MODE_OFF 9        // Need 6x, no wait required
#define DIGITAL_CMD_3D_MODE_ON 0         // Need 6x, no wait required
#define DIGITAL_CMD_SETTINGS_REQUEST 11  // Currently not implemented
#define DIGITAL_CMD_SAVE_SETTINGS 12     // Need 6x, wait at least 35ms before next command
#define DIGITAL_CMD_SPIN_DIRECTION_NORMAL 20    // Need 6x, no wait required
#define DIGITAL_CMD_SPIN_DIRECTION_REVERSED 21  // Need 6x, no wait required
#define DIGITAL_CMD_LED0_ON 22                  // No wait required
#define DIGITAL_CMD_LED1_ON 23                  // No wait required
#define DIGITAL_CMD_LED2_ON 24                  // No wait required
#define DIGITAL_CMD_LED3_ON 25                  // No wait required
#define DIGITAL_CMD_LED0_OFF 26                 // No wait required
#define DIGITAL_CMD_LED1_OFF 27                 // No wait required
#define DIGITAL_CMD_LED2_OFF 28                 // No wait required
#define DIGITAL_CMD_LED3_OFF 29                 // No wait required
#define AUDIO_STREAM_MODE 30  // Currently not implemented (Toggle on/off)
#define SILENT_MODE 31        // Currently not implemented (Toggle on/off)
#define DIGITAL_CMD_SIGNAL_LINE_TELEMETRY_DISABLE 32  // Need 6x, no wait required. Disables commands 42 to 47
#define DIGITAL_CMD_SIGNAL_LINE_TELEMETRY_ENABLE 33  // Need 6x, no wait required. Enables commands 42 to 47
#define DIGITAL_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY 34  \
  // Need 6x, no wait required. Enables commands 42 to 47, and sends erpm if normal Dshot frame
#define DIGITAL_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY 35  \
  // Need 6x, no wait required. Enables commands 42 to 47, and sends erpm period if normal Dshot frame
// Commands above are only executed when motors are stopped

/*
36 // Not yet assigned
37 // Not yet assigned
38 // Not yet assigned
39 // Not yet assigned
40 // Not yet assigned
41 // Not yet assigned
*/

// Commands below are executed at any time
#define DIGITAL_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY 42  // No wait required
#define DIGITAL_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY 43      // No wait required
#define DIGITAL_CMD_SIGNAL_LINE_CURRENT_TELEMETRY 44      // No wait required
#define DIGITAL_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY 45  // No wait required
#define DIGITAL_CMD_SIGNAL_LINE_ERPM_TELEMETRY 46         // No wait required
#define DIGITAL_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY 47  // No wait required
// The above commands are valid for Dshot and Proshot input signals

/*
DIGITAL_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY: 1°C per LSB
DIGITAL_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY: 10mV per LSB, 40.95V max
DIGITAL_CMD_SIGNAL_LINE_CURRENT_TELEMETRY: 100mA per LSB, 409.5A max
DIGITAL_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY: 10mAh per LSB, 40.95Ah max
DIGITAL_CMD_SIGNAL_LINE_ERPM_TELEMETRY: 100erpm per LSB, 409500erpm max
DIGITAL_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY: 16us per LSB, 65520us max TBD


ESC_INFO layout for BLHeli_32:
1-12: ESC SN
13: Indicates which response version is used. 254 is for BLHeli_32 version.
14: FW revision (32 = 32)
15: FW sub revision (10 = xx.1, 11 = xx.11)
16: Unused
17: Rotation direction reversed by dshot command or not (1:0)
18: 3D mode active or not (1:0)
19: Low voltage protection limit [0.1V] (255 = not capable, 0 = disabled)
20: Current protection limit [A] (255 = not capable, 0 = disabled)
21: LED0 on or not (1:0, 255 = LED0 not present)
22: LED1 on or not (1:0, 255 = LED1 not present)
23: LED2 on or not (1:0, 255 = LED2 not present)
24: LED3 on or not (1:0, 255 = LED3 not present)
25-31: Unused
32-63: ESC signature
64: CRC (same CRC as is used for telemetry)

*/

/*
ESC Telemetry
  Byte 0: Temperature 
  Byte 1: Voltage high byte
  Byte 2: Voltage low byte
  Byte 3: Current high byte
  Byte 4: Current low byte
  Byte 5: Consumption high byte
  Byte 6: Consumption low byte
  Byte 7: Rpm high byte
  Byte 8: Rpm low byte
  Byte 9: 8-bit CRC

Converting the received values to standard units
  int8_t Temperature = Temperature in 1 degree C
  uint16_t Voltage = Volt *100 so 1000 are 10.00V
  uint16_t Current = Ampere * 100 so 1000 are 10.00A
  uint16_t Consumption = Consumption in 1mAh
  uint16_t ERpm = Electrical Rpm /100 so 100 are 10000 Erpm

  note: to get the real Rpm of the motor you will need to divide the Erpm result 
  by the magnetpole count divided by two.

  So with a 14magnetpole motor:
  Rpm = Erpm/7
  rpm = erpm / (motor poles/2)
*/
