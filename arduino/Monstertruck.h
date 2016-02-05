// Only modify this file to include
// - function definitions (prototypes)
// - include files
// - extern variable definitions
// In the appropriate section

#ifndef _MONSTERTRUCK_H_
#define _MONSTERTRUCK_H_
#include "Arduino.h"
//Arduino librarys
#include <Servo.h>
#include <SPI.h>
//add your includes for the project Arduino_Element here


//end of add your includes here
#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

void initServos();
void initIMU();

void sendPackage();
void sendImuMessage();
void sendStatusMessage();
void sendOdometrieMessage();

void readPackage();
bool findPackage();
void moveBufferContent();
bool isPackageComplete();
bool isChecksumCorrect();
bool handlePackage();
bool handlePreloadPackage();
bool handleServoPackage();

uint32_t getPLength();

bool isPacketRecived();
void handlePacket();
void resetPacket();

bool writeServo(uint8_t servo, uint16_t position);
void imuStatus(bool &power_error_low, bool &power_error_high, bool &control_reg_failure, bool &spi_communication_failure, bool &sensor_over_range, bool &self_test_failure, bool &gyro_x_failure, bool &gyro_y_failure, bool &gyro_z_failure, bool &accl_x_failure, bool &accl_y_failure, bool &accl_z_failure);
void imuRead(int16_t &gyroX, int16_t &gyroY, int16_t &gyroZ, int16_t &acclX, int16_t &acclY, int16_t &acclZ, int16_t &tempX, int16_t &tempY, int16_t &tempZ);
//add your function definitions for the project Arduino_Element here
void handleInputMessage();

//for debug
void printVoltage();
void printIMU();
void printIMUStatus();


//Do not add code below this line
#endif /* _MONSTERTRUCK_H_ */
