// Do not remove the include below
#include "Monstertruck.h"
#include "SerialUtil.hpp"
#include "UbloxUtils.hpp"

#define SYNC_BYTE 0x62

#define Servo_Brake 4
#define Servo_Antrieb 2

#define Rad_1 2 //Pin 21
#define Rad_2 3 //Pin 20
#define Rad_3 4 //Pin 19
#define Rad_4 5 //Pin 18

#define NAV_INTERFACE_VEHICLE_CLASS     0x10      //!< Class for UGV1
#define NAV_INTERFACE_COM_ID_STATUS     0x00      //!< ID for Status msg
#define NAV_INTERFACE_COM_ID_IMU        0x01      //!< ID for IMU msg
#define NAV_INTERFACE_COM_ID_GPS_IN     0x02      //!< ID for GPS_In msg
#define NAV_INTERFACE_COM_ID_COMPASS_2D 0x03      //!< ID for 2D-compass msg
#define NAV_INTERFACE_COM_ID_HODOMETER  0x04      //!< ID for Hodometer msg
#define NAV_INTERFACE_COM_ID_PRELOAD    0x11      //!< ID for Preload msg
#define NAV_INTERFACE_COM_ID_GPS_OUT    0x12      //!< ID for GPS_Out msg
#define NAV_INTERFACE_COM_ID_SERVO_OUT  0x13      //!< ID for Servo_out msg
#define NAV_INTERFACE_COM_ID_BAUDRATE   0x14      //!< ID for Baudrate msg
#define NAV_INTERFACE_COM_ID_PC_OUT     0x15      //!< ID for PC_Out msg

#define STATUS_MESSAGE_TOTAL_LENGTH 21

uint8_t CommandsProcessed = 0;
uint8_t ErrorId = 0;                      //!< Id: error Id; 0: no error
uint8_t ErrorValue = 0;             //!< Value to add information to the ErrorId
uint8_t GpsSendQueueLoad = 0;             //!< Bytes in the GPS send queue
uint8_t AutonomousModeFlag = 1;         //!< 1: autonoumous; <>1: not autonomous

uint8_t inputBuffer[1024];
unsigned int inputPos = 0; //Position inside buffer where next char can be placed

int packagePos = 0; //Start-Position of next Package

bool output_started = false; //Should the board give output via Serial

uint32_t output_time = 0; //Time where the next packages should be send
uint32_t output_cooldown = 0; //Timeout between the output messages

uint32_t count_rad_1 = 0;
uint32_t count_rad_2 = 0;
uint32_t count_rad_3 = 0;
uint32_t count_rad_4 = 0;

void interruptRad1() {
	count_rad_1++;
}
void interruptRad2() {
	count_rad_2++;
}
void interruptRad3() {
	count_rad_3++;
}
void interruptRad4() {
	count_rad_4++;
}

//The setup function is called once at startup of the sketch
void setup() {
	// Add your initialization code here

	initServos();
	initIMU();

	Serial.begin(115200);

	serialPrint("System started");

	pinMode(13, OUTPUT);
	digitalWrite(13, HIGH);
	attachInterrupt(Rad_1, interruptRad1, CHANGE);
	attachInterrupt(Rad_2, interruptRad2, CHANGE);
	attachInterrupt(Rad_3, interruptRad3, CHANGE);
	attachInterrupt(Rad_4, interruptRad4, CHANGE);
}

enum State {
	SYNC_PACKAGE, WAIT_COMPLETE, CHECKSUM, HANDLE
};

State state = SYNC_PACKAGE;

// The loop function is called in an endless loop
void loop() {

	readPackage();

	switch (state) {
	case SYNC_PACKAGE:
		if (findPackage()) {
			serialPrint("Package found");
			moveBufferContent();
			state = WAIT_COMPLETE;
		}
		break;
	case WAIT_COMPLETE:
		if (isPackageComplete()) {
			serialPrint("Package is complete");
			state = CHECKSUM;
		}
		break;
	case CHECKSUM:
		if (isChecksumCorrect()) {
			serialPrint("Checksum correct");
			state = HANDLE;
		} else {
			serialPrint("Checksum incorrect");
			state = SYNC_PACKAGE;
			packagePos++;
		}
		break;
	case HANDLE:
		if (handlePackage()) {
			serialPrint("Package handled");
			inputBuffer[packagePos] = 0x00; //Confirm Package
			inputBuffer[packagePos + 1] = 0x00;
			CommandsProcessed++;
		} else {
			serialPrint("Handle failed, moving on to next package");
			packagePos++;
		}
		state = SYNC_PACKAGE;
		break;
	}

	//sendPackage();
}

/**
 * Read packages from serial
 */
void readPackage() {

	while (Serial.available() > 0) {
		size_t length = 100;
		char buf[length];
		length = Serial.readBytes(buf, length);
		memcpy(&inputBuffer[inputPos], buf, length);
		inputPos += length;
		serialPrint("Bytes read: ", length);

		for (int i = 0; i < length; i++) {
			serialPrintFormated(buf[i], HEX);
		}
	}

}

/**
 * Set package pos to next package starting with the sync packages
 * returns true if succeded otherwise false
 */
bool findPackage() {
	while (inputBuffer[packagePos] != 0xB5
			&& inputBuffer[packagePos + 1] != 0x62 && packagePos < inputPos) {
		packagePos++;
	}
	return (packagePos < inputPos);
}

/**
 * Check if the current package is read completely
 */
bool isPackageComplete() {
	if (inputBuffer[packagePos] != 0xB5
			&& inputBuffer[packagePos + 1] == 0x62) {
		return false;
	}

	if (packagePos + 3 > inputPos) {
		return false;
	}

	uint32_t pLength = getPLength();

	serialPrint("Package size: ", pLength);

	return (packagePos + pLength < inputPos);
}

uint32_t getPLength() {
	uint32_t pLength = (uint8_t) inputBuffer[packagePos + 4];
	pLength |= (uint8_t) inputBuffer[packagePos + 5] << 8;

	return pLength;
}

bool isChecksumCorrect() {
	char checksumA = 0, checksumB = 0;

	int i = packagePos;

	i += 2; //Ublox Sync
	i += 2; //Class + Message Id
	i += 2; //pLength
	i += getPLength();

	int checksumAPos = i;
	int checksumBPos = i + 1;

	i += 2; //Checksum

	checksumA = inputBuffer[checksumAPos];
	checksumB = inputBuffer[checksumBPos];

	serialPrint("Checksum A Pos: ", checksumAPos);
	serialPrint("Checksum A: ", checksumA, HEX);
	serialPrint("Checksum B Pos: ", checksumBPos);
	serialPrint("Checksum B: ", checksumB, HEX);

	// checksum calculations
	for (i -= 3; i >= 2; i--) {
		checksumB -= checksumA;
		checksumA -= inputBuffer[i];
	}
	if (checksumA != 0 || checksumB != 0) {
		return false;
	}
	return true;
}

char getClassId() {
	return inputBuffer[packagePos + 2];
}

char getMessageId() {
	return inputBuffer[packagePos + 3];
}

/**
 * Handle the package
 * Need to adjust packagePos when correct
 */
bool handlePackage() {
	if (inputBuffer[packagePos] != 0xB5
			&& inputBuffer[packagePos + 1] == 0x62) {
		return false;
	}

	uint32_t pLength = getPLength();

	if (getClassId() != 0x10) {
		serialPrint("Received message not for us: ", getClassId());
		return false;
	}

	switch (getMessageId()) {
	case NAV_INTERFACE_COM_ID_PRELOAD:
		return handlePreloadPackage();
		break;
	case NAV_INTERFACE_COM_ID_SERVO_OUT:
		return handleServoPackage();
		break;
	default:
		serialPrint("Unknown message id: ", getMessageId(), HEX);
		break;
	}

	return false;
}

bool handlePreloadPackage() {
	char preloadLow = inputBuffer[packagePos + 6];
	char preloadHigh = inputBuffer[packagePos + 7];
	serialPrint("PreloadPackage");
	serialPrint("PreloadLow: ", preloadLow, HEX);
	serialPrint("PreloadHigh: ", preloadHigh, HEX);

	output_started = true;
	output_cooldown = 100;
	serialPrint("setting output_cooldown to (ms) ", output_cooldown);

	return true;
}

bool handleServoPackage(){
	serialPrint("ServoPackage");

	unsigned int servo_position = packagePos + 6;
	for(uint8_t id = 1; id<8; id++){
		uint16_t position = 0;
		position = inputBuffer[servo_position];
		position += (inputBuffer[servo_position+1] << 8);
		serialPrint("Servo id: ", id);
		serialPrint("Value: ", position);
		writeServo(id, position);
		servo_position += 2;
	}
	return true;
}

/**
 * Move the content of the inputBuffer forward so that packagePos will be 0 then.
 */
void moveBufferContent() {
	inputPos = inputPos - packagePos;

	memcpy(&inputBuffer[0], &inputBuffer[packagePos], inputPos);

	packagePos = 0;
}

/**
 * Check if packages should be sent to pc.
 */
void sendPackage() {
	if (output_started && output_time < millis()) {
		serialPrint("Sending packages");
		sendStatusMessage();
		sendOdometrieMessage();
		output_time = millis() + output_cooldown;
	}
}

int voltage1Pin = A0;
int voltage2Pin = A1;

void voltageRead(int &inputVoltage1, int &inputVoltage2){
	inputVoltage1 = analogRead(voltage1Pin);                //!< Input voltage Bat1 [mV]
	inputVoltage2 = analogRead(voltage2Pin);                //!< Input voltage Bat2 [mV]
}

/**
 * Send single status message to pc.
 */
void sendStatusMessage() {
	serialPrint("Sending status package");
	uint8_t buffer[STATUS_MESSAGE_TOTAL_LENGTH];

	ubloxSetHeader(buffer, NAV_INTERFACE_VEHICLE_CLASS,
			NAV_INTERFACE_COM_ID_STATUS, 13);

	ubloxSetInt(buffer, 6, millis());

	buffer[10] = CommandsProcessed;
	buffer[11] = ErrorId;
	buffer[12] = ErrorValue;
	buffer[13] = GpsSendQueueLoad;
	buffer[14] = AutonomousModeFlag;

	//TODO read analog pins
	int InputVoltage1;
	int InputVoltage2;
	voltageRead(InputVoltage1, InputVoltage2);

	ubloxSetShort(buffer, 15, InputVoltage1);
	ubloxSetShort(buffer, 17, InputVoltage2);

	ubloxSetChecksum(buffer, 19);

	Serial.write(buffer, STATUS_MESSAGE_TOTAL_LENGTH);
}

#define ODO_MESSAGE_TOTAL_LENGTH 28

void sendOdometrieMessage() {
	uint8_t buffer[ODO_MESSAGE_TOTAL_LENGTH];

	ubloxSetHeader(buffer, NAV_INTERFACE_VEHICLE_CLASS,
			NAV_INTERFACE_COM_ID_HODOMETER, 16);

	ubloxSetInt(buffer, 6, count_rad_1);
	ubloxSetInt(buffer, 10, count_rad_2);
	ubloxSetInt(buffer, 14, count_rad_3);
	ubloxSetInt(buffer, 18, count_rad_4);

	ubloxSetChecksum(buffer, 22);

	Serial.write(buffer, sizeof(buffer));
}

#define IMU_MESSAGE_TOTAL_LENGTH 40
void sendImuMessage() {
	uint8_t buffer[IMU_MESSAGE_TOTAL_LENGTH];

	int16_t gyroX = 0;
	int16_t gyroY = 0;
	int16_t gyroZ = 0;
	int16_t acclX = 0;
	int16_t acclY = 0;
	int16_t acclZ = 0;
	int16_t tempX = 0;
	int16_t tempY = 0;
	int16_t tempZ = 0;

	imuRead(gyroX, gyroY, gyroZ, acclX, acclY, acclZ, tempX, tempY, tempZ);

	ubloxSetHeader(buffer, NAV_INTERFACE_VEHICLE_CLASS,
			NAV_INTERFACE_COM_ID_IMU, 32);

	ubloxSetShort(buffer, 6, gyroX);
	ubloxSetShort(buffer, 8, gyroY);
	ubloxSetShort(buffer, 10, gyroZ);

	ubloxSetShort(buffer, 12, acclX);
	ubloxSetShort(buffer, 14, acclY);
	ubloxSetShort(buffer, 16, acclZ);

	ubloxSetShort(buffer, 18, 0);
	ubloxSetShort(buffer, 20, 0);
	ubloxSetShort(buffer, 22, 0);

	ubloxSetShort(buffer, 24, tempX);
	ubloxSetShort(buffer, 26, tempY);
	ubloxSetShort(buffer, 28, tempZ);

	ubloxSetShort(buffer, 30, 0);
	ubloxSetShort(buffer, 32, 0);
	ubloxSetShort(buffer, 34, 0);
	ubloxSetShort(buffer, 36, 0);

	ubloxSetChecksum(buffer, 38);

	Serial.write(buffer, sizeof(buffer));
}

void printVoltage(){
	int inputVoltage1;
	int inputVoltage2;

	voltageRead(inputVoltage1, inputVoltage2);

	serialPrint("InputVoltage1: ", inputVoltage1);
	serialPrint("InputVoltage2: ", inputVoltage2);
}

void printIMUStatus(){
	bool power_error_low, power_error_high, control_reg_failure, spi_communication_failure, sensor_over_range, self_test_failure, gyro_x_failure, gyro_y_failure, gyro_z_failure, accl_x_failure,accl_y_failure, accl_z_failure;
	imuStatus(power_error_low, power_error_high, control_reg_failure, spi_communication_failure, sensor_over_range, self_test_failure, gyro_x_failure, gyro_y_failure, gyro_z_failure, accl_x_failure,accl_y_failure, accl_z_failure);
	if(power_error_low){
		serialPrint("Power supply below 4.75 V");
	}else if(power_error_high){
			serialPrint("Power supply above 5.25 V");
	}else{
		serialPrint("Power supply normal");
	}

	if(control_reg_failure){
		serialPrint("Control register update failed");
	}

	if(spi_communication_failure){
		serialPrint("SPI communications failure");
	}
}

void printIMU() {
	int16_t gyroX = 0;
	int16_t gyroY = 0;
	int16_t gyroZ = 0;
	int16_t acclX = 0;
	int16_t acclY = 0;
	int16_t acclZ = 0;
	int16_t tempX = 0;
	int16_t tempY = 0;
	int16_t tempZ = 0;
	imuRead(gyroX, gyroY, gyroZ, acclX, acclY, acclZ, tempX, tempY, tempZ);

	serialPrint("gyroX: ", gyroX, DEC);
	serialPrint("gyroY: ", gyroY, DEC);
	serialPrint("gyroZ: ", gyroZ, DEC);
	serialPrint("acclX: ", acclX, DEC);
	serialPrint("acclY: ", acclY, DEC);
	serialPrint("acclZ: ", acclZ, DEC);
	serialPrint("tempX: ", tempX, DEC);
	serialPrint("tempY: ", tempY, DEC);
	serialPrint("tempZ: ", tempZ, DEC);

	double c0_GyroX = -0.01035964491089110678379015;
	double c1_GyroX = -0.0002181659999999999979478083;
	double c0_GyroY = 0.02048600340594060667531728;
	double c1_GyroY = 0.0002181659999999999979478083;
	double c0_GyroZ = -0.003905387405940576039942158;
	double c1_GyroZ = 0.0002181659999999999979478083;
	serialPrint("angular_velocity.x: ", gyroX * c1_GyroX + c0_GyroX);
	serialPrint("angular_velocity.y: ", gyroY * c1_GyroY + c0_GyroY);
	serialPrint("angular_velocity.z: ", gyroZ * c1_GyroZ + c0_GyroZ);
}

Servo servos[6];

void initServos() {
	servos[0].attach(2); //Servo CH1 Pin 2
	servos[1].attach(3); //Servo CH2 Pin 3
	servos[2].attach(4); //Servo CH3 Pin 4
	servos[3].attach(5); //Servo CH4 Pin 5
	servos[4].attach(6); //Servo CH5 Pin 6
	servos[5].attach(7); //Servo CH6 Pin 7
}

#define IMU_CS 49

unsigned short spi_adis16350_read_once(byte Address) {
	Address = Address & 0x3F; //Set 7. and MSB to 0
	digitalWrite(IMU_CS, LOW);
	SPI.transfer(Address);
	SPI.transfer(0x00);
	delay(10);
	digitalWrite(IMU_CS, HIGH);
	unsigned short ReadData = 0;
	digitalWrite(IMU_CS, LOW);
	delay(1); //Chip select to clock edge: 50ns
	ReadData = SPI.transfer(Address);
	ReadData = ReadData << 8;
	ReadData += SPI.transfer(0x00);
	delay(10);
	digitalWrite(IMU_CS, HIGH);
	return ReadData;
}

unsigned short spi_adis16350_read(byte Address) {
	Address = Address & 0x3F; //Set 7. and MSB to 0
	unsigned short ReadData = 0;
	digitalWrite(IMU_CS, LOW);
	delay(1); //Chip select to clock edge: 50ns
	ReadData = SPI.transfer(Address);
	ReadData = ReadData << 8;
	ReadData += SPI.transfer(0x00);
	delay(10);
	digitalWrite(IMU_CS, HIGH);

	return ReadData;
}

unsigned short spi_adis16350_write(byte Address, byte Data) {
	Address = Address | 0x80; //Set MSB to 1
	Address = Address & 0xBF; //Set 7. bit to 0
	unsigned short ReadData = 0;
	digitalWrite(IMU_CS, LOW);
	delay(1); //Chip select to clock edge: 50ns
	ReadData = SPI.transfer(Address);
	ReadData = ReadData << 8;
	ReadData += SPI.transfer(Data);
	delay(10);
	digitalWrite(IMU_CS, HIGH);

	return ReadData;
}

void initIMU() {
	pinMode(IMU_CS, OUTPUT);
	digitalWrite(IMU_CS, HIGH);
	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV32);
	//CPOL = 1
	//CPHA = 1
	SPI.setDataMode(SPI_MODE3);
	return;

	spi_adis16350_write(0x3E, 0x40); // software reset
	delay(10); //Wait a few milliseconds
	// configure offsets
	spi_adis16350_write(0x1B, 0x00);
	spi_adis16350_write(0x1A, 0x00); // XGYRO_OFF = 0x0000
	spi_adis16350_write(0x1D, 0x00);
	spi_adis16350_write(0x1C, 0x00); // YGYRO_OFF = 0x0000
	spi_adis16350_write(0x1F, 0x00);
	spi_adis16350_write(0x1E, 0x00); // ZGYRO_OFF = 0x0000
	spi_adis16350_write(0x21, 0x00);
	spi_adis16350_write(0x20, 0x00); // XACCL_OFF = 0x0000
	spi_adis16350_write(0x23, 0x00);
	spi_adis16350_write(0x22, 0x00); // YACCL_OFF = 0x0000
	spi_adis16350_write(0x25, 0x00);
	spi_adis16350_write(0x24, 0x00); // ZACCL_OFF = 0x0000

	// configure sample rate and sensitivity/range
	spi_adis16350_write(0x37, 0x00);
	spi_adis16350_write(0x36, 0x01); // SMPL_PRD = 0x0001
	spi_adis16350_write(0x39, 0x04);
	spi_adis16350_write(0x38, 0x02); // SENS/AVG = 0x0402
}

int16_t convert_adisData(uint16_t gyroOut){
	gyroOut = gyroOut & 0x3FFF;
	if((gyroOut >> 13) != 0){
		gyroOut = gyroOut | 0xC000;
	}
	return (int16_t) gyroOut;
}

void imuStatus(bool &power_error_low, bool &power_error_high, bool &control_reg_failure, bool &spi_communication_failure, bool &sensor_over_range, bool &self_test_failure, bool &gyro_x_failure, bool &gyro_y_failure, bool &gyro_z_failure, bool &accl_x_failure, bool &accl_y_failure, bool &accl_z_failure) {
	unsigned short status = spi_adis16350_read_once(0x3C);
	power_error_low = (status & 0x01) != 0;
	power_error_high = (status & 0x02) != 0;
	control_reg_failure = (status & 0x04) != 0;
	spi_communication_failure = (status & 0x08) != 0;
	sensor_over_range = (status & (1 << 4)) != 0;
	self_test_failure = (status & (1 << 5)) != 0;
	gyro_x_failure = (status & (1 << 10)) != 0;
	gyro_y_failure = (status & (1 << 11)) != 0;
	gyro_z_failure = (status & (1 << 12)) != 0;
	accl_x_failure = (status & (1 << 13)) != 0;
	accl_y_failure = (status & (1 << 14)) != 0;
	accl_z_failure = (status & (1 << 15)) != 0;
}

void imuRead(int16_t &gyroX, int16_t &gyroY,
		int16_t &gyroZ, int16_t &acclX, int16_t &acclY,
		int16_t &acclZ, int16_t &tempX, int16_t &tempY,
		int16_t &tempZ) {
	spi_adis16350_read(0x04);
	delay(100);
	uint16_t gyroX_raw = spi_adis16350_read(0x06);
	gyroX = convert_adisData(gyroX_raw);
	delay(100);
	uint16_t gyroY_raw = spi_adis16350_read(0x08);
	gyroY = convert_adisData(gyroY_raw);
	delay(100);
	uint16_t gyroZ_raw = spi_adis16350_read(0x0A);
	gyroZ = convert_adisData(gyroZ_raw);
	delay(100);
	acclX = convert_adisData(spi_adis16350_read(0x0C));
	delay(100);
	acclY = convert_adisData(spi_adis16350_read(0x0E));
	delay(100);
	acclZ = convert_adisData(spi_adis16350_read(0x10));
	delay(100);
	tempX = convert_adisData(spi_adis16350_read(0x12));
	delay(100);
	tempY = convert_adisData(spi_adis16350_read(0x14));
	delay(100);
	tempZ = convert_adisData(spi_adis16350_read(0x3C));
}

void batteryRead() {

}

bool writeServo(uint8_t servo, uint16_t position) {
	if (servo < 1 || servo > 6) {
		serialPrint("Invalid Servo id: ", servo);
		return false;
	}
	if (position > 2000) {
		serialPrint("Invalid Servo position, trimming to 2000");
		position = 2000;
	}
	servos[servo - 1].writeMicroseconds(position);
	return false;
}
