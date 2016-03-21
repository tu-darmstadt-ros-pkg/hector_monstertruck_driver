// Do not remove the include below

#include "Arduino.h"
//Arduino librarys
#include <Servo.h>
#include <SPI.h>

#include <ros.h>
#include <ros/time.h>
#include <monstertruck_msgs/Status.h>
#include <monstertruck_msgs/RawOdometry.h>
#include <sensor_msgs/Imu.h>
#include <monstertruck_msgs/ServoCommands.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt32.h>

#include <monstertruck_model.h>

ros::NodeHandle nh;

monstertruck_msgs::Status status_msg;
ros::Publisher status_pub("status", &status_msg);

long last_msg = 0;
monstertruck_msgs::RawOdometry odom_msg;
ros::Publisher odom_pub("odom", &odom_msg);

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

bool publish_gyroX = true;
std_msgs::Float32 gyroX_msg;
ros::Publisher gyroX_pub("gyroX", &gyroX_msg);

bool publish_gyroY = true;
std_msgs::Float32 gyroY_msg;
ros::Publisher gyroY_pub("gyroY", &gyroY_msg);

bool publish_gyroZ = true;
std_msgs::Float32 gyroZ_msg;
ros::Publisher gyroZ_pub("gyroZ", &gyroZ_msg);

bool publish_acclX = false;
std_msgs::Float32 acclX_msg;
ros::Publisher acclX_pub("acclX", &acclX_msg);

bool publish_acclY = false;
std_msgs::Float32 acclY_msg;
ros::Publisher acclY_pub("acclY", &acclY_msg);

bool publish_acclZ = false;
std_msgs::Float32 acclZ_msg;
ros::Publisher acclZ_pub("acclZ", &acclZ_msg);

bool publish_rad1 = true;
std_msgs::UInt32 rad1_msg;
ros::Publisher rad1_pub("rad1", &rad1_msg);

bool publish_rad2 = true;
std_msgs::UInt32 rad2_msg;
ros::Publisher rad2_pub("rad2", &rad2_msg);

bool publish_rad3 = true;
std_msgs::UInt32 rad3_msg;
ros::Publisher rad3_pub("rad3", &rad3_msg);

bool publish_rad4 = true;
std_msgs::UInt32 rad4_msg;
ros::Publisher rad4_pub("rad4", &rad4_msg);

void servo_commands_cb( const monstertruck_msgs::ServoCommands& msg);
ros::Subscriber<monstertruck_msgs::ServoCommands> servo_commands_sub("servoCommands", servo_commands_cb);

void backwards_cb( const std_msgs::Bool& msg);
ros::Subscriber<std_msgs::Bool> backwards_sub("backwards", backwards_cb);

void publishStatus();
void voltageRead(float &inputVoltage1, float &inputVoltage2);
void publishOdom();
void publishImu();
void processImu();

volatile uint32_t count_rad_1 = 0;
volatile uint32_t count_rad_2 = 0;
volatile uint32_t count_rad_3 = 0;
volatile uint32_t count_rad_4 = 0;

#define Rad_1 2 //Pin 21
#define Rad_2 3 //Pin 20
#define Rad_3 4 //Pin 19
#define Rad_4 5 //Pin 18

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


#define IMU_CS 49

unsigned short spi_adis16350_read_once(byte Address) {
    Address = Address & 0x3F; //Set 7. and MSB to 0
    digitalWrite(IMU_CS, LOW);
    SPI.transfer(Address);
    SPI.transfer(0x00);
    //delay(10);
    digitalWrite(IMU_CS, HIGH);
    unsigned short ReadData = 0;
    digitalWrite(IMU_CS, LOW);
    delay(1); //Chip select to clock edge: 50ns
    ReadData = SPI.transfer(Address);
    ReadData = ReadData << 8;
    ReadData += SPI.transfer(0x00);
    //delay(10);
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
    //delay(10);
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
    //delay(10);
    digitalWrite(IMU_CS, HIGH);

    return ReadData;
}

int16_t convert_adisData(uint16_t gyroOut){
    gyroOut = gyroOut & 0x3FFF; //Remove leading status bits
    if((gyroOut >> 13) != 0){ //Is negativ number
        int16_t out = (int16_t) gyroOut;
        out = out - 16384;
        return out;
    }
    return (int16_t) gyroOut;
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
    delay(10);
    spi_adis16350_write(0x1A, 0x00); // XGYRO_OFF = 0x0000
    delay(10);
    spi_adis16350_write(0x1D, 0x00);
    delay(10);
    spi_adis16350_write(0x1C, 0x00); // YGYRO_OFF = 0x0000
    delay(10);
    spi_adis16350_write(0x1F, 0x00);
    delay(10);
    spi_adis16350_write(0x1E, 0x00); // ZGYRO_OFF = 0x0000
    delay(10);
    spi_adis16350_write(0x21, 0x00);
    delay(10);
    spi_adis16350_write(0x20, 0x00); // XACCL_OFF = 0x0000
    delay(10);
    spi_adis16350_write(0x23, 0x00);
    delay(10);
    spi_adis16350_write(0x22, 0x00); // YACCL_OFF = 0x0000
    delay(10);
    spi_adis16350_write(0x25, 0x00);
    delay(10);
    spi_adis16350_write(0x24, 0x00); // ZACCL_OFF = 0x0000
    delay(10);
    // configure sample rate and sensitivity/range
    spi_adis16350_write(0x37, 0x00);
    delay(10);
    spi_adis16350_write(0x36, 0x01); // SMPL_PRD = 0x0001
    delay(10);
    spi_adis16350_write(0x39, 0x04);
    delay(10);
    spi_adis16350_write(0x38, 0x02); // SENS/AVG = 0x0402
    delay(10);
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

bool writeServo(uint8_t servo, uint16_t position) {
    if (servo < 1 || servo > 6) {
        return false;
    }
    if (position > 2000) {
        position = 2000;
    }
    servos[servo-1].write(position);
    return true;
}

int axis_AccelX, axis_AccelY, axis_AccelZ, axis_GyroX, axis_GyroY, axis_GyroZ;
float c0_AccelX, c1_AccelX, c0_AccelY, c1_AccelY, c0_AccelZ, c1_AccelZ, c0_GyroX, c1_GyroX, c0_GyroY, c1_GyroY, c0_GyroZ, c1_GyroZ;
float c0_CompassX, c0_CompassY;

float ACCEL_SCALE = 0.002522 * 9.81; //LSB to m/s^2
float GYRO_SCALE = 0.07326 * 0.017453293; //LSB to rad/sec

void readParams(){

    axis_AccelX = 1;
    //nh.getParam("axis_AccelX", &axis_AccelX);

    axis_AccelY = 0;
    //nh.getParam("axis_AccelY", &axis_AccelY);

    axis_AccelZ = 2;
    //nh.getParam("axis_AccelZ", &axis_AccelZ);

    axis_GyroX = 1;
    //nh.getParam("axis_GyroX", &axis_GyroX);

    axis_GyroY = 0;
    //nh.getParam("axis_GyroY", &axis_GyroY);

    axis_GyroZ = 2;
    //nh.getParam("axis_GyroZ", &axis_GyroZ);

    //c0_AccelX = 0.1712259326732673281323827f;
    c0_AccelX = 0;
    //nh.getParam("c0_AccelX", &c0_AccelX);

    //c1_AccelX = 0.006185200000000000072897244f;
    c1_AccelX = ACCEL_SCALE;
    //nh.getParam("c1_AccelX", &c1_AccelX);

    //c0_AccelY = 1.816366653465347091866988f;
    c0_AccelY = 0;
    //nh.getParam("c0_AccelY", &c0_AccelY);

    //c1_AccelY = -0.006185200000000000072897244f;
    c1_AccelY = ACCEL_SCALE;
    //nh.getParam("c1_AccelY", &c1_AccelY);

    //c0_AccelZ = -2.177110661386136847283979f;
    c0_AccelZ = 0;
    //nh.getParam("c0_AccelZ", &c0_AccelZ);

    //c1_AccelZ = -0.006185200000000000072897244f;
    c1_AccelZ = ACCEL_SCALE;
    //nh.getParam("c1_AccelZ", &c1_AccelZ);

    //c0_GyroX = 0.05943721247524752504531875f;
    c0_GyroX = 0;
    //nh.getParam("c0_GyroX", &c0_GyroX);

    //c1_GyroX = -0.0003196569999999999923609439f;
    c1_GyroX = GYRO_SCALE;
    //nh.getParam("c1_GyroX", &c1_GyroX);

    //c0_GyroY = 0.001126711801980199989525744f;
    c0_GyroY = 0;
    //nh.getParam("c0_GyroY", &c0_GyroY);

    //c1_GyroY = 0.0003196569999999999923609439f;
    c1_GyroY = GYRO_SCALE;
    //nh.getParam("c1_GyroY", &c1_GyroY);

    //c0_GyroZ = 0.02021751401980198536989519f;
    c0_GyroZ = 0;
    //nh.getParam("c0_GyroZ", &c0_GyroZ);

    //c1_GyroZ = 0.0003196569999999999923609439f;
    c1_GyroZ = GYRO_SCALE;
    //nh.getParam("c1_GyroZ", &c1_GyroZ);

    c0_CompassX = 0.0f;
    //nh.getParam("c0_CompassX", &c0_CompassX);

    c0_CompassY = 0.0f;
    //nh.getParam("c0_CompassY", &c0_CompassY);

}

void setup()
{

    attachInterrupt(Rad_1, interruptRad1, CHANGE);
    attachInterrupt(Rad_2, interruptRad2, CHANGE);
    attachInterrupt(Rad_3, interruptRad3, CHANGE);
    attachInterrupt(Rad_4, interruptRad4, CHANGE);

    initIMU();

    initServos();

    nh.initNode();

    //TODO Parameter not working right now
    //while(!nh.connected()) {nh.spinOnce();}

    readParams();

    nh.advertise(status_pub);
    nh.advertise(odom_pub);
    nh.advertise(imu_pub);
    nh.subscribe(backwards_sub);
    nh.subscribe(servo_commands_sub);

    if(publish_gyroX){
        nh.advertise(gyroX_pub);
    }
    if(publish_gyroY){
        nh.advertise(gyroY_pub);
    }
    if(publish_gyroZ){
        nh.advertise(gyroZ_pub);
    }
    if(publish_acclX){
        nh.advertise(acclX_pub);
    }
    if(publish_acclY){
        nh.advertise(acclY_pub);
    }
    if(publish_acclZ){
        nh.advertise(acclZ_pub);
    }

    if(publish_rad1){
        nh.advertise(rad1_pub);
    }

    if(publish_rad2){
        nh.advertise(rad2_pub);
    }

    if(publish_rad3){
        nh.advertise(rad3_pub);
    }

    if(publish_rad4){
        nh.advertise(rad4_pub);
    }

}

void loop()
{
    nh.spinOnce();

    publishStatus();

    publishOdom();

    processImu();

}

void publishStatus(){
    status_msg.header.seq++;
    status_msg.header.stamp = nh.now();
    status_msg.status = monstertruck_msgs::Status::AUTONOMOUS;
    voltageRead(status_msg.voltage1, status_msg.voltage2);

    status_pub.publish(&status_msg);
}

void voltageRead(float &inputVoltage1, float &inputVoltage2){
    inputVoltage1 = (24.0 / 1024) * analogRead(A0);                //!< Input voltage Bat1 [mV]
    inputVoltage2 = (24.0 / 1024) * analogRead(A1);                //!< Input voltage Bat2 [mV]
}

bool isBackwards = false;

void publishOdom(){

    if(publish_rad1){
        rad1_msg.data = count_rad_1;
        rad1_pub.publish(&rad1_msg);
    }

    if(publish_rad2){
        rad2_msg.data = count_rad_2;
        rad2_pub.publish(&rad2_msg);
    }

    if(publish_rad3){
        rad3_msg.data = count_rad_3;
        rad3_pub.publish(&rad3_msg);
    }

    if(publish_rad4){
        rad4_msg.data = count_rad_4;
        rad4_pub.publish(&rad4_msg);
    }

    float sign = 1.0;

    if (isBackwards == true) sign = -1.0;

    long old[4] = { odom_msg.tics_fl, odom_msg.tics_rl, odom_msg.tics_rr, odom_msg.tics_fr };

    long dt = millis() - last_msg;
    last_msg = millis();

    odom_msg.header.seq++;
    odom_msg.header.stamp = nh.now();

    //fake rad 2
    long diff_right = count_rad_4 - count_rad_3;
    count_rad_2 = count_rad_1 - diff_right;


    odom_msg.tics_fl = count_rad_1;     // front left
    odom_msg.tics_rl = count_rad_2;     // rear left
    odom_msg.tics_rr = count_rad_3;     // rear right
    odom_msg.tics_fr = count_rad_4;     // front right

    odom_msg.v_fl = sign * (odom_msg.tics_fl - old[0]) / 240.0 * 0.5 / dt;
    odom_msg.v_rl = sign * (odom_msg.tics_rl - old[1]) / 240.0 * 0.5 / dt;
    odom_msg.v_rr = sign * (odom_msg.tics_rr - old[2]) / 240.0 * 0.5 / dt;
    odom_msg.v_fr = sign * (odom_msg.tics_fr - old[3]) / 240.0 * 0.5 / dt;

    odom_msg.speed   = ( odom_msg.v_fl + odom_msg.v_rl + odom_msg.v_rr + odom_msg.v_fr) / 4;
    odom_msg.yawRate = (-odom_msg.v_fl - odom_msg.v_rl + odom_msg.v_rr + odom_msg.v_fr) / 4 / (Monstertruck::MonstertruckModel::getWheelTrack() / 2.0);

    odom_pub.publish(&odom_msg);
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

enum IMU_STATE{IDLE, GYRO_X, GYRO_Y, GYRO_Z, ACCL_X, ACCL_Y, ACCL_Z, TEMP_X, TEMP_Y, TEMP_Z, MSG} imu_state;

int16_t gyroX = 0;
int16_t gyroY = 0;
int16_t gyroZ = 0;
int16_t acclX = 0;
int16_t acclY = 0;
int16_t acclZ = 0;
int16_t tempX = 0;
int16_t tempY = 0;
int16_t tempZ = 0;

uint32_t imu_timeout;
uint32_t IMU_DELAY = 10;

void processImu(){
    switch(imu_state){
    case IDLE:
        if(imu_timeout < millis()){
            spi_adis16350_read(0x04);
            imu_timeout = millis() + IMU_DELAY;
            imu_state = GYRO_X;
        }
        break;
    case GYRO_X:
        if(imu_timeout < millis()){
            uint16_t gyroX_raw = spi_adis16350_read(0x06);
            gyroX = convert_adisData(gyroX_raw);

            imu_timeout = millis() + IMU_DELAY;
            imu_state = GYRO_Y;
            if(publish_gyroX){
                gyroX_msg.data = gyroX * GYRO_SCALE;
                gyroX_pub.publish(&gyroX_msg);
            }
        }
        break;
    case GYRO_Y:
        if(imu_timeout < millis()){
            uint16_t gyroY_raw = spi_adis16350_read(0x08);
            gyroY = convert_adisData(gyroY_raw);
            imu_timeout = millis() + IMU_DELAY;
            imu_state = GYRO_Z;
            if(publish_gyroY){
                gyroY_msg.data = gyroY * GYRO_SCALE;
                gyroY_pub.publish(&gyroY_msg);
            }
        }
        break;
    case GYRO_Z:
        if(imu_timeout < millis()){
            uint16_t gyroZ_raw = spi_adis16350_read(0x0A);
            gyroZ = convert_adisData(gyroZ_raw);
            imu_timeout = millis() + IMU_DELAY;
            imu_state = ACCL_X;
            if(publish_gyroZ){
                gyroZ_msg.data = gyroZ * GYRO_SCALE;
                gyroZ_pub.publish(&gyroZ_msg);
            }
        }
        break;
    case ACCL_X:
        if(imu_timeout < millis()){
            uint16_t raw = spi_adis16350_read(0x0C);
            acclX = convert_adisData(raw);
            imu_timeout = millis() + IMU_DELAY;
            imu_state = ACCL_Y;
            if(publish_acclX){
                acclX_msg.data = acclX * ACCEL_SCALE;
                acclX_pub.publish(&acclX_msg);
            }
        }
        break;
    case ACCL_Y:
        if(imu_timeout < millis()){
            acclY = convert_adisData(spi_adis16350_read(0x0E));
            imu_timeout = millis() + IMU_DELAY;
            imu_state = ACCL_Z;
            if(publish_acclY){
                acclY_msg.data = acclY * ACCEL_SCALE;
                acclY_pub.publish(&acclY_msg);
            }
        }
        break;
    case ACCL_Z:
        if(imu_timeout < millis()){
            uint16_t raw = spi_adis16350_read(0x10);
            acclZ = convert_adisData(raw);
            imu_timeout = millis() + IMU_DELAY;
            imu_state = MSG;
            if(publish_acclZ){
                acclZ_msg.data = acclZ * ACCEL_SCALE;
                acclZ_pub.publish(&acclZ_msg);
            }
        }
        break;
    case MSG:

        imu_msg.header.seq++;
        imu_msg.header.stamp = nh.now();
        imu_msg.header.frame_id = "imu";

        imu_msg.angular_velocity.x = gyroX * c1_GyroX  + c0_GyroX;
        imu_msg.angular_velocity.y = gyroY * c1_GyroY  + c0_GyroY;
        imu_msg.angular_velocity.z = gyroZ * c1_GyroZ  + c0_GyroZ;
        imu_msg.linear_acceleration.x = acclX * c1_AccelX + c0_AccelX;
        imu_msg.linear_acceleration.y = acclY * c1_AccelY + c0_AccelY;
        imu_msg.linear_acceleration.z = acclZ * c1_AccelZ + c0_AccelZ;

        imu_pub.publish(&imu_msg);
        imu_state = IDLE;
        break;

    }
}

void publishImu(){

    imu_msg.header.seq++;
    imu_msg.header.stamp = nh.now();

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
    /*
    imu_msg.angular_velocity.x = (gyroX - 32768)  * c1_GyroX  + c0_GyroX;
    imu_msg.angular_velocity.y = (gyroY - 32768)  * c1_GyroY  + c0_GyroY;
    imu_msg.angular_velocity.z = (gyroZ - 32768)  * c1_GyroZ  + c0_GyroZ;
    imu_msg.linear_acceleration.x = (acclX - 32768) * c1_AccelX + c0_AccelX;
    imu_msg.linear_acceleration.y = (acclY - 32768) * c1_AccelY + c0_AccelY;
    imu_msg.linear_acceleration.z = (acclZ - 32768) * c1_AccelZ + c0_AccelZ;
    */
    //OR
    /*
    imu_msg.angular_velocity.x = gyroX * c1_GyroX  + c0_GyroX;
    imu_msg.angular_velocity.y = gyroY * c1_GyroY  + c0_GyroY;
    imu_msg.angular_velocity.z = gyroZ * c1_GyroZ  + c0_GyroZ;
    imu_msg.linear_acceleration.x = acclX * c1_AccelX + c0_AccelX;
    imu_msg.linear_acceleration.y = acclY * c1_AccelY + c0_AccelY;
    imu_msg.linear_acceleration.z = acclZ * c1_AccelZ + c0_AccelZ;
*/
    imu_pub.publish(&imu_msg);
}


void servo_commands_cb( const monstertruck_msgs::ServoCommands& msg){
    for(size_t i = 0; i < msg.servo_length; ++i) {

        signed short SValue = 1500 + (signed short) (msg.servo[i].position * 500.0 + .5);

        if (SValue > 2500) SValue = 2500;
        if (SValue < 500) SValue = 500;

        writeServo(msg.servo[i].id, SValue);
    }
}


void backwards_cb( const std_msgs::Bool& msg){
    isBackwards = msg.data;
}
