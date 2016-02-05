#ifndef HECTOR_MONSTERTRUCK_DRIVER_H
#define HECTOR_MONSTERTRUCK_DRIVER_H

#include <ros/ros.h>

#include <string>
#include <monstertruck_msgs/Status.h>
#include <monstertruck_msgs/RawOdometry.h>
#include <sensor_msgs/Imu.h>
#include <monstertruck_msgs/Compass.h>
#include <monstertruck_msgs/ServoCommands.h>
#include <std_msgs/Bool.h>

#include "definitions.h"
#include "ublox.h"
#include <hector_monstertruck_driver/SerialDevice.h>

class HectorMonstertruckDriver
{

private:

    ros::Publisher statusPub_;
    ros::Publisher odometryPub_;
    ros::Publisher imuPub_;
    ros::Publisher compassPub_;

    ros::Subscriber servoCommandSub_;
    ros::Subscriber backwardSub_;
    bool isBackwards;


    int axis_AccelX, axis_AccelY, axis_AccelZ, axis_GyroX, axis_GyroY, axis_GyroZ;
    double c0_AccelX, c1_AccelX, c0_AccelY, c1_AccelY, c0_AccelZ, c1_AccelZ, c0_GyroX, c1_GyroX, c0_GyroY, c1_GyroY, c0_GyroZ, c1_GyroZ;
    double c0_CompassX, c0_CompassY;

    std::string com_port;
    int baudrate;

    SerialDevice *device;

    bool servoCommandsNewData;

    ros::Time now;

    struct {
        monstertruck_msgs::Status status;
        monstertruck_msgs::RawOdometry odometry;
        sensor_msgs::Imu imu;
        monstertruck_msgs::Compass compass;
        monstertruck_msgs::ServoCommands servoCommands;
        NAV_API_RAW_SERVO_DATA servoRaw;
        std_msgs::Bool backward;
    } data;

    char bufferIn[1024];
    size_t bufferInLength;
    char bufferOut[1024];
    size_t bufferOutLength;

    UBloxInterpreter interfaceUBlox;

    unsigned int CommandsProcessed;            //!< Number of sucessfully processed packets
    unsigned int ErrorId;                      //!< Id: error Id; 0: no error
    unsigned int ErrorValue;                   //!< Value to add information to the ErrorId
    unsigned int GpsSendQueueLoad;             //!< Bytes in the GPS send queue
    bool         updatedStatus;

    sensor_msgs::Imu meanImu;
    int meanImu_count;

public:
    HectorMonstertruckDriver();

    bool configure();
    void cleanup();
    bool start();
    void stop();

    void spin();

private:

    void servoCommandsCb(const monstertruck_msgs::ServoCommands::ConstPtr& msg);
    void backwardCb(const std_msgs::Bool::ConstPtr& msg);

    bool readDataFromInterfaceBoard();
    bool sendDataToInterfaceBoard();

    bool send(void *message, size_t size, uint8_t classId, uint8_t messageId);
    bool configure(void *message, size_t size, uint8_t classId, uint8_t messageId);

    bool zeroCommandUpdate(sensor_msgs::Imu const& imu);
};

#endif // HECTOR_MONSTERTRUCK_DRIVER_H
