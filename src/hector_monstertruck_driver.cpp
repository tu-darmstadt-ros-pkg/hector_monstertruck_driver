#include "hector_monstertruck_driver/hector_monstertruck_driver.h"

#include <ublox.h>
#include "monstertruck_model.h"

HectorMonstertruckDriver::HectorMonstertruckDriver() :
  bufferInLength(0)
, bufferOutLength(0)
, meanImu()
, meanImu_count(-1)
{

    ros::NodeHandle _nh;
    ros::NodeHandle _pnh("~");

    statusPub_ = _pnh.advertise<monstertruck_msgs::Status>("status", 1000);
    odometryPub_ = _pnh.advertise<monstertruck_msgs::RawOdometry>("odometry", 1000);
    imuPub_ = _pnh.advertise<sensor_msgs::Imu>("imu", 1000);
    compassPub_ = _pnh.advertise<monstertruck_msgs::Compass>("compass", 1000);

    servoCommandSub_ = _pnh.subscribe("servoCommand", 1000, &HectorMonstertruckDriver::servoCommandsCb, this);
    backwardSub_ = _pnh.subscribe("backward", 1000, &HectorMonstertruckDriver::backwardCb, this);

    _pnh.param<int>("axis_AccelX", axis_AccelX, -1);
    _pnh.param<int>("axis_AccelY", axis_AccelY, -1);
    _pnh.param<int>("axis_AccelZ", axis_AccelZ, -1);

    _pnh.param<int>("axis_GyroX", axis_GyroX, -1);
    _pnh.param<int>("axis_GyroY", axis_GyroY, -1);
    _pnh.param<int>("axis_GyroZ", axis_GyroZ, -1);

    _pnh.param<double>("c0_AccelX", c0_AccelX, 0.0);
    _pnh.param<double>("c1_AccelX", c1_AccelX, 0.0);

    _pnh.param<double>("c0_AccelY", c0_AccelY, 0.0);
    _pnh.param<double>("c1_AccelY", c1_AccelY, 0.0);

    _pnh.param<double>("c0_AccelZ", c0_AccelZ, 0.0);
    _pnh.param<double>("c1_AccelZ", c1_AccelZ, 0.0);

    _pnh.param<double>("c0_GyroX", c0_GyroX, 0.0);
    _pnh.param<double>("c1_GyroX", c1_GyroX, 0.0);

    _pnh.param<double>("c0_GyroY", c0_GyroY, 0.0);
    _pnh.param<double>("c1_GyroY", c1_GyroY, 0.0);

    _pnh.param<double>("c0_GyroZ", c0_GyroZ, 0.0);
    _pnh.param<double>("c1_GyroZ", c1_GyroZ, 0.0);

    _pnh.param<double>("c0_CompassX", c0_CompassX, 0.0);
    _pnh.param<double>("c0_CompassY", c0_CompassY, 0.0);


    _pnh.param<std::string>("com_port", com_port, "/dev/ttyS0");

    _pnh.param<int>("baudrate", baudrate, 0);

    servoCommandsNewData = false;
    isBackwards = false;

    interfaceUBlox.setSync(NAV_INTERFACE_COM_SYNC);

}

void HectorMonstertruckDriver::servoCommandsCb(const monstertruck_msgs::ServoCommands::ConstPtr& msg){
    //TODO need to store in data.servoCommands
    data.servoCommands = *msg;
}

void HectorMonstertruckDriver::backwardCb(const std_msgs::Bool::ConstPtr& msg){
    isBackwards = msg->data;
}


bool HectorMonstertruckDriver::configure(){
    ROS_INFO("Configure Device ...");
    ROS_INFO("Com-Port: %s", this->com_port.c_str());
    ROS_INFO("Baudrate: %d", this->baudrate);
    device = new SerialDevice(this->com_port, this->baudrate);
    if (!device->configure()) {
        ROS_WARN("...failed");
        delete device;
        device = 0;
        return false;
    }
    ROS_INFO("... done");
    return true;
}


void HectorMonstertruckDriver::cleanup()
{
    if (device) {
        delete device;
        device = 0;
    }
}

bool HectorMonstertruckDriver::start()
{

    // Reset all servo signals
    for(size_t i = 0; i < data.servoRaw.CHANNELS; i++) data.servoRaw.Servo[i] = 1500;

    /* configuring Interface Board */
    ROS_INFO("starting interface board...");

    if (!configure(NAV_INTERFACE_COMMAND_START_50HZ, sizeof(NAV_INTERFACE_COMMAND_START_50HZ), NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_PRELOAD)) {
        ROS_ERROR("failed to start interface board");
        return false;
    }

    ROS_WARN("ready and running.");

    return true;

}


void HectorMonstertruckDriver::stop()
{

    ROS_INFO("sending stop command...");
    send(NAV_INTERFACE_COMMAND_STOP, sizeof(NAV_INTERFACE_COMMAND_STOP), NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_PRELOAD);
}

void HectorMonstertruckDriver::spin()
{
    // read data from Interface Board and publich results on output ports
    readDataFromInterfaceBoard();

    if(servoCommandsNewData){
        signed short SValue;

        for(size_t i = 0; i < data.servoCommands.servo.size(); ++i) {
            if (data.servoCommands.servo[i].id >= data.servoRaw.CHANNELS) continue;
            SValue = 1500 + (signed short) (data.servoCommands.servo[i].position * 500.0 + .5);
            if (SValue > 2500) SValue = 2500;
            if (SValue < 500) SValue = 500;
            data.servoRaw.Servo[data.servoCommands.servo[i].id] = SValue;
        }

        if (!send(&data.servoRaw, sizeof(data.servoRaw), NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_SERVO_OUT)) {
            ROS_ERROR("Could not send servo commands to interface board");// << uBlox.strlasterror() << endlog();
        }

        // trigger data from output buffer to be sent to the Interface Board
        sendDataToInterfaceBoard();
    }

}


bool HectorMonstertruckDriver::readDataFromInterfaceBoard()
{
    ROS_DEBUG("Reading data from interface board");
    void *payload;
    unsigned int pos, length, checksum_errors, processed;
    UBloxHeader header;
    int ret;

    if (!device) return false;

    bufferInLength = sizeof(bufferIn);

    //send(0,0, NAV_INTERFACE_VEHICLE_CLASS, NAV_INTERFACE_COM_ID_PC_OUT);
    //sendDataToInterfaceBoard(); //Inform Interface-Board to Send Data once

    if (!device->receive(bufferIn, &bufferInLength)) {
        ROS_DEBUG("No data present");
        return false;
    }

    ros::Time Now = ros::Time::now();
    double dt = (Now - now).toSec();
    if (now.isZero()) dt = 0.0;
    now = Now;

    pos = 0;
    checksum_errors = 0;
    processed = 0;
    while (pos < bufferInLength)
    {
        ret = interfaceUBlox.decode((char *)(bufferIn + pos), bufferInLength - pos, &payload, &length, &header);
        if (ret <= 0){
            switch(-ret){
            case UBLOX_ERROR_NOUBLOX: pos++; break;
            case UBLOX_ERROR_CHECKSUM:  ROS_WARN("checksum error: classId=%x messageId=%x length=%d", header.classId, header.messageId, length);
                checksum_errors++; pos += length + UBLOX_FRAMESIZE; break;
            case UBLOX_ERROR_BIGPACKET: ROS_ERROR("Ublox_error: packet length too big"); pos += length + UBLOX_FRAMESIZE; break;
            default: pos++; break;
            }
            continue;
        };
        processed++;
        pos += length + UBLOX_FRAMESIZE;

        ROS_INFO("Class ID: %x", header.classId);
        ROS_INFO("Message ID: %x", header.messageId);

        if (header.classId != NAV_INTERFACE_VEHICLE_CLASS){
            ROS_WARN("Class ID missmatch");
            continue;
        }
        switch (header.messageId)
        {
        case NAV_INTERFACE_COM_ID_STATUS: {
            if (length != sizeof(NAV_API_RAW_STATUS_DATA)){
                ROS_ERROR("Size-mismatch for NAV_API_RAW_STATUS_DATA: %d",length);
                continue;
            }
            ROS_INFO("STATUS Message recieved");
            NAV_API_RAW_STATUS_DATA *raw = (NAV_API_RAW_STATUS_DATA *) payload;
            static const double VOLTAGE_FILTER_T = 1.0;

            data.status.header.seq++;
            data.status.header.stamp = now;
            data.status.status = raw->AutonomousModeFlag ? monstertruck_msgs::Status::AUTONOMOUS : 0;
            data.status.voltage1 = (dt * (raw->InputVoltage1 / 1000.0) + VOLTAGE_FILTER_T * data.status.voltage1) / (dt + VOLTAGE_FILTER_T);
            data.status.voltage2 = (dt * (raw->InputVoltage2 / 1000.0) + VOLTAGE_FILTER_T * data.status.voltage2) / (dt + VOLTAGE_FILTER_T);

            CommandsProcessed = raw->CommandsProcessed;
            ErrorId           = raw->ErrorId;
            ErrorValue        = raw->ErrorValue;
            GpsSendQueueLoad  = raw->GpsSendQueueLoad;
            updatedStatus     = true;

            statusPub_.publish(data.status);

            ROS_DEBUG("Status:  %s, V1 = %.1f, V2 = %.1f",
                      (data.status.status & monstertruck_msgs::Status::AUTONOMOUS ? "AUTONOMOUS" : "MANUAL"),
                      data.status.voltage1, data.status.voltage2);
        }
            break;

        case NAV_INTERFACE_COM_ID_IMU: {
            if (length != sizeof(NAV_API_RAW_IMU_DATA)){
                ROS_ERROR("Size-mismatch for NAV_API_RAW_IMU_DATA: %d", length);
                continue;
            }
            ROS_INFO("IMU Message recieved");
            NAV_API_RAW_IMU_DATA *raw = (NAV_API_RAW_IMU_DATA *) payload;

            data.imu.header.seq++;
            data.imu.header.stamp = now;

            unsigned short *RGyro  = &(raw->RGyroX);
            unsigned short *RAccel = &(raw->RAccelX);
            data.imu.angular_velocity.x = static_cast<double>(RGyro [axis_GyroX] - 32768)  * c1_GyroX  + c0_GyroX;
            data.imu.angular_velocity.y = static_cast<double>(RGyro [axis_GyroY] - 32768)  * c1_GyroY  + c0_GyroY;
            data.imu.angular_velocity.z = static_cast<double>(RGyro [axis_GyroZ] - 32768)  * c1_GyroZ  + c0_GyroZ;
            data.imu.linear_acceleration.x = static_cast<double>(RAccel[axis_AccelX] - 32768) * c1_AccelX + c0_AccelX;
            data.imu.linear_acceleration.y = static_cast<double>(RAccel[axis_AccelY] - 32768) * c1_AccelY + c0_AccelY;
            data.imu.linear_acceleration.z = static_cast<double>(RAccel[axis_AccelZ] - 32768) * c1_AccelZ + c0_AccelZ;


            imuPub_.publish(data.imu);

            zeroCommandUpdate(data.imu);

            /*
          data.baro.Pressure = ((double)((NAV_API_RAW_IMU_DATA*) payload)->RBaro / 65536.0 + 0.152) / .01059 * 10.0;
          {
            static double zero = data.baro.Pressure;
            data.baro.HeightQNH = 0.0;
            data.baro.HeightQFE = - (data.baro.Pressure - zero) * 8.0;
          }
          */

            ROS_DEBUG("IMU:     w_x = % 5.3f, w_y = % 5.3f, w_z = % 5.3f", data.imu.angular_velocity.x * 180.0/M_PI, data.imu.angular_velocity.y * 180.0/M_PI, data.imu.angular_velocity.z * 180.0/M_PI);
            ROS_DEBUG("         a_x = % 5.3f, a_y = % 5.3f, a_z = % 5.3f", data.imu.linear_acceleration.x, data.imu.linear_acceleration.y, data.imu.linear_acceleration.z);
        }
            break;

        case NAV_INTERFACE_COM_ID_GPS_IN:
            ROS_ERROR("GPS is not supported");
            /*{

            if (bufferGPS.write(static_cast<char *>(payload), length) == 0)
            {
                // VehicleAPI::dump(VehicleAPI::INFO, "GPS", static_cast<char *>(payload), length);
                readDataFromGPS();
            }
            else
            {
                log(Warning) << "GPS Buffer overflow" << endlog();
                bufferGPS.flush();
            }
             }*/
            break;

        case NAV_INTERFACE_COM_ID_COMPASS_2D: {
            if (length != sizeof(NAV_API_RAW_COMPASS_2D_DATA)){
                ROS_ERROR("Size-mismatch for NAV_API_RAW_COMPASS_2D_DATA: %d", length);
                continue;
            }
            NAV_API_RAW_COMPASS_2D_DATA *raw = (NAV_API_RAW_COMPASS_2D_DATA *) payload;

            data.compass.header.seq++;
            data.compass.header.stamp = now;

            data.compass.x = raw->comp_x;
            data.compass.y = raw->comp_y;
            data.compass.z = 0;
            data.compass.heading = atan2(-(data.compass.y - c0_CompassY), (data.compass.x - c0_CompassX));
            data.compass.declination = 0;


            compassPub_.publish(data.compass);

            ROS_DEBUG("Compass: x = % 5.3f, y = % 5.3f, z = % 5.3f", data.compass.x, data.compass.y, data.compass.z);
            ROS_DEBUG("         heading = % 5.1f, declination = % 5.1f", data.compass.heading, data.compass.declination);
        }
            break;

        case NAV_INTERFACE_COM_ID_HODOMETER: {
            if (length != sizeof(NAV_API_RAW_HODOMETER_DATA)){
                ROS_ERROR("Size-mismatch for NAV_API_RAW_HODOMETER_DATA: %d", length);
                continue;
            }
            NAV_API_RAW_HODOMETER_DATA *raw = (NAV_API_RAW_HODOMETER_DATA *) payload;

            float sign = 1.0;

            if (isBackwards == true) sign = -1.0;

            long old[4] = { data.odometry.tics_fl, data.odometry.tics_rl, data.odometry.tics_rr, data.odometry.tics_fr };

            data.odometry.header.seq++;
            data.odometry.header.stamp = now;

            data.odometry.tics_fl = raw->hodometer_count[0];     // front left
            data.odometry.tics_rl = raw->hodometer_count[1];     // rear left
            data.odometry.tics_rr = raw->hodometer_count[2];     // rear right
            data.odometry.tics_fr = raw->hodometer_count[3];     // front right

            //VehicleAPI::print("odometry: %u %u %u %u", data.odometry.tics_fl, data.odometry.tics_rl, data.odometry.tics_rr, data.odometry.tics_fr);

            //data.odometry.s_fl = data.odometry.tics_fl / 240.0 * 0.5;
            //data.odometry.s_rl = data.odometry.tics_rl / 240.0 * 0.5;
            //data.odometry.s_rr = data.odometry.tics_rr / 240.0 * 0.5;
            //data.odometry.s_fr = data.odometry.tics_fr / 240.0 * 0.5;

            data.odometry.v_fl = sign * (data.odometry.tics_fl - old[0]) / 240.0 * 0.5 / dt;
            data.odometry.v_rl = sign * (data.odometry.tics_rl - old[1]) / 240.0 * 0.5 / dt;
            data.odometry.v_rr = sign * (data.odometry.tics_rr - old[2]) / 240.0 * 0.5 / dt;
            data.odometry.v_fr = sign * (data.odometry.tics_fr - old[3]) / 240.0 * 0.5 / dt;

            data.odometry.speed   = ( data.odometry.v_fl + data.odometry.v_rl + data.odometry.v_rr + data.odometry.v_fr) / 4;
            data.odometry.yawRate = (-data.odometry.v_fl - data.odometry.v_rl + data.odometry.v_rr + data.odometry.v_fr) / 4 / (Monstertruck::MonstertruckModel::getWheelTrack() / 2.0);

            // float v_l = std::min(data.odometry.v_fl, data.odometry.v_rl);
            // float v_r = std::min(data.odometry.v_rl, data.odometry.v_rr);
            // data.odometry.speed   = (v_r + v_l) / 2;
            // data.odometry.yawRate = (v_r - v_l) / 2 / (MonstertruckModel::getWheelTrack() / 2.0);

            odometryPub_.publish(data.odometry);

            ROS_DEBUG("Odom:    v_fl = % 5.3f    v_fr = % 5.3f", data.odometry.v_fl, data.odometry.v_fr);
            ROS_DEBUG("         v_rl = % 5.3f    v_rr = % 5.3f", data.odometry.v_rl, data.odometry.v_rr);
        }
            break;

        case NAV_INTERFACE_COM_ID_PRELOAD:
            break;

        case NAV_INTERFACE_COM_ID_SERVO_OUT:
            break;

        default:
            ROS_WARN("unknown uBlox received: classId=%x  messageId=%x length=%d" , header.classId , header.messageId , length);
            break;
        }
    }

    ROS_INFO("processed: %d", processed);
    ROS_INFO("checksum_error: %d", checksum_errors);

    if (pos == 0 && ret < 0) {
        ROS_ERROR("Error in Ublox Interpreter: %s", interfaceUBlox.strerror(ret));
        // VehicleAPI::dump(VehicleAPI::DEBUG, 0, (char *)(bufferIn + pos), bufferInLength - pos);
        return false;
    }
    return true;
}


bool HectorMonstertruckDriver::zeroCommandUpdate(sensor_msgs::Imu const& imu) {
    if (meanImu_count < 0) return false;

    // mean acceleration calculation
    meanImu.linear_acceleration.x += imu.linear_acceleration.x;
    meanImu.linear_acceleration.y += imu.linear_acceleration.y;
    meanImu.linear_acceleration.z += imu.linear_acceleration.z;
    meanImu.angular_velocity.x += imu.angular_velocity.x;
    meanImu.angular_velocity.y += imu.angular_velocity.y;
    meanImu.angular_velocity.z += imu.angular_velocity.z;

    if (++meanImu_count > 100) {
        meanImu.linear_acceleration.x /= meanImu_count;
        meanImu.linear_acceleration.y /= meanImu_count;
        meanImu.linear_acceleration.z /= meanImu_count;
        meanImu.angular_velocity.x /= meanImu_count;
        meanImu.angular_velocity.y /= meanImu_count;
        meanImu.angular_velocity.z /= meanImu_count;

        c0_AccelX = c0_AccelX - meanImu.linear_acceleration.x;
        c0_AccelY = c0_AccelY - meanImu.linear_acceleration.y;
        c0_AccelZ = c0_AccelZ - meanImu.linear_acceleration.z + 9.8065;
        c0_GyroX  = c0_GyroX  - meanImu.angular_velocity.x;
        c0_GyroY  = c0_GyroY  - meanImu.angular_velocity.y;
        c0_GyroZ  = c0_GyroZ  - meanImu.angular_velocity.z;

        meanImu_count = -1;
        meanImu = sensor_msgs::Imu();

        ROS_INFO("Finished IMU bias calibration!");
        return true;
    }

    return false;
}


bool HectorMonstertruckDriver::send(void *message, size_t size, uint8_t classId, uint8_t messageId)
{
    ROS_INFO("Sending message to interface board");
    int length = -1;

    if (sizeof(bufferOut) - bufferOutLength < size){
        ROS_WARN("Message does not fit into output buffer");
        return false;
    }
    if (classId == 0 && messageId == 0)
    {
        memcpy(bufferOut + bufferOutLength, message, size);
        length = size;
    }
    else
    {
        ROS_INFO("Encoding message...");
        length = interfaceUBlox.encode(message, size, classId, messageId, bufferOut + bufferOutLength, sizeof(bufferOut) - bufferOutLength);
        if (length < 0){
            ROS_WARN("Message encoding failed");
            return false;
        }
        ROS_INFO("...done");
    }

    bufferOutLength += length;
    return true;
}


bool HectorMonstertruckDriver::sendDataToInterfaceBoard()
{
    if (bufferOutLength == 0) return true;

    if (!device) return false;

    if (!device->send(bufferOut, bufferOutLength)) {
        ROS_ERROR("Could not send data to the Monstertruck device: %s" ,strerror(errno));
        return false;
    }
    bufferOutLength = 0;

    return true;
}


bool HectorMonstertruckDriver::configure(void *message, size_t size, uint8_t classId, uint8_t messageId)
{
    ROS_INFO("Sending configure message to interface board");
    int timeout, retry, step;

    for (retry = 3; retry > 0; --retry)
    {
        send(message, size, classId, messageId);
        if (!sendDataToInterfaceBoard()){
            ROS_WARN("Sending data to interface board failed");
            return false;
        }
        timeout = 100;
        step = 0;

        while(timeout-- > 0)
        {
            updatedStatus = false;
            if (!readDataFromInterfaceBoard()){
                ROS_INFO("configure: Waiting for successful read");
                usleep(100 * 1000);
                continue; //TODO consider wait
            }

            if (updatedStatus)
            {
                ROS_DEBUG("CommandsProcessed: %d ErrorId: %d ErrorValue: %d", CommandsProcessed, ErrorId , ErrorValue);
                if (CommandsProcessed > 0){
                    return true;
                }
            }else{
                ROS_INFO("No status update");
            }
        }

        ROS_WARN("Sending failed. (Remaining %d)", retry);
    }
    return false;
}
