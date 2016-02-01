#include "hector_monstertruck_driver/SerialDevice.h"

#include <ros/ros.h>

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <termios.h>

SerialDevice::SerialDevice(std::string device_name, long baudrate) : device_name(device_name), baudrate(baudrate), fd_com_port(-1) {

}

SerialDevice::~SerialDevice() {
    cleanup();
}

bool SerialDevice::configure()
{
    struct termios terminal;
    fd_com_port = open(device_name.c_str(), O_RDWR | O_NDELAY | O_NONBLOCK);
    if (fd_com_port == -1)
    {
        ROS_ERROR("Can't setup com port: %s", device_name.c_str());
        return false;
    }


    int baud = -1;
    switch(baudrate){
    case 9600: baud = B9600; break;
    case 115200: baud = B115200; break;
    }
    if (baud == -1)
    {
        ROS_ERROR("No valid baud rate: %ld", baudrate);
        cleanup();
        return false;
    }

    fcntl(fd_com_port, F_SETFL,FNDELAY);
    terminal.c_cflag = baud | CS8 | CLOCAL | CREAD;
    terminal.c_iflag = IGNPAR;
    terminal.c_iflag &= ~(ICRNL | INLCR); //Disable new line conversion
    terminal.c_oflag = 0;
    terminal.c_lflag &=~(ICANON);
    tcflush(fd_com_port,TCIOFLUSH);
    tcsetattr(fd_com_port,TCSANOW,&terminal);
    return true;
}

bool SerialDevice::send(const void *message, size_t size)
{
    write(fd_com_port,message,size);
    return true; }

bool SerialDevice::receive(void *message, size_t *size)
{
    /*Variablen für select*/
    fd_set rfds;
    struct timeval tv;
    //tcflush(fd_com_port,TCIFLUSH); //Flush input buffer
    FD_ZERO(&rfds);
    FD_SET(fd_com_port,&rfds);
    tv.tv_sec = 0;
    tv.tv_usec = 100;
    if (select(fd_com_port+1,&rfds,NULL,NULL,&tv)) {
        *size= read(fd_com_port,message, *size);
        return true;
    }
    return false;
}
void SerialDevice::cleanup()
{
    if(fd_com_port != -1){
        close(fd_com_port);
    }
}
