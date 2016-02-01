#ifndef SERIALDEVICE_H
#define SERIALDEVICE_H

#include <string>

class SerialDevice
{

private:
    int fd_com_port;
    std::string device_name;
    long baudrate;
public:
    SerialDevice(std::string device_name, long baudrate);
    virtual ~SerialDevice();

    bool configure();
    bool send(const void *message, size_t size);
    bool receive(void *message, size_t *size);
    void cleanup();
};

#endif // SERIALDEVICE_H
