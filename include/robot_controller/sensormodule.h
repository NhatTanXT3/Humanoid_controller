#ifndef SENSORMODULE_H
#define SENSORMODULE_H

#define _SERIAL_PORT_SENSOR "/dev/ttyUSB2"
#define _SERIAL_BUFF_SIZE    100
//#define _BAUDRATE   B1200
//#define _BAUDRATE   B2400
//#define _BAUDRATE    B4800
//#define _BAUDRATE    B9600
//#define _BAUDRATE    B57600
#define _BAUDRATE    B115200
//#define _BAUDRATE    B230400
//#define _BAUDRATE    B500000
//#define _BAUDRATE    B921600
//#define _BAUDRATE    B1500000
#define PC2MCU_TERMINATOR_  0xFE
#define PC2MCU_HEADER_      0xFF

#define PC_SENSOR_READ_RAW 0xAA
#define PC_SENSOR_READ_PROCESSED 0xDD


#define PC_SENSOR_READ_MODE_1 0xAA
#define PC_SENSOR_READ_MODE_2 0xBB
#define PC_SENSOR_READ_MODE_3 0xA1

#define SENDING_PASSIVE_LEFT_ARM_   0xB1
#define SENDING_PASSIVE_RIGHT_ARM_  0xB2

#define SENDING_MODE_CONTROL_HEAD   0xB5
#define SENDING_MODE_RESET_HEAD     0xB6

class SensorModule
{
private:

    int InitSerial(const char *Serial_Port);
    unsigned char Trans_chr[_SERIAL_BUFF_SIZE];
    unsigned char Recev_chr[_SERIAL_BUFF_SIZE];
    unsigned char Store_chr[_SERIAL_BUFF_SIZE];

    unsigned char flagDataReceived:1;
    unsigned char flagEnableCapute:1;

    unsigned char dataIndex;
    unsigned char NumofSensor;
public:
    SensorModule();

    unsigned char DataReceived_readMode;
    unsigned char flagDataReceived_readRaw:1;
    unsigned char flagDataReceived_readProcessed:1;

    unsigned char flagSetPassive:1;
    unsigned char setPassive_IDarm;

    unsigned char flagResetHead:1;
    unsigned char flagDataReceived_readRaw_Head:1;
     void Send_Serial_String(int Serial, unsigned char *Trans_chr, int Size);
    int Serial;
    int16_t sensorData[24];
    unsigned char sensorDataAvail[24];

    int getRawData();
    int getFilteredData();
    void Recev_Data_hanlder();

};

#endif // SENSORMODULE_H
