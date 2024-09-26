#include <boost/asio.hpp>
#include <iostream>
#include <vector>
#include <cstring> // For memcpy
#include <cstdio>  // For printf

#ifndef SERIALHELPER_H
#define SERIALHELPER_H

#define START_BYTE 0xAA
#define REQUEST_POSITION_COMMAND 0x00
#define RESET_POSITION_COMMAND 0x01
#define SET_POSITION_COMMAND 0x02
#define SET_SPEEDS_COMMAND 0x03
#define SET_ARM_POSITION_COMMAND 0x04
#define GET_ARM_POSITION_COMMAND 0x05
#define SET_CAMERA_ANGLE_COMMAND 0x06
#define REQUEST_DISTANCE_SENSOR_COMMAND 0x07
#define POOP_PICKUP_COMMAND 0x08
#define HOME_ARM_ONLY_COMMAND 0x09
#define HOME_ALL_COMMAND 0x0A

#include <thread>
#include <chrono>

using namespace boost;
using namespace std;

enum DistSense
{
    SENSE_1, // Represents the first sensor
    SENSE_2, // Represents the second sensor
    SENSE_3, // Represents the third sensor
    SENSE_4, // Represents the fourth sensor
    SENSE_5  // Represents the fifth sensor
};

// Enum for the steppers
enum Stepper
{
    STEPPER_1, // Represents the first stepper
    STEPPER_2, // Represents the second stepper
    STEPPER_3  // Represents the third stepper
};

enum Dir
{
    CLOCKWISE,       // Represents clockwise direction
    COUNTERCLOCKWISE // Represents counterclockwise direction
};
class serialHelper
{
public:
    serialHelper(const std::string &portname, unsigned int baud_rate);
    ~serialHelper();

    void sendData(const std::vector<uint8_t> &data);
    std::vector<uint8_t> receiveData(std::size_t expectedLength);

    // Functions to send specific data
    void sendSpeeds(float m1, float m2, float m3, float m4);
    void requestPosition();
    void resetPosition();
    void requestDistanceSensorMeasurement(enum DistSense sensor);
    void poopPickup();
    void setArmPosition(enum Stepper stepper, enum Dir dir, float degrees);
    void setCameraAngle(float angle);
    void requestHomeArmOnly();
    void requestHomeAll();

    // Functions to receive specific data
    std::vector<float> receivePosition();
    float receiveDistanceSensorMeasurement(enum DistSense sensor);
    int requestAndWaitForPoopPickup();
    int requestAndWaitForArmPosition(enum Stepper stepper, enum Dir dir, float degrees);
    int requestAndWaitForHomeArmOnly();
    int requestAndWaitForHomeAll();

private:
    void floatToBytesBE(float value, uint8_t *bytes);
    float bytesToFloatBE(const uint8_t *bytes);

    std::string portname;
    unsigned int baud_rate;

    asio::io_service io;      // Create an IO service
    asio::serial_port serial; // Create a serial port object
};

#endif // SERIALHELPER_H