#include "serialHelper.h"

serialHelper::serialHelper(const std::string &portname, unsigned int baud_rate) : serial(io) // Initialize serial with the io_service
{
    this->portname = portname;
    this->baud_rate = baud_rate;

    try
    {
        // Open the specified serial port
        this->serial.open(this->portname);

        // Set the baud rate
        this->serial.set_option(asio::serial_port_base::baud_rate(this->baud_rate));

        // Set character size to 8 bits
        this->serial.set_option(asio::serial_port_base::character_size(8));

        // Disable parity
        this->serial.set_option(asio::serial_port_base::parity(asio::serial_port_base::parity::none));

        // Set one stop bit
        this->serial.set_option(asio::serial_port_base::stop_bits(asio::serial_port_base::stop_bits::one));

        // Disable flow control
        this->serial.set_option(asio::serial_port_base::flow_control(asio::serial_port_base::flow_control::none));
    }
    catch (const std::exception &e)
    {
        cerr << "Error configuring serial port: "
             << e.what() << endl;
    }
}

serialHelper::~serialHelper()
{
    // Close the serial port
    this->serial.close();
}

void serialHelper::sendSpeeds(float m1, float m2, float m3, float m4)
{
    // Convert the floats to bytes
    uint8_t m1_bytes[4], m2_bytes[4], m3_bytes[4], m4_bytes[4];
    floatToBytesBE(m1, m1_bytes);
    floatToBytesBE(m2, m2_bytes);
    floatToBytesBE(m3, m3_bytes);
    floatToBytesBE(m4, m4_bytes);

    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x11); // Length byte
    data_packet.push_back(SET_SPEEDS_COMMAND);
    data_packet.insert(data_packet.end(), m1_bytes, m1_bytes + 4);
    data_packet.insert(data_packet.end(), m2_bytes, m2_bytes + 4);
    data_packet.insert(data_packet.end(), m3_bytes, m3_bytes + 4);
    data_packet.insert(data_packet.end(), m4_bytes, m4_bytes + 4);

    // Send the data packet
    sendData(data_packet);
}

void serialHelper::requestPosition()
{
    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x01);                     // Length byte
    data_packet.push_back(REQUEST_POSITION_COMMAND); // Request position command

    // Send the data packet
    sendData(data_packet);
}

void serialHelper::resetPosition()
{
    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x01);                   // Length byte
    data_packet.push_back(RESET_POSITION_COMMAND); // Reset position command

    // Send the data packet
    sendData(data_packet);
}

void serialHelper::requestDistanceSensorMeasurement(enum DistSense sensor)
{
    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x02);                            // Length byte
    data_packet.push_back(REQUEST_DISTANCE_SENSOR_COMMAND); // Request distance sensor measurement command
    data_packet.push_back(sensor);

    // Send the data packet
    sendData(data_packet);
}

void serialHelper::poopPickup()
{
    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x01);                // Length byte
    data_packet.push_back(POOP_PICKUP_COMMAND); // Poop pickup command

    // Send the data packet
    sendData(data_packet);

    // Print a message to the console
    cout << "Poop pickup requested" << endl;
}

void serialHelper::setArmPosition(enum Stepper stepper, enum Dir dir, float degrees)
{
    // Convert the float to bytes
    uint8_t degrees_bytes[4];
    floatToBytesBE(degrees, degrees_bytes);

    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x07);                     // Length byte
    data_packet.push_back(SET_ARM_POSITION_COMMAND); // Set arm position command
    data_packet.push_back(stepper);
    data_packet.push_back(dir);
    data_packet.insert(data_packet.end(), degrees_bytes, degrees_bytes + 4);

    // Send the data packet
    sendData(data_packet);

    // Print a message to the console
    cout << "Arm position change on stepper " << stepper << " requested, with direction " << dir << " and degrees " << degrees << endl;
}

void serialHelper::setCameraAngle(float angle)
{
    // Convert the float to bytes
    uint8_t angle_bytes[4];
    floatToBytesBE(angle, angle_bytes);

    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x05);                     // Length byte
    data_packet.push_back(SET_CAMERA_ANGLE_COMMAND); // Set camera angle command
    data_packet.insert(data_packet.end(), angle_bytes, angle_bytes + 4);

    // Send the data packet
    sendData(data_packet);

    // Print a message to the console
    cout << "Camera angle change requested to " << angle << endl;
}

void serialHelper::requestHomeArmOnly()
{
    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x01);                  // Length byte
    data_packet.push_back(HOME_ARM_ONLY_COMMAND); // Home arm only command

    // Send the data packet
    sendData(data_packet);
}

void serialHelper::requestHomeAll()
{
    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x01);             // Length byte
    data_packet.push_back(HOME_ALL_COMMAND); // Home all command

    // Send the data packet
    sendData(data_packet);
}

void serialHelper::requestPickupLift()
{
    // Build the data packet
    vector<uint8_t> data_packet;
    data_packet.push_back(START_BYTE);
    data_packet.push_back(0x01);                // Length byte
    data_packet.push_back(PICKUP_LIFT_COMMAND); // Poop lift command

    // Send the data packet
    sendData(data_packet);

    // Print a message to the console
    cout << "Poop lift requested" << endl;
}

void serialHelper::sendData(const std::vector<uint8_t> &data)
{
    try
    {
        asio::write(this->serial, asio::buffer(data));
        // cout << "Data packet sent: ";
        // for (size_t i = 0; i < data.size(); ++i)
        // {
        //     printf("%02X ", data[i]);
        // }
        // cout << endl;
    }
    catch (const std::exception &e)
    {
        cerr << "Error writing to serial port: "
             << e.what() << endl;
        this->serial.close();
    }
}

std::vector<uint8_t> serialHelper::receiveData(std::size_t expectedLength)
{
    vector<uint8_t> data;
    data.resize(expectedLength);

    try
    {
        asio::read(this->serial, asio::buffer(data));
        // cout << "Data packet received: ";
        // for (size_t i = 0; i < data.size(); ++i)
        // {
        //     printf("%02X ", data[i]);
        // }
        // cout << endl;
    }
    catch (const std::exception &e)
    {
        cerr << "Error reading from serial port: "
             << e.what() << endl;
        this->serial.close();
    }

    return data;
}

std::vector<float> serialHelper::receivePosition()
{
    // Request the position data
    requestPosition();
    // Delay for 100ms to allow the data to be received

    // Receive the data packet
    vector<uint8_t> data = receiveData(14);

    // Extract the position data
    vector<float> position;
    position.push_back(bytesToFloatBE(&data[2]));
    position.push_back(bytesToFloatBE(&data[6]));
    position.push_back(bytesToFloatBE(&data[10]));

    return position;
}

float serialHelper::receiveDistanceSensorMeasurement(enum DistSense sensor)
{
    // Request the distance sensor measurement
    requestDistanceSensorMeasurement(sensor);

    // Receive the data packet
    vector<uint8_t> data = receiveData(6);

    // Extract the distance sensor measurement
    return bytesToFloatBE(&data[2]);
}

// A successful poop pickup will return 0
int serialHelper::requestAndWaitForPoopPickup()
{
    // Request the poop pickup
    poopPickup();

    // Receive the data packet
    vector<uint8_t> data = receiveData(2);

    // Check dat data[1] = 99, 99 is the success code
    if (data[1] == 99)
    {
        cout << "Poop pickup initate success" << endl;
        return 0;
    }

    cout << "Poop pickup failed" << endl;
    return -1;
}

// A successful arm position request will return 0
int serialHelper::requestAndWaitForArmPosition(enum Stepper stepper, enum Dir dir, float degrees)
{
    // Request the arm position
    setArmPosition(stepper, dir, degrees);

    // Receive the data packet
    vector<uint8_t> data = receiveData(2);

    // Check dat data[1] = 90, 90 is the success code
    if (data[1] == 90)
    {
        cout << "Arm position set successfully" << endl;
        return 0;
    }

    cout << "Arm position set failed" << endl;
    return -1;
}

int serialHelper::requestAndWaitForHomeArmOnly()
{
    // Request the home arm only
    requestHomeArmOnly();

    // Receive the data packet
    vector<uint8_t> data = receiveData(2);

    // Check dat data[1] = 90, 90 is the success code
    if (data[1] == 90)
    {
        cout << "Arm homed successfully" << endl;
        return 0;
    }

    cout << "Arm home failed" << endl;
    return -1;
}

int serialHelper::requestAndWaitForHomeAll()
{
    // Request the home all
    requestHomeAll();

    // Receive the data packet
    vector<uint8_t> data = receiveData(2);

    // Check dat data[1] = 90, 90 is the success code
    if (data[1] == 90)
    {
        cout << "All homed successfully" << endl;
        return 0;
    }

    cout << "All home failed" << endl;
    return -1;
}

int serialHelper::requestAndWaitForPickupLift()
{
    // Request the poop lift
    requestPickupLift();

    // Receive the data packet
    vector<uint8_t> data = receiveData(2);

    // Check dat data[1] = 99, 99 is the success code
    if (data[1] == 99)
    {
        cout << "Poop lift success" << endl;
        return 0;
    }

    cout << "Poop lift failed" << endl;
    return -1;
}

void serialHelper::floatToBytesBE(float value, uint8_t *bytes)
{
    uint32_t asInt;
    memcpy(&asInt, &value, sizeof(float));

    // Ensure big-endian byte order (MSB first)
    bytes[0] = (asInt >> 24) & 0xFF;
    bytes[1] = (asInt >> 16) & 0xFF;
    bytes[2] = (asInt >> 8) & 0xFF;
    bytes[3] = asInt & 0xFF;
}

float serialHelper::bytesToFloatBE(const uint8_t *bytes)
{
    uint32_t asInt = ((uint32_t)bytes[0] << 24) |
                     ((uint32_t)bytes[1] << 16) |
                     ((uint32_t)bytes[2] << 8) |
                     (uint32_t)bytes[3];

    float result;
    std::memcpy(&result, &asInt, sizeof(float));
    return result;
}