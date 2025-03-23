#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include "I2c_interface.h"
#include "KalmanFilter.h"
#include <math.h>

#define PIN_SCL 20
#define PIN_SDA 21

struct MPUData
{
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
};

struct MAGData
{
    float magX, magY, magZ;
};

struct BMPData
{
    float temperature;
    float pressure;
    float altitude;
};

struct SensorData
{
    MPUData mpu;
    MAGData mag;
    BMPData bmp;
    float roll, pitch, yaw;
};

class SensorManager
{
public:
    SensorManager(); // Constructor

    void begin(bool enableMPU, bool enableMag, bool enableBMP); // Khởi động cảm biến

    void readMPU6050(MPUData &mpuData);
    void readHMC5883L(MAGData &magData);
    void readBMP180(BMPData &bmpData);

    void calculateAngles(SensorData &data); // Tính toán Roll, Pitch, Yaw
    float normalizeAngle(float angle);

    void computeOrientation(MPUData &mpuData, MAGData &magData);

private:
    Adafruit_MPU6050 mpu;
    Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
    Adafruit_BMP085 bmp;

    KalmanFilter kalmanX, kalmanY, kalmanZ;
    float dt, lastTime;

    // Offset để hiệu chỉnh MPU6050
    float accX_offset, accY_offset, accZ_offset;
    float gyroX_offset, gyroY_offset, gyroZ_offset;

    void enable_HMC5883L_via_MPU6050();
    void initMPU6050();
    void initHMC5883L();
    void initBMP180();
    void calibrateMPU6050(); // Hiệu chỉnh MPU6050
};

#endif // SENSOR_MANAGER_H
