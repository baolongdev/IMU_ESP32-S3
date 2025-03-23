#include "SensorManager.h"
#include "Arduino.h"

// Constructor
SensorManager::SensorManager() : lastTime(0) {}

void SensorManager::begin(bool enableMPU, bool enableMag, bool enableBMP)
{
    Serial.begin(115200);
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000); // Tăng tốc độ giao tiếp I2C

    if (enableMPU)
        initMPU6050();
    if (enableMag)
        initHMC5883L();
    if (enableBMP)
        initBMP180();
}

// Khởi tạo MPU6050
void SensorManager::initMPU6050()
{
    if (!mpu.begin())
    {
        Serial.println("Lỗi: Không tìm thấy MPU6050!");
        while (1)
            ;
    }
    Serial.println("MPU6050 OK!");

    // Cân chỉnh MPU6050 để loại bỏ offset
    calibrateMPU6050();
}

// Hiệu chỉnh MPU6050 (Loại bỏ offset)
void SensorManager::calibrateMPU6050()
{
    Serial.println("Đang hiệu chỉnh MPU6050...");
    float sumAccX = 0, sumAccY = 0, sumAccZ = 0;
    float sumGyroX = 0, sumGyroY = 0, sumGyroZ = 0;
    int samples = 1000;

    for (int i = 0; i < samples; i++)
    {
        MPUData tempData;
        readMPU6050(tempData);
        sumAccX += tempData.accX;
        sumAccY += tempData.accY;
        sumAccZ += tempData.accZ - 9.81; // Bù trọng lực

        sumGyroX += tempData.gyroX;
        sumGyroY += tempData.gyroY;
        sumGyroZ += tempData.gyroZ;
        delay(2);
    }

    accX_offset = sumAccX / samples;
    accY_offset = sumAccY / samples;
    accZ_offset = sumAccZ / samples;
    gyroX_offset = sumGyroX / samples;
    gyroY_offset = sumGyroY / samples;
    gyroZ_offset = sumGyroZ / samples;
    Serial.println("Hiệu chỉnh MPU6050 xong!");
}

// Khởi tạo HMC5883L
void SensorManager::initHMC5883L()
{
    enable_HMC5883L_via_MPU6050(); // Bật chế độ Bypass trên MPU6050
    delay(100);
    if (!mag.begin())
    {
        Serial.println("Lỗi: Không tìm thấy HMC5883L!");
        while (1)
            ;
    }
    Serial.println("HMC5883L OK!");
}

// Khởi tạo BMP180
void SensorManager::initBMP180()
{
    if (!bmp.begin())
    {
        Serial.println("Lỗi: Không tìm thấy BMP180!");
        while (1)
            ;
    }
    Serial.println("BMP180 OK!");
}

// Đọc dữ liệu từ MPU6050
void SensorManager::readMPU6050(MPUData &mpuData)
{
    sensors_event_t a, g, temp;
    mpu.getAccelerometerSensor()->getEvent(&a);
    mpu.getGyroSensor()->getEvent(&g);

    // Trừ đi offset
    mpuData.accX = a.acceleration.x - accX_offset;
    mpuData.accY = a.acceleration.y - accY_offset;
    mpuData.accZ = a.acceleration.z - accZ_offset;

    mpuData.gyroX = g.gyro.x - gyroX_offset;
    mpuData.gyroY = g.gyro.y - gyroY_offset;
    mpuData.gyroZ = g.gyro.z - gyroZ_offset;
}

// Đọc dữ liệu từ HMC5883L
void SensorManager::readHMC5883L(MAGData &magData)
{
    sensors_event_t event;
    mag.getEvent(&event);

    magData.magX = event.magnetic.x;
    magData.magY = event.magnetic.y;
    magData.magZ = event.magnetic.z;
}

// Tính toán Roll, Pitch từ MPU6050 và Yaw từ HMC5883L
void SensorManager::calculateAngles(SensorData &data)
{
    // Lấy dữ liệu từ cảm biến
    readMPU6050(data.mpu);
    readHMC5883L(data.mag);

    // Tính Roll và Pitch từ gia tốc kế
    float rollAcc = atan2(data.mpu.accY, data.mpu.accZ) * 180.0 / M_PI;
    float pitchAcc = atan2(-data.mpu.accX, sqrt(data.mpu.accY * data.mpu.accY + data.mpu.accZ * data.mpu.accZ)) * 180.0 / M_PI;

    // Tính thời gian giữa các lần đo
    unsigned long currentMicros = micros();
    float dt = (currentMicros - lastTime) / 1e6;
    lastTime = currentMicros;

    // Lọc Kalman cho Roll và Pitch
    data.roll = kalmanX.update(rollAcc, data.mpu.gyroX, dt);
    data.pitch = kalmanY.update(pitchAcc, data.mpu.gyroY, dt);

    // Bù nghiêng cho từ kế (Hiệu chỉnh Roll, Pitch trước khi tính Yaw)
    float cosPitch = cos(data.pitch * M_PI / 180.0);
    float sinPitch = sin(data.pitch * M_PI / 180.0);
    float cosRoll = cos(data.roll * M_PI / 180.0);
    float sinRoll = sin(data.roll * M_PI / 180.0);

    float magX = data.mag.magX * cosPitch + data.mag.magZ * sinPitch;
    float magY = data.mag.magX * sinRoll * sinPitch + data.mag.magY * cosRoll - data.mag.magZ * sinRoll * cosPitch;

    // Tính Yaw từ Magnetometer
    float yawMag = atan2(-magY, magX) * 180.0 / M_PI;

    // Lọc Kalman cho Yaw
    data.yaw = kalmanZ.update(yawMag, data.mpu.gyroZ, dt);
    if (data.yaw < 0)
        data.yaw += 360.0;
}

// Kích hoạt HMC5883L thông qua MPU6050
void SensorManager::enable_HMC5883L_via_MPU6050()
{
    sensor_write_reg_single(0x68, 0x37, 0x02); // Bật I2C Bypass mode
    sensor_write_reg_single(0x68, 0x6A, 0x00); // Tắt Master I2C Mode
}
