#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
    angle = 0.0f;
    bias = 0.0f;
    rate = 0.0f;

    P[0][0] = 1.0f;
    P[0][1] = 0.0f;
    P[1][0] = 0.0f;
    P[1][1] = 1.0f;

    Q_angle = 0.01f;   // Tăng lên để phản ứng nhanh hơn
    Q_bias = 0.001f;   // Giảm xuống để giảm trễ
    R_measure = 0.03f; // Giữ nguyên
}

float KalmanFilter::update(float newAngle, float newRate, float dt)
{
    rate = newRate - bias;
    angle += dt * rate;

    // Cập nhật ma trận hiệp phương sai
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Tính toán Kalman Gain
    float S = P[0][0] + R_measure;
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Điều chỉnh theo đo đạc mới
    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    // Cập nhật ma trận hiệp phương sai
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}
