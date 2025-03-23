#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

class KalmanFilter
{
public:
    KalmanFilter();
    float update(float newAngle, float newRate, float dt);

private:
    float angle; // Góc ước lượng
    float bias;  // Sai số của con quay
    float rate;  // Tốc độ góc

    float P[2][2]; // Ma trận hiệp phương sai lỗi
    float Q_angle;
    float Q_bias;
    float R_measure;
};

#endif
