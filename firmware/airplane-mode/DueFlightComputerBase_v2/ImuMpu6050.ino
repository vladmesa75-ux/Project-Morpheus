/*
  ImuMpu6050.ino

  This tab reads the MPU-6050 and calculates roll, pitch, and yaw.

  Roll and pitch are corrected using the accelerometer.
  Yaw is only gyro integration, so it can still drift over time.
*/

bool setupImu() {
  if (!mpu.begin(MPU6050_ADDRESS, &Wire)) {
    return false;
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  Wire.setClock(I2C0_SPEED_HZ);

  return true;
}

bool calibrateGyroBias() {
  /*
    Measures the gyro output while the aircraft is not moving.

    The gyro often reports a tiny rotation even when it is still.
    That tiny false value creates yaw drift.
    This function measures that false value and subtracts it later.

    This runs only at startup, not during the flight loop.
  */

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;

  for (uint16_t i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    if (!mpu.getEvent(&accel, &gyro, &temp)) {
      return false;
    }

    sumX += gyro.gyro.x;
    sumY += gyro.gyro.y;
    sumZ += gyro.gyro.z;

    delay(2);
  }

  gyroXBiasRad = sumX / GYRO_CALIBRATION_SAMPLES;
  gyroYBiasRad = sumY / GYRO_CALIBRATION_SAMPLES;
  gyroZBiasRad = sumZ / GYRO_CALIBRATION_SAMPLES;

  gyroZBiasRad += GYRO_Z_BIAS_TRIM_DEG_PER_SEC / RAD_TO_DEG_FLOAT;

  return true;
}

bool readImu() {
  /*
    Reads raw acceleration and gyro data.

    The Adafruit library gives:
    - acceleration in m/s^2
    - gyro in rad/s

    We subtract the gyro bias before saving the gyro values.
  */

  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;

  bool ok = mpu.getEvent(&accel, &gyro, &temp);

  if (!ok) {
    return false;
  }

  imu.ax = accel.acceleration.x;
  imu.ay = accel.acceleration.y;
  imu.az = accel.acceleration.z;

  imu.gx = gyro.gyro.x - gyroXBiasRad;
  imu.gy = gyro.gyro.y - gyroYBiasRad;
  imu.gz = gyro.gyro.z - gyroZBiasRad;

  imu.tempC = temp.temperature;

  updateAttitudeFromImu();

  return true;
}

void updateAttitudeFromImu() {
  /*
    Calculates roll, pitch, and yaw.

    Roll and pitch:
    - gyro gives smooth short-term motion
    - accelerometer gives long-term correction using gravity
    - complementary filter combines them

    Yaw:
    - only integrates the Z gyro
    - this means yaw is relative and can drift
  */

  uint32_t now = micros();
  float dt = (now - lastImuUpdateUs) * 0.000001f;
  lastImuUpdateUs = now;

  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.001f;
  }

  float rollAcc = atan2(imu.ay, imu.az) * RAD_TO_DEG_FLOAT;
  float pitchAcc = atan2(-imu.ax, sqrt((imu.ay * imu.ay) + (imu.az * imu.az))) * RAD_TO_DEG_FLOAT;

  float gyroRollRateDeg = imu.gx * RAD_TO_DEG_FLOAT;
  float gyroPitchRateDeg = imu.gy * RAD_TO_DEG_FLOAT;
  float gyroYawRateDeg = imu.gz * RAD_TO_DEG_FLOAT;

  if (gyroYawRateDeg > -YAW_RATE_DEADBAND_DEG_PER_SEC &&
      gyroYawRateDeg < YAW_RATE_DEADBAND_DEG_PER_SEC) {
    gyroYawRateDeg = 0.0f;
  }

  if (!attitudeInitialized) {
    attitude.rollDeg = rollAcc;
    attitude.pitchDeg = pitchAcc;
    attitude.yawDeg = 0.0f;
    attitudeInitialized = true;
    return;
  }

  attitude.rollDeg =
    (ATTITUDE_ALPHA * (attitude.rollDeg + gyroRollRateDeg * dt)) +
    ((1.0f - ATTITUDE_ALPHA) * rollAcc);

  attitude.pitchDeg =
    (ATTITUDE_ALPHA * (attitude.pitchDeg + gyroPitchRateDeg * dt)) +
    ((1.0f - ATTITUDE_ALPHA) * pitchAcc);

  attitude.yawDeg += gyroYawRateDeg * dt;

  wrapAngle180(attitude.rollDeg);
  wrapAngle180(attitude.pitchDeg);
  wrapAngle180(attitude.yawDeg);
}

void wrapAngle180(float &angle) {
  if (angle > 180.0f) {
    angle -= 360.0f;
  } else if (angle < -180.0f) {
    angle += 360.0f;
  }
}