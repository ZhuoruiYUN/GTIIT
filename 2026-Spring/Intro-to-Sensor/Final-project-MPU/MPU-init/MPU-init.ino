#include <Wire.h>

#define MPU_ADDR       0x68
#define REG_PWR_MGMT_1 0x6B
#define REG_CONFIG     0x1A
#define REG_SMPLRT_DIV 0x19
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B

int16_t ax, ay, az, gx, gy, gz;

void write_mpu(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void read_mpu() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // temperature
  gx = (Wire.read() << 8) | Wire.read();
  gy = (Wire.read() << 8) | Wire.read();
  gz = (Wire.read() << 8) | Wire.read();
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  write_mpu(REG_PWR_MGMT_1, 0x00); // 唤醒
  write_mpu(REG_CONFIG,     0x03); // DLPF 33Hz
  write_mpu(REG_SMPLRT_DIV, 0x01); // 500Hz
  write_mpu(REG_GYRO_CONFIG,  0x00); // ±250 dps
  write_mpu(REG_ACCEL_CONFIG, 0x00); // ±2g
}

void loop() {
  read_mpu();
  Serial.print(ax); Serial.print(',');
  Serial.print(ay); Serial.print(',');
  Serial.print(az); Serial.print(',');
  Serial.print(gx); Serial.print(',');
  Serial.print(gy); Serial.print(',');
  Serial.println(gz);
}