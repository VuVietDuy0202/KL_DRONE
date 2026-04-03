#define ICM20602_ADDR    0x68

#define WHO_AM_I         0x75
#define PWR_MGMT_1       0x6B
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D   // ICM-20602 có thêm register này
#define INT_ENABLE       0x38
#define INT_STATUS       0x3A
#define DATA_REG         0x3B   // ACCEL_XOUT_H