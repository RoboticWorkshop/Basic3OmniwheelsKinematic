#ifndef I2CINTERFACE_H
#define I2CINTERFACE_H

#define KIRI      11  //alamat slave kontroler motor kiri
#define TENGAH    12  //alamat slave kontroler motor tengah
#define KANAN     13  //alamat slave kontroler motor kanan
#define DRIBBLER  14  //alamat slave kontroler dribbler
#define KICKER    15  //alamat slave kontroler motor kicker
#define EXT_ENC_1 16  //alamat slave kontroler external encoder 1
#define EXT_ENC_2 17  //alamat slave kontroler external encoder 2

extern volatile float imu_x, imu_y, imu_z, imu_ref;
extern volatile int enc[2];

void init_slave_i2c();
void init_bno055();
void read_bno055();
void set_slave(char part, unsigned char addr);
void transmit_motor(int lpwm, int mpwm, int rpwm);
void transmit_dribbler(int lpwm, int rpwm);
void read_encoder();
void reset_encoder();
void encoder_enable();
void encoder_disable();

#endif
