/*
 * IncFile1.h
 *
 * Created: 6/26/2014 7:50:48 PM
 *  Author: jjfish
 */ 


#ifndef INCFILE1_H_
#define INCFILE1_H_

//void get_position (void);
float compute_pitch(float alpha, int16_t y_accl, int16_t z_accl, int16_t x_gyro);
void gyro_init(void);
void gyro_position(int16_t *x_gyro, int16_t *y_gyro, int16_t *z_gyro);
void accl_init(void);
void accl_position(int16_t *x_accl, int16_t *y_accl, int16_t *z_accl);

#endif /* INCFILE1_H_ */