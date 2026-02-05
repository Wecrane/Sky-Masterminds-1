#ifndef __ENCODER_H
#define __ENCODER_H

void Encoder_Init(void);    
void Clear_Location(void);
int16_t Encoder_Get_L(void);
int16_t Encoder_Get_R(void);
void Encoder_Read(void);
extern int Speed_L,Speed_R;
extern float Location;
#endif
