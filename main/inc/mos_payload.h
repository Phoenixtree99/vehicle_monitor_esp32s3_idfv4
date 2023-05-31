#ifndef __MOS_PAYLOUD_H
#define __MOS_PAYLOUD_H
#endif

typedef struct params
{
    int temperature;
    int humidity;
    int beidounum;
    int gpsnum;
    float lon;
    float lat;
    int speed;
} Params;

typedef struct mos_send_payload_object
{
    char method[10];
    char ClientID[32];
    Params env_params;
} Mos_Send_Payload_Object, *Mos_Send_Payload_Object_t;

float TurnToStdCoord(float coord);

void Mos_Send_Payload_Object_Init(Mos_Send_Payload_Object_t Send_Payload_Object);

void Mos_Send_Payload_Object_Reflesh(Mos_Send_Payload_Object_t Send_Payload_Object);