#ifndef __MOS_PAYLOUD_H
#define __MOS_PAYLOUD_H
#endif

typedef struct params
{
    float temperature;
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