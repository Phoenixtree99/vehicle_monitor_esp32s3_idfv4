#include "mos_payload.h"
#include "nmea0183_parse.h"
#include <string.h>

extern GNRMC GNRMC_Info;
extern BDGSV BDGSV_Info;
extern GPGSV GPGSV_Info;

extern int temperature;
extern int humidity;

float TurnToStdCoord(float coord)
{
    int integer = coord / 100;
    float decimal1 = (int)coord % 100;
    float decimal2 = (int)((decimal1 + coord - (int)coord) * 100000);
    float decimal3 = (float)decimal2 / 60;
    float result = (float)integer + decimal3 * 0.00001;
    return result;
}

void Mos_Send_Payload_Object_Init(Mos_Send_Payload_Object_t Send_Payload_Object)
{
    strcpy(Send_Payload_Object->method, "report");
    strcpy(Send_Payload_Object->ClientID , "Truck001");
    Send_Payload_Object->env_params.temperature = 0;
    Send_Payload_Object->env_params.humidity = 0;
    Send_Payload_Object->env_params.beidounum = 0;
    Send_Payload_Object->env_params.gpsnum = 0;
    Send_Payload_Object->env_params.lon = 0.0;
    Send_Payload_Object->env_params.lat = 0.0;
    Send_Payload_Object->env_params.speed = 0;
}

void Mos_Send_Payload_Object_Reflesh(Mos_Send_Payload_Object_t Send_Payload_Object)
{
    Send_Payload_Object->env_params.temperature = temperature;
    Send_Payload_Object->env_params.humidity = humidity;
    Send_Payload_Object->env_params.beidounum = BDGSV_Info.BD_SAT_Number;
    Send_Payload_Object->env_params.gpsnum = GPGSV_Info.GPS_SAT_Number;
    Send_Payload_Object->env_params.lon = GNRMC_Info.Longi;
    Send_Payload_Object->env_params.lat = GNRMC_Info.Lati;
    Send_Payload_Object->env_params.speed = GNRMC_Info.SpeedKont;
}