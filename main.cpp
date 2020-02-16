#include "mbed.h"
#include "wifi.h"
#include <time.h> 
#include <pthread.h> 

// Sensors drivers present in the BSP library
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#include "stm32l475e_iot01_psensor.h"
#include "stm32l475e_iot01_magneto.h"
#include "stm32l475e_iot01_gyro.h"
#include "stm32l475e_iot01_accelero.h"

#define LD1_ON     {led1 = 1;}
#define LD1_OFF    {led1 = 0;}
#define LD1_TOG    {led1 = !led1;}

#define LD2_ON     {led2 = 1;}
#define LD2_OFF    {led2 = 0;}
#define LD2_TOG    {led2 = !led2;}

#define LD3_ON     {led34.output(); led34 = 1; led3_status = 1;}
#define LD3_OFF    {led34.input(); led3_status = 0;}
#define LD3_TOG    {if (led3_status) LD3_OFF else LD3_ON;}

#define LD4_ON     {led34.output(); led34 = 0; led4_status = 1;}
#define LD4_OFF    {led34.input(); led4_status = 0;}
#define LD4_TOG    {if (led4_status) LD4_OFF else LD4_ON;}

DigitalOut led1(LED1);
DigitalOut led2(LED2);
// This object drives both LD3 and LD4 on the board.
// Only one of these LEDs can be driven at a time.
DigitalInOut led34(LED3);

InterruptIn button(USER_BUTTON);

int led_nb = 0;
int led3_status = 0;
int led4_status = 0;
double led_delay = 0.1; // 100 ms

void button_pressed()
{
    if (led_nb == 4) {
        if (led_delay == 0.1) {
            led_delay = 1.0; // 1 second
        } else {
            led_delay = 0.1; // 100 ms
        }
    }
}

void button_released()
{
    LD1_OFF;
    LD2_OFF;
    LD3_OFF;
    LD4_OFF;
    led_nb++;
    if (led_nb > 4) {
        led_nb = 0;
    }
}

void simul() {
    button.fall(&button_pressed); // Change led delay
    button.rise(&button_released); // Change led

    while(1) {
        switch(led_nb) {
            case 0:
                LD1_TOG;
                break;
            case 1:
                LD2_TOG;
                break;
            case 2:
                LD3_TOG;
                break;
            case 3:
                LD4_TOG;
                break;
            case 4:
                LD1_TOG;
                LD2_TOG;
                LD3_TOG;
                break;
            default:
                break;
        }
        wait(led_delay);
    }   
}

/* Private defines -----------------------------------------------------------*/
#define WIFI_WRITE_TIMEOUT 10000
#define WIFI_READ_TIMEOUT  10000
#define PORT           80

/* Private macro -------------------------------------------------------------*/
void sensor_data_show();
void wifi_sample_run();
static void WebServerProcess(void);
/* Private variables ---------------------------------------------------------*/
Serial pc(SERIAL_TX, SERIAL_RX);
static   uint8_t http[1024];
uint16_t respLen;
uint8_t  IP_Addr[4]; 
uint8_t  MAC_Addr[6]; 
int32_t Socket = -1;
char     ModuleName[32];
DigitalOut led(LED2);
AnalogIn adc_temp(ADC_TEMP);


int main() {
    pc.baud(115200);
    pthread_t thread_id; 
    
    wifi_sample_run();
    
    BSP_TSENSOR_Init();
    BSP_HSENSOR_Init();
    BSP_PSENSOR_Init();

    BSP_MAGNETO_Init();
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    
    while(1) { 
        WebServerProcess (); 
        //pthread_create(&thread_id, NULL, WebServerProcess, NULL); 
        //pthread_join(thread_id, NULL); 
        sensor_data_show();
        exit(0);
    }
}

void sensor_data_show() {
    
    int16_t pDataXYZ[3] = {0};
    float pGyroDataXYZ[3] = {0}; 
    //float sensor_value = 0;
    
    printf("\nTEMPERATURE = %.2f degC\t\tHUMIDITY = %.2f %%\t\tPRESSURE = %.2f mBar\n", BSP_TSENSOR_ReadTemp(), BSP_HSENSOR_ReadHumidity(), BSP_PSENSOR_ReadPressure());
    wait(1);
    BSP_MAGNETO_GetXYZ(pDataXYZ);
    printf("\nMAGNETOMETER  X = %d\t\tY = %d\t\tZ = %d", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
    BSP_ACCELERO_AccGetXYZ(pDataXYZ);
    printf("\nACCELEROMETER X = %d\t\tY = %d\t\tZ = %d", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
    BSP_GYRO_GetXYZ(pGyroDataXYZ);
    printf("\nGYROSCOPE     X = %.2f\t\tY = %.2f\t\tZ = %.2f\n", pGyroDataXYZ[0], pGyroDataXYZ[1], pGyroDataXYZ[2]);    
    wait(1);
}

void wifi_sample_run() {
  
    WIFI_Init();
    printf("\n");
    printf("************************************************************\n");
    printf("***   STM32 IoT Discovery kit for STM32L475 MCU          ***\n");
    printf("***         WIFI Web Server demonstration                ***\n\n");
    printf("*** Copy the IP address on another device connected      ***\n");
    printf("*** to the wifi network                                  ***\n");
    printf("***       Please be patient and wait for IP address      ***\n");
    printf("************************************************************\n");
    WIFI_Connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, WIFI_ECN_WPA2_PSK);
    WIFI_GetIP_Address(IP_Addr);
    printf("> Go to IP Address : %d.%d.%d.%d\n", IP_Addr[0],IP_Addr[1],IP_Addr[2],IP_Addr[3]); 
}

static void WebServerProcess(void) {
  
  Socket = 0;
  uint8_t  temp[50];
  uint16_t SentDataLength;
  int16_t pAccelDataXYZ[3] = {0};
  int16_t pMagDataXYZ[3] = {0};
  float pGyroDataXYZ[3] = {0};
  
  BSP_MAGNETO_GetXYZ(pAccelDataXYZ);
  BSP_ACCELERO_AccGetXYZ(pMagDataXYZ);
  BSP_GYRO_GetXYZ(pGyroDataXYZ);

  set_time(1581861535);
  time_t seconds = time(NULL);
  
  WIFI_StartServer(Socket, WIFI_TCP_PROTOCOL, "", PORT);
  printf("> HTTP Server Started \n");

  //strcpy((char *)http, (char *)"HTTP/1.0 200 OK\r\nContent-Type: text/html\r\nPragma: no-cache\r\n\r\n");
  //strcat((char *)http, (char *)"<html>\r\n<body>\r\n");
  //strcat((char *)http, (char *)"<title>STM32 Web Server</title>\r\n");
  //strcat((char *)http, (char *)"<h2>InventekSys : Web Server using Es-Wifi with STM32</h2>\r\n");
  //strcat((char *)http, (char *)"<br /><hr>\r\n");
  //strcat((char *)http, (char *)"<p><form method=\"POST\">{ \"Temp\": %d°C", BSP_TSENSOR_ReadTemp());
  strcat((char *)http, (char *)"{  \"Temperature\" : \"");
  sprintf((char *)temp, "%.2f", BSP_TSENSOR_ReadTemp());
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\" ");
  strcat((char *)http, (char *)"<  \"Humidity\" : \"");
  sprintf((char *)temp, "%.2f", BSP_HSENSOR_ReadHumidity());
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\" ");
  strcat((char *)http, (char *)"<  \"Pressure\" : \"");
  sprintf((char *)temp, "%.2f", BSP_PSENSOR_ReadPressure());
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\" ");
  strcat((char *)http, (char *)"  \"MAGNETOMETER\" : { \"X\" : \"");
  sprintf((char *)temp, "%d", pMagDataXYZ[0]);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\", \"Y\" : \"");
  sprintf((char *)temp, "%d", pMagDataXYZ[1]);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\", \"Z\" : \"");
  sprintf((char *)temp, "%d", pMagDataXYZ[2]);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\" } ");
  strcat((char *)http, (char *)"  \"ACCELEROMETER\" : { \"X\" : \"");
  sprintf((char *)temp, "%d", pAccelDataXYZ[0]);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\", \"Y\" : \"");
  sprintf((char *)temp, "%d", pAccelDataXYZ[1]);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\", \"Z\" : \"");
  sprintf((char *)temp, "%d", pAccelDataXYZ[2]);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\" } ");
  strcat((char *)http, (char *)"  \"GYROSCOPE\" : { \"X\" : \"");
  sprintf((char *)temp, "%.2f", pGyroDataXYZ[0]);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\", \"Y\" : \"");
  sprintf((char *)temp, "%.2f", pGyroDataXYZ[1]);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\", \"Z\" : \"");
  sprintf((char *)temp, "%.2f", pGyroDataXYZ[2]);
  strcat((char *)http, (char *)temp);
  strcat((char *)http, (char *)"\" } ");
  strcat((char *)http, (char *)"  \"Time\" : \"");
  sprintf((char *)temp, "%2s\" }", ctime(&seconds));
  strcat((char *)http, (char *)temp);
  
  //strcat((char *)http, (char *)"<p><form method=\"POST\">HACK@CEWIT 2020 : Ali Asgar Tashrifwala");
  
  WIFI_SendData(0, (uint8_t *)http, strlen((char *)http), &SentDataLength, WIFI_WRITE_TIMEOUT);

  WIFI_StopServer(Socket);
  wait(1);
}
/*

{
  "name" : [{
    "id" : "f2",
    "use" : "official" ,
    "given" : [ "Karen" ],
    "family" :  "Van",
    "_family" : {"id" : "a2"}
   }],
  "text" : {
    "status" : "generated" ,
    "div" : "<div xmlns=\"http://www.w3.org/1999/xhtml\"><p>...</p></div>"
  }
} */
