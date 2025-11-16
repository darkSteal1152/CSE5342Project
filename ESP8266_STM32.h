#ifndef ESP8266_STM32_H
#define ESP8266_STM32_H

#include "main.h"
#include <string.h>
#include <stdio.h>

/* ------------ USER CONFIG ------------- */
#define ESP_UART       huart4   // UART connected to ESP8266

// Enable/Disable logs
#define ENABLE_USER_LOG   1
#define ENABLE_DEBUG_LOG  1  // Changed to 1 for debugging
/* -------------------------------------- */

extern UART_HandleTypeDef ESP_UART;

/* ------------ LOG MACROS ------------- */
#if ENABLE_USER_LOG
  #define USER_LOG(fmt, ...) printf("[USER] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define USER_LOG(fmt, ...)
#endif

#if ENABLE_DEBUG_LOG
  #define DEBUG_LOG(fmt, ...) printf("[DEBUG] " fmt "\r\n", ##__VA_ARGS__)
#else
  #define DEBUG_LOG(fmt, ...)
#endif
/* ------------------------------------- */

/* ------------ ENUMS & STATES --------- */
typedef enum {
    ESP8266_OK = 0,
    ESP8266_ERROR,
    ESP8266_TIMEOUT,
    ESP8266_NO_RESPONSE
} ESP8266_Status;

typedef enum {
    ESP8266_DISCONNECTED = 0,
    ESP8266_NOT_CONNECTED,
    ESP8266_CONNECTED_NO_IP,
    ESP8266_CONNECTED_IP
} ESP8266_ConnectionState;

extern ESP8266_ConnectionState ESP_ConnState;
/* ------------------------------------- */

/* ------------ API FUNCTIONS ---------- */
ESP8266_Status ESP_Init(void);
ESP8266_Status ESP_ConnectWiFi(const char *ssid, const char *password, char *ip_buffer, uint16_t buffer_len);
ESP8266_ConnectionState ESP_GetConnectionState(void);
ESP8266_Status ESP_StartWebServer(uint16_t port);
ESP8266_Status ESP_CheckForClient(char *request_buffer, uint16_t buffer_len);
ESP8266_Status ESP_SendWebPage(const char *html_content);
ESP8266_Status ESP_CloseConnection(void);
/* ------------------------------------- */

#endif // ESP8266_STM32_H
