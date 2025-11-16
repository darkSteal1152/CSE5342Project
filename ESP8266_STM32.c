#include "ESP8266_STM32.h"

ESP8266_ConnectionState ESP_ConnState = ESP8266_DISCONNECTED;   // Default state
static char esp_rx_buffer[2048];
static ESP8266_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len);
static ESP8266_Status ESP_SendCommand(const char *cmd, const char *ack, uint32_t timeout);
static ESP8266_Status ESP_SendCommandWiFi(const char *cmd, uint32_t timeout);

ESP8266_Status ESP_Init(void)
{
    ESP8266_Status res;
    USER_LOG("Initializing ESP8266...");
    HAL_Delay(1000);

    res = ESP_SendCommand("AT\r\n", "OK", 2000);
    if (res != ESP8266_OK){
        DEBUG_LOG("ESP8266 Not Responding...");
        return res;
    }

    res = ESP_SendCommand("ATE0\r\n", "OK", 2000); // Disable echo
    if (res != ESP8266_OK){
        DEBUG_LOG("Disable echo Command Failed...");
        return res;
    }
    USER_LOG("ESP8266 Initialized Successfully...");
    return ESP8266_OK;
}

ESP8266_Status ESP_ConnectWiFi(const char *ssid, const char *password, char *ip_buffer, uint16_t buffer_len)
{
    USER_LOG("Setting in Station Mode");
    char cmd[128];
    snprintf(cmd, sizeof(cmd), "AT+CWMODE=1\r\n");

    ESP8266_Status result = ESP_SendCommand(cmd, "OK", 2000);
    if (result != ESP8266_OK)
    {
        USER_LOG("Station Mode Failed.");
        return ESP8266_ERROR;
    }
    
    USER_LOG("Connecting to WiFi SSID: %s", ssid);
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);

    // Send the join command - just wait for OK
    result = ESP_SendCommand(cmd, "OK", 15000);
    if (result != ESP8266_OK)
    {
        USER_LOG("WiFi command failed.");
        ESP_ConnState = ESP8266_NOT_CONNECTED;
        return ESP8266_ERROR;
    }

    // OK received - now wait for the connection to actually establish
    USER_LOG("Command accepted. Waiting for connection to establish...");

    // Poll connection status for up to 10 seconds
    for (int i = 0; i < 10; i++)
    {
        HAL_Delay(1000);
        USER_LOG("Checking connection status (attempt %d/10)...", i + 1);

        result = ESP_SendCommand("AT+CWJAP?\r\n", "OK", 3000);
        if (result == ESP8266_OK)
        {
            // Check if we're connected to the right SSID
            if (strstr(esp_rx_buffer, ssid) != NULL && strstr(esp_rx_buffer, "No AP") == NULL)
            {
                USER_LOG("Connected to WiFi!");
                break;
            }
            else if (strstr(esp_rx_buffer, "No AP"))
            {
                DEBUG_LOG("Not connected yet, retrying...");
            }
        }

        if (i == 9)
        {
            USER_LOG("Connection timeout - could not verify connection");
            ESP_ConnState = ESP8266_NOT_CONNECTED;
            return ESP8266_ERROR;
        }
    }

    USER_LOG("WiFi Connected. Fetching IP...");
    ESP_ConnState = ESP8266_CONNECTED_NO_IP;

    result = ESP_GetIP(ip_buffer, buffer_len);
    if (result != ESP8266_OK)
    {
        USER_LOG("Failed to fetch IP. Status=%d", result);
        return ESP8266_ERROR;
    }

    USER_LOG("WiFi + IP ready: %s", ip_buffer);
    ESP_ConnState = ESP8266_CONNECTED_IP;
    return ESP8266_OK;
}

ESP8266_ConnectionState ESP_GetConnectionState(void)
{
    return ESP_ConnState;
}

static ESP8266_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len)
{
    DEBUG_LOG("Fetching IP Address...");

    for (int attempt = 1; attempt <= 3; attempt++)
    {
        ESP8266_Status result = ESP_SendCommand("AT+CIFSR\r\n", "OK", 5000);
        if (result != ESP8266_OK)
        {
            DEBUG_LOG("CIFSR failed on attempt %d", attempt);
            continue;
        }

        char *search = esp_rx_buffer;
        char *last_ip = NULL;

        while ((search = strstr(search, "STAIP,")) != NULL)
        {
            char *ip_start = strstr(search, "STAIP,\"");
            if (ip_start)
            {
                ip_start += 7;
                char *end = strchr(ip_start, '"');
                if (end && ((end - ip_start) < buffer_len))
                {
                    last_ip = ip_start;
                }
            }
            search += 6;
        }

        if (last_ip)
        {
            char *end = strchr(last_ip, '"');
            strncpy(ip_buffer, last_ip, end - last_ip);
            ip_buffer[end - last_ip] = '\0';

            if (strcmp(ip_buffer, "0.0.0.0") == 0)
            {
                DEBUG_LOG("Attempt %d: IP not ready yet (0.0.0.0). Retrying...", attempt);
                ESP_ConnState = ESP8266_CONNECTED_NO_IP;
                HAL_Delay(1000);
                continue;
            }

            DEBUG_LOG("Got IP: %s", ip_buffer);
            ESP_ConnState = ESP8266_CONNECTED_IP;
            return ESP8266_OK;
        }

        DEBUG_LOG("Attempt %d: Failed to parse STAIP.", attempt);
        HAL_Delay(500);
    }

    DEBUG_LOG("Failed to fetch IP after retries.");
    ESP_ConnState = ESP8266_CONNECTED_NO_IP;
    return ESP8266_ERROR;
}

static ESP8266_Status ESP_SendCommand(const char *cmd, const char *ack, uint32_t timeout)
{
    uint8_t ch;
    uint16_t idx = 0;
    uint32_t tickstart;
    int found = 0;

    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
    tickstart = HAL_GetTick();

    if (strlen(cmd) > 0)
    {
        DEBUG_LOG("Sending: %s", cmd);
        if (HAL_UART_Transmit(&ESP_UART, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY) != HAL_OK)
            return ESP8266_ERROR;
    }

    while ((HAL_GetTick() - tickstart) < timeout && idx < sizeof(esp_rx_buffer) - 1)
    {
        if (HAL_UART_Receive(&ESP_UART, &ch, 1, 10) == HAL_OK)
        {
            esp_rx_buffer[idx++] = ch;
            esp_rx_buffer[idx]   = '\0';

            if (!found && strstr(esp_rx_buffer, ack))
            {
                DEBUG_LOG("Matched ACK: %s", ack);
                found = 1;
            }

            if (strstr(esp_rx_buffer, "busy"))
            {
                DEBUG_LOG("ESP is busy... delaying before retry");
                HAL_Delay(1500);
                idx = 0;
                memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
                continue;
            }
        }
    }

    if (found)
    {
        DEBUG_LOG("Full buffer: %s", esp_rx_buffer);
        return ESP8266_OK;
    }

    if (idx == 0)
        return ESP8266_NO_RESPONSE;

    DEBUG_LOG("Timeout or no ACK. Buffer: %s", esp_rx_buffer);
    return ESP8266_TIMEOUT;
}

static ESP8266_Status ESP_SendCommandWiFi(const char *cmd, uint32_t timeout)
{
    uint8_t ch;
    uint16_t idx = 0;
    uint32_t tickstart;
    int got_ok = 0;
    int got_wifi_connected = 0;

    memset(esp_rx_buffer, 0, sizeof(esp_rx_buffer));
    tickstart = HAL_GetTick();

    DEBUG_LOG("Sending WiFi command: %s", cmd);
    if (HAL_UART_Transmit(&ESP_UART, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY) != HAL_OK)
        return ESP8266_ERROR;

    // Keep reading until timeout
    while ((HAL_GetTick() - tickstart) < timeout && idx < sizeof(esp_rx_buffer) - 1)
    {
        if (HAL_UART_Receive(&ESP_UART, &ch, 1, 100) == HAL_OK)
        {
            esp_rx_buffer[idx++] = ch;
            esp_rx_buffer[idx]   = '\0';

            // Check for OK
            if (!got_ok && strstr(esp_rx_buffer, "OK"))
            {
                DEBUG_LOG("Got OK response - continuing to wait for connection status...");
                got_ok = 1;
                // Don't break, keep waiting for WIFI CONNECTED
            }

            // Check for WIFI CONNECTED or WIFI GOT IP
            if (strstr(esp_rx_buffer, "WIFI CONNECTED") || strstr(esp_rx_buffer, "WIFI GOT IP"))
            {
                DEBUG_LOG("WiFi connection confirmed!");
                got_wifi_connected = 1;
                // Keep reading a bit more to get the full message
                uint32_t extra_wait = HAL_GetTick();
                while ((HAL_GetTick() - extra_wait) < 500 && idx < sizeof(esp_rx_buffer) - 1)
                {
                    if (HAL_UART_Receive(&ESP_UART, &ch, 1, 10) == HAL_OK)
                    {
                        esp_rx_buffer[idx++] = ch;
                        esp_rx_buffer[idx] = '\0';
                    }
                }
                break; // Success!
            }

            // Check for failure
            if (strstr(esp_rx_buffer, "FAIL"))
            {
                DEBUG_LOG("Connection FAILED");
                DEBUG_LOG("Full response: %s", esp_rx_buffer);
                return ESP8266_ERROR;
            }

            // Check for specific error codes
            if (strstr(esp_rx_buffer, "+CWJAP:1"))
            {
                DEBUG_LOG("Error: Connection timeout");
                return ESP8266_ERROR;
            }
            if (strstr(esp_rx_buffer, "+CWJAP:2"))
            {
                DEBUG_LOG("Error: Wrong password");
                return ESP8266_ERROR;
            }
            if (strstr(esp_rx_buffer, "+CWJAP:3"))
            {
                DEBUG_LOG("Error: Cannot find target AP");
                return ESP8266_ERROR;
            }
            if (strstr(esp_rx_buffer, "+CWJAP:4"))
            {
                DEBUG_LOG("Error: Connection failed");
                return ESP8266_ERROR;
            }

            // Handle busy response
            if (strstr(esp_rx_buffer, "busy"))
            {
                DEBUG_LOG("ESP is busy... waiting");
                HAL_Delay(1000);
            }
        }
    }

    DEBUG_LOG("Buffer contents (%d bytes):", idx);
    DEBUG_LOG("%s", esp_rx_buffer);

    if (got_wifi_connected)
    {
        return ESP8266_OK;
    }

    if (got_ok && !got_wifi_connected)
    {
        DEBUG_LOG("Got OK but no WIFI CONNECTED - may need more time");
        DEBUG_LOG("Check: 1) WiFi credentials 2) AP is in range 3) AP is on");
        return ESP8266_TIMEOUT;
    }

    DEBUG_LOG("Timeout waiting for WiFi connection");
    return ESP8266_TIMEOUT;
}
