#include "ESP8266_STM32.h"

ESP8266_ConnectionState ESP_ConnState = ESP8266_DISCONNECTED;   // Default state
static char esp_rx_buffer[2048];
static ESP8266_Status ESP_GetIP(char *ip_buffer, uint16_t buffer_len);
static ESP8266_Status ESP_SendCommand(const char *cmd, const char *ack, uint32_t timeout);

static uint8_t current_connection_id = 0;

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

ESP8266_Status ESP_StartWebServer(uint16_t port)
{
    char cmd[64];
    ESP8266_Status result;

    USER_LOG("Starting Web Server on port %d...", port);

    // Enable multiple connections
    result = ESP_SendCommand("AT+CIPMUX=1\r\n", "OK", 2000);
    if (result != ESP8266_OK)
    {
        DEBUG_LOG("Failed to enable multiple connections");
        return result;
    }

    // Start TCP server
    snprintf(cmd, sizeof(cmd), "AT+CIPSERVER=1,%d\r\n", port);
    result = ESP_SendCommand(cmd, "OK", 2000);
    if (result != ESP8266_OK)
    {
        DEBUG_LOG("Failed to start server");
        return result;
    }

    // Set timeout for server
    result = ESP_SendCommand("AT+CIPSTO=180\r\n", "OK", 2000);
    if (result != ESP8266_OK)
    {
        DEBUG_LOG("Failed to set timeout");
        return result;
    }

    USER_LOG("Web Server Started Successfully!");
    return ESP8266_OK;
}

ESP8266_Status ESP_CheckForClient(char *request_buffer, uint16_t buffer_len)
{
    uint8_t ch;
    uint16_t idx = 0;
    static uint32_t last_data_log = 0;

    memset(request_buffer, 0, buffer_len);

    // First, do a quick check if ANY data is available
    if (HAL_UART_Receive(&ESP_UART, &ch, 1, 1) != HAL_OK)
    {
        return ESP8266_NO_RESPONSE;  // No data at all
    }

    // Data is available! Store first byte
    request_buffer[idx++] = ch;
    request_buffer[idx] = '\0';

    // Now read continuously with longer timeout per byte
    // Keep reading as long as we keep getting data
    uint32_t last_byte_time = HAL_GetTick();

    while (idx < buffer_len - 1)
    {
        // Wait up to 200ms for next byte
        if (HAL_UART_Receive(&ESP_UART, &ch, 1, 200) == HAL_OK)
        {
            request_buffer[idx++] = ch;
            request_buffer[idx] = '\0';
            last_byte_time = HAL_GetTick();
        }
        else
        {
            // No more data for 200ms - we're done
            break;
        }

        // Safety timeout - if we've been reading for more than 2 seconds, stop
        if ((HAL_GetTick() - last_byte_time + 200) > 2000)
        {
            DEBUG_LOG("Safety timeout after %d bytes", idx);
            break;
        }
    }

    if (idx > 0)
    {
        DEBUG_LOG("=== Received %d bytes ===", idx);
        DEBUG_LOG("Full data: %s", request_buffer);

        // Check for connection notification
        if (strstr(request_buffer, ",CONNECT"))
        {
            DEBUG_LOG("Found CONNECT message");

            // Extract connection ID (digit before ,CONNECT)
            char *conn_ptr = strstr(request_buffer, ",CONNECT");
            if (conn_ptr && conn_ptr > request_buffer)
            {
                char prev_char = *(conn_ptr - 1);
                if (prev_char >= '0' && prev_char <= '9')
                {
                    current_connection_id = prev_char - '0';
                    DEBUG_LOG("Connection ID: %d", current_connection_id);
                    return ESP8266_OK;
                }
            }
        }

        // Check for +IPD data
        if (strstr(request_buffer, "+IPD,"))
        {
            DEBUG_LOG("Found +IPD message");

            char *id_ptr = strstr(request_buffer, "+IPD,");
            if (id_ptr && id_ptr[5] >= '0' && id_ptr[5] <= '9')
            {
                current_connection_id = id_ptr[5] - '0';
                DEBUG_LOG("Connection ID: %d", current_connection_id);
                return ESP8266_OK;
            }
        }

        // Check if it's just a connection ID by itself
        if (idx >= 1 && idx <= 3 && request_buffer[0] >= '0' && request_buffer[0] <= '9')
        {
            current_connection_id = request_buffer[0] - '0';
            DEBUG_LOG("Got connection ID alone: %d", current_connection_id);

            // Continue reading to get more data
            uint32_t extra_start = HAL_GetTick();
            while ((HAL_GetTick() - extra_start) < 1000 && idx < buffer_len - 1)
            {
                if (HAL_UART_Receive(&ESP_UART, &ch, 1, 100) == HAL_OK)
                {
                    request_buffer[idx++] = ch;
                    request_buffer[idx] = '\0';
                    extra_start = HAL_GetTick(); // Reset timer
                }
            }

            DEBUG_LOG("After waiting, total: %d bytes", idx);
            DEBUG_LOG("Data now: %s", request_buffer);

            // Check again for CONNECT or +IPD
            if (strstr(request_buffer, "CONNECT") || strstr(request_buffer, "+IPD"))
            {
                return ESP8266_OK;
            }
        }

        // Log unrecognized format occasionally
        if (HAL_GetTick() - last_data_log > 3000)
        {
            DEBUG_LOG("Unrecognized format (throttled logging)");
            last_data_log = HAL_GetTick();
        }
    }

    return ESP8266_NO_RESPONSE;
}

ESP8266_Status ESP_SendWebPage(const char *html_content)
{
    char cmd[128];
    uint16_t content_len = strlen(html_content);

    DEBUG_LOG("Sending web page (%d bytes)...", content_len);

    // Prepare to send data
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,%d\r\n", current_connection_id, content_len);

    // Clear any pending data first
    uint8_t dummy;
    while (HAL_UART_Receive(&ESP_UART, &dummy, 1, 1) == HAL_OK);

    // Send CIPSEND and wait for ">"
    DEBUG_LOG("Sending: %s", cmd);
    if (HAL_UART_Transmit(&ESP_UART, (uint8_t*)cmd, strlen(cmd), 1000) != HAL_OK)
    {
        DEBUG_LOG("Failed to send command");
        return ESP8266_ERROR;
    }

    // Wait for ">" prompt
    uint8_t ch;
    uint32_t start = HAL_GetTick();
    char rx_buf[64] = {0};
    int rx_idx = 0;

    while ((HAL_GetTick() - start) < 3000 && rx_idx < 63)
    {
        if (HAL_UART_Receive(&ESP_UART, &ch, 1, 10) == HAL_OK)
        {
            rx_buf[rx_idx++] = ch;
            rx_buf[rx_idx] = '\0';

            if (ch == '>')
            {
                DEBUG_LOG("Got prompt, sending data");
                HAL_Delay(10);

                // Send HTML
                if (HAL_UART_Transmit(&ESP_UART, (uint8_t*)html_content, content_len, 5000) != HAL_OK)
                {
                    DEBUG_LOG("Failed to send HTML");
                    return ESP8266_ERROR;
                }

                DEBUG_LOG("HTML sent successfully");
                HAL_Delay(100);
                return ESP8266_OK;
            }

            // Check for ERROR
            if (strstr(rx_buf, "ERROR"))
            {
                DEBUG_LOG("Got ERROR response: %s", rx_buf);
                return ESP8266_ERROR;
            }
        }
    }

    DEBUG_LOG("Timeout waiting for prompt. Received: %s", rx_buf);
    return ESP8266_TIMEOUT;
}

ESP8266_Status ESP_CloseConnection(void)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%d\r\n", current_connection_id);

    DEBUG_LOG("Closing connection %d", current_connection_id);

    // Send close command
    HAL_UART_Transmit(&ESP_UART, (uint8_t*)cmd, strlen(cmd), 1000);

    // Give it time to close
    HAL_Delay(50);

    return ESP8266_OK;
}
