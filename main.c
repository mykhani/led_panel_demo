#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <espressif/esp_common.h>
#include <ssid_config.h>

#include <espressif/esp_sta.h>
#include <espressif/esp_wifi.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <paho_mqtt_c/MQTTESP8266.h>
#include <paho_mqtt_c/MQTTClient.h>

#include <esp/uart.h>
#include <esp/uart_regs.h>

#include "base64.h"
#include "cJSON.h"
#include "PBCamera.h"

#define PCA9685_DEBUG
#include <pca9685/pca9685.h>

#define ADDR PCA9685_ADDR_BASE

#define I2C_BUS 0
#define SCL_PIN 4
#define SDA_PIN 5

#define POWER_CHANNEL     4
#define FAR_RED_CHANNEL   2
#define WHITE_CHANNEL     3

#define RED_CHANNEL       1 /* Note: for chamber1, due to physical connection mistake, red and
                             * blue channels have been interchanged. Actually, Red is ch0 and blue is ch1 */
#define BLUE_CHANNEL      0

#define PWM_LED_POWER_ON    4095
#define PWM_LED_POWER_OFF   0

#define PWM_WHITE_ON    4095
#define PWM_WHITE_OFF   0

#define PWM_MAX_COUNT  4095
#define PWM_MIN_COUNT  2048

void IRAM *zalloc(size_t nbytes);

int wifi_disconnected = 0;

#define CAMERA_UART 0

#define BUF_SIZE (1024)
#define MAX_PACKET_SIZE (512)
int reset_image = 0;

SemaphoreHandle_t wifi_alive;

/* FreeRTOS semaphore to signal when to take a snapshot */
SemaphoreHandle_t capture_image;
/* For syncing image publish */
SemaphoreHandle_t publish_image;

QueueHandle_t publish_queue;
#define PUB_MSG_LEN 16

PBCamera camera;

/* You can use http://test.mosquitto.org/ to test mqtt_client instead
 * of setting up your own MQTT server */

#define MQTT_HOST ("chamber1.local")
#define MQTT_PORT 5555

#define MQTT_USER NULL
#define MQTT_PASS NULL

#define MQTT_CLIENT_THREAD_NAME         "mqtt_client_thread"
#define MQTT_CLIENT_THREAD_STACK_WORDS  8192
#define MQTT_CLIENT_THREAD_PRIO         8

#define CAMERA_SERVER "192.168.10.1"
#define CAMERA_PORT   7777

#define CAMERA_CLIENT_THREAD_NAME           "camera_client_thread"
#define CAMERA_CLIENT_THREAD_STACK_WORDS    8192
#define CAMERA_CLIENT_THREAD_PRIO           7

/* wrapper for passing json message to publish queue */
typedef struct QueueMessage_t {
    size_t len;
    uint8_t *data;
} QueueMessage;

enum {
    RED_LED,
    BLUE_LED,
    FAR_RED_LED
};

typedef struct led_controller_t {
    uint16_t red;
    uint16_t blue;
    uint16_t far_red;
    i2c_dev_t dev;
} LedController;

LedController led_controller;

static void  beat_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    char msg[PUB_MSG_LEN];
    int count = 0;

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, 10000 / portTICK_PERIOD_MS);
        printf("beat\r\n");
        snprintf(msg, PUB_MSG_LEN, "Beat %d\r\n", count++);
        if (xQueueSend(publish_queue, (void *)msg, 0) == pdFALSE) {
            printf("Publish queue overflow.\r\n");
        }
    }
}


static void  wifi_task(void *pvParameters)
{
    uint8_t status  = 0;
    uint8_t retries = 30;
    struct sdk_station_config config = {
        .ssid = WIFI_SSID,
        .password = WIFI_PASS,
    };

    printf("WiFi: connecting to WiFi\n\r");
    sdk_wifi_set_opmode(STATION_MODE);
    sdk_wifi_station_set_config(&config);

    while(1)
    {
        while ((status != STATION_GOT_IP) && (retries)){
            status = sdk_wifi_station_get_connect_status();
            printf("%s: status = %d\n\r", __func__, status );
            if( status == STATION_WRONG_PASSWORD ){
                printf("WiFi: wrong password\n\r");
                break;
            } else if( status == STATION_NO_AP_FOUND ) {
                printf("WiFi: AP not found\n\r");
                break;
            } else if( status == STATION_CONNECT_FAIL ) {
                printf("WiFi: connection failed\r\n");
                break;
            }
            vTaskDelay( 1000 / portTICK_PERIOD_MS );
            --retries;
        }
#if 0
        if (status == STATION_GOT_IP) {
            printf("WiFi: Connected\n\r");
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
#endif

        while ((status = sdk_wifi_station_get_connect_status()) == STATION_GOT_IP) {
            xSemaphoreGive( wifi_alive );
            taskYIELD();
        }
        printf("WiFi: disconnected\n\r");
        sdk_wifi_station_disconnect();
        //retries = 30;
        //vTaskDelay( 1000 / portTICK_PERIOD_MS );
        printf("Restarting...\n");
        uart_flush_txfifo(0);
        uart_flush_txfifo(1);
        sdk_system_restart();
    }
}

static void  topic_received(mqtt_message_data_t *md)
{
    int i;
    mqtt_message_t *message = md->message;
    printf("Received: ");
    for( i = 0; i < md->topic->lenstring.len; ++i)
        printf("%c", md->topic->lenstring.data[ i ]);

    printf(" = ");
    for( i = 0; i < (int)message->payloadlen; ++i)
        printf("%c", ((char *)(message->payload))[i]);

    printf("\r\n");
}

static void handleCameraCommand(mqtt_message_data_t* data)
{
    char *message = data->message->payload;
    bool take_snapshot = false;
    printf("Camera cmd arrived\r\n");
    //printf("Message arrived: %s\r\n", message);

    cJSON *root = cJSON_Parse(message);
    if (!root) {
             printf("Error before: [%s]\n", cJSON_GetErrorPtr());
    }

    const cJSON *flash = NULL;
    flash = cJSON_GetObjectItemCaseSensitive(root, "flash");
    if (cJSON_IsNumber(flash) && flash->valueint) {
        camera.flash = flash->valueint;
        printf("Flash: %d \r\n", camera.flash);
    }

    const cJSON *capture = NULL;
    capture = cJSON_GetObjectItemCaseSensitive(root, "capture");

    if (cJSON_IsNumber(capture) && capture->valueint) {
        printf("Capture: %d \r\n", capture->valueint);
        take_snapshot = true;
    }

    cJSON_Delete(root);

    if (take_snapshot) {
        xSemaphoreGive(capture_image);
    }
}

static void handleCameraReset(mqtt_message_data_t* data)
{
    char *message = data->message->payload;
    bool reset = false;
    printf("Camera reset cmd arrived##########################################\r\n");
    //printf("Message arrived: %s\r\n", message);

    cJSON *root = cJSON_Parse(message);
    if (!root) {
             printf("Error before: [%s]\n", cJSON_GetErrorPtr());
    }

    const cJSON *camera_id = NULL;
    camera_id = cJSON_GetObjectItemCaseSensitive(root, "camera_id");
    if (cJSON_IsNumber(camera_id)) {
        if (camera_id->valueint == pbcamera_get_camera_id(&camera)) {
            printf("Image reset command received\r\n");
            reset_image = 1;
        }
    }
}


static void turn_on_flash(LedController *controller)
{
    pca9685_set_pwm_value(&controller->dev, WHITE_CHANNEL, 4095);
}

static void turn_off_flash(LedController *controller)
{
    pca9685_set_pwm_value(&controller->dev, WHITE_CHANNEL, 0);
}

static void power_on_leds(LedController *controller)
{
    pca9685_set_pwm_value(&controller->dev, POWER_CHANNEL, 4095);
}

static void power_off_leds(LedController *controller)
{
    pca9685_set_pwm_value(&controller->dev, POWER_CHANNEL, 0);
}

static void led_set_brightness(LedController *controller, uint8_t color, uint16_t val)
{
/* A Note on how LEDs are wired up:
 *---------------------------------
 * Currently color LEDs are connected via inverted inputs.
 * It means a 1 on pwm output is 0 for color leds.
 * Moreover, due to high current demand, and the fact that
 * we are not still sure how much power can the chamber
 * power circuit can provide, to be on the safe side,
 * we have limited the maximum duty cycle (brightness) to 50%
 * i.e. the duty cycle cannot be more than 50%.
 *
 * Use case example:
 * For duty cycle of 100%, the pwm output is straight HIGH
 * but after inversion, it appears as LOW to LED driver circuit.
 * For 100% duty cycle, the pwm count is 4095. It means to set
 * any pwm channel to LOW, the pwm count should be 4095 and to
 * set it HIGH, pwm count should be 0. Since we are operating
 * at 50% duty cycle max, the value of pwm count cannot be less
 * than half of 4095 or 2048.
 *
 * Argument val:
 * The val argument varies from 0 - 100 and currently corresponds to
 * the percentage of a LED color in total PAR i.e. Photosynthetically
 * Active Radiation.
 */
    uint16_t pwm_count = PWM_MAX_COUNT - (PWM_MIN_COUNT * val) / 100;

    switch (color) {
    case RED_LED:
        pca9685_set_pwm_value(&controller->dev, RED_CHANNEL, pwm_count);
        controller->red = val;
        printf("Red led pwm cnt: %d \r\n", pwm_count);
        break;
    case BLUE_LED:
        pca9685_set_pwm_value(&controller->dev, BLUE_CHANNEL, pwm_count);
        controller->blue = val;
        printf("Blue led pwm cnt: %d \r\n", pwm_count);
        break;
    case FAR_RED_LED:
        pca9685_set_pwm_value(&controller->dev, FAR_RED_CHANNEL, pwm_count);
        controller->far_red = val;
        printf("Far Red led pwm cnt: %d \r\n", pwm_count);
        break;
    default:
        break;
    }
}

static void handleLightCommand(mqtt_message_data_t* data)
{
    char *message = data->message->payload;
    
    printf("Light cmd arrived.\r\n");
    //printf("Light cmd arrived: %s \r\n", message);

    cJSON *root = cJSON_Parse(message);
    if (!root) {
        printf("Error before: [%s]\n", cJSON_GetErrorPtr());
    }

    const cJSON *red = NULL;
    red = cJSON_GetObjectItemCaseSensitive(root, "red");
    if (cJSON_IsNumber(red)) {
        if (led_controller.red != red->valueint) {
            led_set_brightness(&led_controller, RED_LED, red->valueint);
        }
    }

    const cJSON *blue = NULL;
    blue = cJSON_GetObjectItemCaseSensitive(root, "blue");
    if (cJSON_IsNumber(blue)) {
        if (led_controller.blue != blue->valueint) {
            led_set_brightness(&led_controller, BLUE_LED, blue->valueint);
        }
    }

    const cJSON *far_red = NULL;
    far_red = cJSON_GetObjectItemCaseSensitive(root, "far_red");
    if (cJSON_IsNumber(far_red)) {
        if (led_controller.far_red != far_red->valueint) {
            led_set_brightness(&led_controller, FAR_RED_LED, far_red->valueint);
        }
    }
    cJSON_Delete(root);
}

static uint8_t i2c_detect(void)
{
    uint8_t i = 0;
    for (i = PCA9685_ADDR_BASE; i < PCA9685_ADDR_BASE + 32; i++) {
        if (!i2c_slave_write(led_controller.dev.bus, i, NULL, NULL, 0)) {
            printf("Detected i2c device at : %x \r\n", i);
            break;
        }
    }
    
    return i;
}

static uint8_t get_camera_id(uint8_t addr)
{
    uint8_t camera_id;
    
    switch (addr - PCA9685_ADDR_BASE) {
    case 1: 
        camera_id = 1;
        break;
    case 2:
        camera_id = 2;
        break;
    case 4:
        camera_id = 3;
        break;
    default:
        camera_id = 0;
    }

    return camera_id;
}

static const char *  get_my_id(void)
{
    // Use MAC address for Station as unique ID
    static char my_id[13];
    static bool my_id_done = false;
    int8_t i;
    uint8_t x;
    if (my_id_done)
        return my_id;
    if (!sdk_wifi_get_macaddr(STATION_IF, (uint8_t *)my_id))
        return NULL;
    for (i = 5; i >= 0; --i)
    {
        x = my_id[i] & 0x0F;
        if (x > 9) x += 7;
        my_id[i * 2 + 1] = x + '0';
        x = my_id[i] >> 4;
        if (x > 9) x += 7;
        my_id[i * 2] = x + '0';
    }
    my_id[12] = '\0';
    my_id_done = true;
    return my_id;
}

#define MQTT_TX_BUFSIZE (1024 + 128)
#define MQTT_RX_BUFSIZE (512)

static void  mqtt_task(void *pvParameters)
{
    int ret         = 0;
    struct mqtt_network network;
    mqtt_client_t client   = mqtt_client_default;
    char mqtt_client_id[20];
    mqtt_packet_connect_data_t data = mqtt_packet_connect_data_initializer;

    uint8_t *mqtt_buf = (uint8_t *)zalloc(MQTT_TX_BUFSIZE);
    uint8_t *mqtt_readbuf = (uint8_t *)zalloc(MQTT_RX_BUFSIZE);

    mqtt_network_new( &network );
    memset(mqtt_client_id, 0, sizeof(mqtt_client_id));
    strcpy(mqtt_client_id, "ESP-");
    strcat(mqtt_client_id, get_my_id());

    while(1) {
        xSemaphoreTake(wifi_alive, portMAX_DELAY);
        printf("%s: started\n\r", __func__);
        printf("%s: (Re)connecting to MQTT server %s ... ",__func__,
               MQTT_HOST);
        ret = mqtt_network_connect(&network, MQTT_HOST, MQTT_PORT);
        if( ret ){
            printf("error: %d\n\r", ret);
            taskYIELD();
            continue;
        }
        printf("done\n\r");
        mqtt_client_new(&client, &network, 5000, mqtt_buf, MQTT_TX_BUFSIZE,
                      mqtt_readbuf, MQTT_RX_BUFSIZE);

        data.willFlag       = 0;
        data.MQTTVersion    = 3;
        data.clientID.cstring   = mqtt_client_id;
        data.username.cstring   = MQTT_USER;
        data.password.cstring   = MQTT_PASS;
        data.keepAliveInterval  = 10;
        data.cleansession   = 0;
        printf("Send MQTT connect ... ");
        ret = mqtt_connect(&client, &data);
        if(ret){
            printf("error: %d\n\r", ret);
            mqtt_network_disconnect(&network);
            taskYIELD();
            continue;
        }
        printf("done\r\n");
        mqtt_subscribe(&client, "/chamber/device/camera/cmd", MQTT_QOS2, handleCameraCommand);
        mqtt_subscribe(&client, "/chamber/device/camera/reset", MQTT_QOS2, handleCameraReset);
        mqtt_subscribe(&client, "/chamber/device/light/cmd", MQTT_QOS2, handleLightCommand);
        xQueueReset(publish_queue);

        while(1){

            char msg[PUB_MSG_LEN - 1] = "\0";
            QueueMessage *qmsg;

            while(xQueueReceive(publish_queue, &qmsg, 0) ==
                  pdTRUE){
                printf("got message to publish\r\n");
                mqtt_message_t message;
                message.payload = qmsg->data;
                message.payloadlen = qmsg->len;
                message.dup = 0;
                message.qos = MQTT_QOS2;
                message.retained = 0;
                ret = mqtt_publish(&client, "/chamber/device/camera/image", &message);
                if (ret != MQTT_SUCCESS ){
                    printf("error while publishing message: %d\n", ret );
                    /* TODO: handle what to do when publishing fails.
                     * Maybe retransmit message? */
                    xSemaphoreGive(publish_image);
                    break;
                } else {
                    printf("Published \r\n");
                    xSemaphoreGive(publish_image);
                }
            }

            ret = mqtt_yield(&client, 100);
            if (ret == MQTT_DISCONNECTED)
                break;
        }
        printf("Connection dropped, request restart\n\r");
        mqtt_network_disconnect(&network);
        taskYIELD();
    }

    free(mqtt_buf);
    free(mqtt_readbuf);
}

static void camera_task(void* pvParameters)
{
    printf("Waiting for capture command \r\n");
 
    while (1) {
    
        /* Wait for the capture command
        */
        xSemaphoreTake(capture_image, portMAX_DELAY);
        printf("Capturing..\r\n");
#if 0
        uint8_t params[13] = {0};

        if (pbcamera_query_parameters(&camera, params)) {
            printf("Failed to read params \r\n");
            //vTaskDelay(50 / portTICK_PERIOD_MS); // add 50 ms delay
            xSemaphoreGive(capture_image);
            continue;
        }
#endif

        if (pbcamera_set_focus_auto(&camera)) {
            printf("Failed autofocus \r\n");
            //vTaskDelay(50 / portTICK_PERIOD_MS); // add 50 ms delay
            xSemaphoreGive(capture_image);
            continue;
        }
        
        power_off_leds(&led_controller); // turn off all color leds to avoid image distortion
        turn_on_flash(&led_controller); // turn on camera flash

        vTaskDelay(50 / portTICK_PERIOD_MS); // add 50 ms delay

        if (pbcamera_take_snapshot(&camera)) {
            printf("Failed to take snapshot \r\n");
            //vTaskDelay(50 / portTICK_PERIOD_MS); // add 50 ms delay
            xSemaphoreGive(capture_image);
            continue;
        }

        turn_off_flash(&led_controller);
        power_on_leds(&led_controller);

        if (camera.imageAvailable) {
            int packet_num = 1;
            int bytes_requested = 0;
            /* now start transferring image */
            while (camera.imageAvailable && camera.bytesRemaining) {
                uint8_t *raw_packet = (uint8_t *)zalloc(MAX_PACKET_SIZE);
                if (reset_image) {
                    /* wait a bit to receive all pending reset messages */
                    vTaskDelay(2000 / portTICK_PERIOD_MS); // 2 seconds wait
                    packet_num = 1;
                    camera.bytesRemaining= camera.bytesTotal;

                    reset_image = 0; 
                }
                printf("Reading packet: %d \r\n", packet_num);
                if (camera.bytesRemaining > MAX_PACKET_SIZE) {
                    bytes_requested = MAX_PACKET_SIZE;
                } else {
                    bytes_requested = camera.bytesRemaining;
                }
                if (pbcamera_get_packet(&camera, raw_packet, packet_num, bytes_requested)) {
                    printf("Failed to read packet: %d \r\n", packet_num);
                } else {
                    char *b64data = (char *)zalloc(1024);
                    int b64len = b64_encode(b64data, raw_packet, bytes_requested);

                    cJSON *msg = cJSON_CreateObject();
                    if (msg == NULL) {
                        printf("Failed to create JSON msg object\r\n");
                        goto end;
                    }
                    
                    cJSON *camera_id = cJSON_CreateNumber(camera.camera_id);
                    if (camera_id == NULL) {
                        printf("Failed to create JSON msg camera_id object\r\n");
                        goto end;
                    }
                    cJSON_AddItemToObject(msg, "camera_id", camera_id);

                    cJSON *image_id = cJSON_CreateNumber(camera.imageId);
                    if (image_id == NULL) {
                        printf("Failed to create JSON msg image_id object\r\n");
                        goto end;
                    }
                    cJSON_AddItemToObject(msg, "image_id", image_id);

                    /* current position in image. 0 indexed */
                    cJSON *position = cJSON_CreateNumber(packet_num - 1);
                    if (position == NULL) {
                        printf("Failed to create JSON msg position object\r\n");
                        goto end;
                    }
                    cJSON_AddItemToObject(msg, "position", position);

                    cJSON *total_pkts = cJSON_CreateNumber(camera.totalPackets);
                    if (total_pkts == NULL) {
                        printf("Failed to create JSON msg total_pkts object\r\n");
                        goto end;
                    }
                    cJSON_AddItemToObject(msg, "total_pkts", total_pkts);

                    cJSON *pkt_size = cJSON_CreateNumber(MAX_PACKET_SIZE);
                    if (pkt_size == NULL) {
                        printf("Failed to create JSON msg pkt_size object\r\n");
                        goto end;
                    }
                    cJSON_AddItemToObject(msg, "pkt_size", pkt_size);

                    cJSON *data_len = cJSON_CreateNumber(b64len);
                    if (data_len == NULL) {
                        printf("Failed to create JSON msg data_len object\r\n");
                        goto end;
                    }

                    cJSON_AddItemToObject(msg, "data_len", data_len);

                    cJSON *data = cJSON_CreateString(b64data);
                    if (data == NULL) {
                        printf("Failed to create JSON msg pkt_size object\r\n");
                        goto end;
                    }
                    cJSON_AddItemToObject(msg, "data", data);
                    char *json_msg = cJSON_Print(msg);
                    if (json_msg == NULL) {
                        printf("Failed to print json msg string\r\n");
                        goto end;
                    }

                    printf("Json string len: %d \r\n", strlen(json_msg));

                    QueueMessage *qmsg = (QueueMessage*) zalloc(sizeof(QueueMessage));
                    qmsg->len = strlen(json_msg) + 1; // for string NULL delimiter
                    qmsg->data = json_msg;

                    if (xQueueSend(publish_queue, &qmsg, portMAX_DELAY) == pdFALSE) {
                        printf("Publish queue overflow.\r\n");
                    }

                    xSemaphoreTake(publish_image, portMAX_DELAY);

                    camera.bytesRemaining -= bytes_requested;
                    packet_num++;
end:
                    cJSON_Delete(msg);
                    free(b64data);
                    free(json_msg);
                    free(qmsg);
                }
                free(raw_packet);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            printf("Read all packets \r\n");
        }

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }

    printf("camera_client_thread going to be deleted\n");
    vTaskDelete(NULL);
    return;
}

void user_uart_init(void)
{
    uart_set_baud(0, 115200);

    gpio_set_iomux_function(2, IOMUX_GPIO2_FUNC_UART1_TXD);
    
    /* Set baud rate of UART1 used for debugging  */
    uart_set_baud(1, 115200);
}

void led_controller_init(LedController *controller, uint8_t addr)
{
    controller->red = 0;
    controller->blue = 0;
    controller->far_red = 0;
    controller->dev.bus = I2C_BUS;
    controller->dev.addr = addr;

    pca9685_init(&controller->dev);

    pca9685_set_pwm_frequency(&controller->dev, 500);
    printf("Freq 500Hz, real %d\n", pca9685_get_pwm_frequency(&controller->dev));
    power_off_leds(controller);

    led_set_brightness(controller, RED_LED, 0);

    led_set_brightness(controller, BLUE_LED, 0);

    led_set_brightness(controller, FAR_RED_LED, 0);

    power_on_leds(controller);
    turn_off_flash(controller);
}

void user_init(void)
{
    user_uart_init();
    i2c_init(I2C_BUS, SCL_PIN, SDA_PIN, I2C_FREQ_100K);
    
    uint8_t addr = 0;

    addr = i2c_detect();
    led_controller_init(&led_controller, addr);

    /* Since we will have 3 cameras in a chamber and currently
     * we dont have any other means of identifying individual
     * camera, we are going to use the led controller's dip 
     * switch settable id as camera id */
    uint8_t camera_id = get_camera_id(addr);
    if (camera_id == 0) {
        printf("Please select camera address using dip switch \r\n");
        printf("Restarting...\n");
        uart_flush_txfifo(0);
        uart_flush_txfifo(1);
        sdk_system_restart();
        while (1);
    }



    vSemaphoreCreateBinary(wifi_alive);
    vSemaphoreCreateBinary(capture_image);
    vSemaphoreCreateBinary(publish_image);
    
    /* By default take semaphore */
    xSemaphoreTake(capture_image, portMAX_DELAY);
    
    xSemaphoreTake(publish_image, portMAX_DELAY);

    publish_queue = xQueueCreate(1, sizeof(uint8_t *));
    
    printf("Initializing camera with id: %u \r\n", camera_id);
    pbcamera_initialise(&camera, CAMERA_UART, camera_id);

    xTaskCreate(&wifi_task, "wifi_task",  256, NULL, 2, NULL);
    //xTaskCreate(&beat_task, "beat_task", 256, NULL, 3, NULL);
    xTaskCreate(&mqtt_task, "mqtt_task", 512, NULL, 4, NULL);
    xTaskCreate(&camera_task, "camera_task", 512, NULL, 5, NULL);

}
