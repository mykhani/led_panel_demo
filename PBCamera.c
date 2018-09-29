#include "PBCamera.h"

#define cmd         4
#define cmdByte1    5
#define value_MSB   6
#define value_LSB   7

#define pkt_num_MSB     6
#define pkt_num_LSB     7
#define num_pkts_MSB    8
#define num_pkts_LSB    9
#define chksum          11

#define PACKET_SIZE 512 /* (default: 512 Bytes) size of single packet.
                          Note: Camera needs to be configured for same size */
void pbcamera_initialise(PBCamera *camera, uint8_t uart, uint8_t id)
{
    camera->bytesTotal = 0;
    camera->bytesRemaining = 0;
    camera->bytesTransferred = 0;
    camera->totalPackets = 0;
    camera->currentPacket = 0;
    camera->uart = uart;
    camera->imageId = 0;
    camera->camera_id = id;
    camera->transferPending = 0;
    camera->imageAvailable = 0;
    camera->flash = 0;
}

int pbcamera_query_parameters(PBCamera *camera, uint8_t* params) {
    uint8_t command[13] = {0x8E, 0x00, 0x08, 0x00, 0x01, 0, 0, 0, 0, 0, 0, 0x09, 0xE8};
    uint8_t response[13] = {0};

    uint8_t uart = camera->uart;
   
    for (int i = 0; i < sizeof(command) ; i++) {
        uart_putc(uart, command[i]);
    }

    int bytes_requested = sizeof(response);
    // Read data from the UART

    for (int i = 0; i < bytes_requested; i++) {
        response[i] = uart_getc(uart);
    }
#if 0
    if (bytes_read != bytes_requested) {
        printf("Fail. Req: %d, Read: %d \r\n", bytes_requested, bytes_read);
        return -1;
    }
#endif

    memcpy(params, response, sizeof(response));

    return 0;
}

static int16_t write_focus_value(uint8_t uart, int auto_focus, int focus_value) {
    uint8_t command[13] = {0x8E, 0x00, 0x08, 0x00, 0x0 /*cmd*/, 0x0/*cmdByte1*/, 0x0 /*value_MSB*/, 0x0 /*value_LSB*/, 0x00, 0x00, 0x00, 0x0 /*chksum*/, 0xE8};
    uint8_t response[13];
    
    command[cmd] = 0x09;

    if (auto_focus) {
        command[cmdByte1] = 0x01;
        command[value_MSB] = 0x00;
        command[value_LSB] = 0x00;  
    } else {
        command[cmdByte1] = 0x03;
        command[value_MSB] = (focus_value >> 8) & 0xFF;
        command[value_LSB] = (focus_value & 0xFF);
    }

    command[chksum] = (8 + command[cmd] + command[cmdByte1] + command[value_MSB] + command[value_LSB]) % 256;
    
    for (int i = 0; i < sizeof(command); i++) {
        uart_putc(uart, command[i]);
    }

    int bytes_requested = sizeof(response);

    for (int i = 0; i < bytes_requested; i++) {
        response[i] = uart_getc(uart);
    }
#if 0
    if (bytes_read != bytes_requested) {
        printf("Fail. Req: %d, Read: %d \r\n", bytes_requested, bytes_read);
        return -1;
    }
#endif

    if (response[4] == 0x0A && response[5] == 0) {
        return 0;
    } else {
        return -1;
    }
}

int pbcamera_set_focus_auto(PBCamera *camera) {
    int retries = 3;
    int ret = -1;
    
    while (retries > 0) {
        if (!write_focus_value(camera->uart, 1, 0)) {
            retries = 0;
            ret = 0;
            break;
        } else {
            retries -= 1;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    return ret;
}

int pbcamera_take_snapshot(PBCamera *camera) {
    uint8_t command[13] = {0x8E, 0x00, 0x08, 0x00, 0x05, 0x08, 0x01, 0x01, 0x00, 0x00, 0x00, 0x17, 0xE8};
    uint8_t response[13];

    uint8_t uart = camera->uart;
    
    for (int i = 0; i < sizeof(command); i++) {
        uart_putc(uart, command[i]);
    }

    int bytes_requested = sizeof(response);
    
    for (int i = 0; i < bytes_requested; i++) {
            response[i] = uart_getc(uart);
    }

#if 0
    if (bytes_read != bytes_requested) {
        printf("Fail. Req: %d, Read: %d \r\n", bytes_requested, bytes_read);
        return -1;
    }
#endif

    if (response[5]) {
        return -1;
    } 
    
    camera->bytesTotal = (response[6] << 16) + (response[7] << 8) + response[8];
    camera->bytesRemaining = camera->bytesTotal;
    camera->totalPackets = (response[9] << 8) + response[10];
    camera->currentPacket = 1; /* packet index starts from 1 */
    camera->imageId++;
    camera->imageAvailable = 1;
    
    return 0;
}

uint8_t pbcamera_get_camera_id(PBCamera *camera)
{
    return camera->camera_id;
}

void pbcamera_get_imagesize(PBCamera *camera, uint32_t *size, uint16_t *packets) {
    *size = camera->bytesTotal;
    *packets = camera->totalPackets;
}

int pbcamera_get_packet(PBCamera *camera, uint8_t *buffer, uint16_t packet_number, uint16_t len) {
    uint8_t command[13] = {0x8E, 0x00, 0x08, 0x00, 0x07, 0x01, 0x0 /*pkt_num_MSB*/, 0x0 /*pkt_num_LSB*/, 0x0 /*num_pkts_MSB*/, 0x0 /*num_pkts_LSB*/, 0x00, 0x0 /*chksum*/, 0xE8};
    uint8_t response[13];
    uint8_t uart = camera->uart;
    int bytes_read; 
    int ret;
    
    command[pkt_num_MSB] = (packet_number >> 8) & 0xFF;
    command[pkt_num_LSB] = (packet_number & 0xFF);
    
    command[num_pkts_MSB] = (camera->totalPackets >> 8) & 0xFF;
    command[num_pkts_LSB] = (camera->totalPackets & 0xFF);
    
    command[chksum] = (8 + 0 + 7 + 1 + command[pkt_num_MSB] + command[pkt_num_LSB] + command[num_pkts_MSB] + command[num_pkts_LSB] + 0) % 256;


    for (int i = 0; i < sizeof(command); i++) {
        uart_putc(uart, command[i]);
    }

    int bytes_requested = sizeof(response) - 2; // subtract last 2 bytes
    
    for (int i = 0; i < bytes_requested; i++) {
        response[i] = uart_getc(uart);
    }
#if 0
    if (bytes_read != bytes_requested) {
        printf("Fail. Req: %d, Read: %d \r\n", bytes_requested, bytes_read);
        return -1;
    }
#endif

    bytes_requested = len;

    for (int i = 0; i < bytes_requested; i++) {
        buffer[i] = uart_getc(uart);
    }
#if 0
    if (bytes_read != bytes_requested) {
        printf("Fail. Req: %d, Read: %d \r\n", bytes_requested, bytes_read);
        return -1;
    }
#endif

    bytes_requested = 2;
    // read last 2 bytes
    for (int i = 0; i < bytes_requested; i++) {
        response[sizeof(response) - 2 + i] = uart_getc(uart);
    }
#if 0
    if (bytes_read != bytes_requested) {
        printf("Fail. Req: %d, Read: %d \r\n", bytes_requested, bytes_read);
        return -1;
    }
#endif
    
    return 0;
}

int pbcamera_image_available(PBCamera *camera) {
    return camera->imageAvailable;
}

