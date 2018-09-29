#ifndef PBCamera_h
#define PBCamera_h

#include <FreeRTOS.h>
#include <task.h>
#include "esp/uart.h"

#include "string.h"

typedef struct PBCamera {
    uint32_t bytesTotal;
    int32_t bytesRemaining;
    uint32_t bytesTransferred;
    uint16_t totalPackets;
    uint16_t currentPacket;
    uint8_t camera_id;
    uint8_t uart;
    uint8_t imageId;
    int8_t transferPending;
    int8_t imageAvailable;
    uint8_t flash;
} PBCamera;

void    pbcamera_initialise(PBCamera *camera, uint8_t uart, uint8_t id);
int     pbcamera_query_parameters(PBCamera *camera, uint8_t *params);
int     pbcamera_set_focus_auto(PBCamera *camera);
int     pbcamera_take_snapshot(PBCamera *camera);
uint8_t pbcamera_get_camera_id(PBCamera *camera);
void    pbcamera_get_imagesize(PBCamera *camera, uint32_t *size, uint16_t *packets);
int     pbcamera_get_packet(PBCamera *camera, uint8_t *buffer, uint16_t packet_number, uint16_t len);
int     pbcamera_image_available(PBCamera *camera);

#endif
