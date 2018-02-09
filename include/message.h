#include <stdint.h>

typedef enum { YPR, TAG, DATA } message_type;
typedef enum { DATA_READY, CONFIG_READY } msg_tag;

typedef struct {
    message_type msg_t;

    union {
        struct {
            uint8_t yaw;
            uint8_t pitch;
            uint8_t roll;
        } YPR;
        msg_tag tag;
        uint8_t data[512] = { 0 };
    } value;
} message;
