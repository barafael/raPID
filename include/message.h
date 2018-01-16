typedef enum { YPR, TAG, DATA } MESSAGE_TYPE ;
typedef enum { DATA_READY, CONFIG_READY } TAG;

typedef struct {
    MESSAGE_TYPE msg_t;

    union {
        struct {
            uint8_t yaw;
            uint8_t pitch;
            uint8_t roll;
        } YPR;
        TAG tag;
        uint8_t data[512] = { 0 };
    } value;
} message;
