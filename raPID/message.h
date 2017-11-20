enum message_type = { YPR, TAG, DATA };
enum TAG          = { DATA_READY, CONFIG_READY };

typedef struct {
    enum message_type msg_t;

    union {
        struct {
            uint8_t yaw;
            uint8_t pitch;
            uint8_t roll;
        } YPR;
        enum TAG;
        uint8_t[512];
    } value;
} message;
