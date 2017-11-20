typedef enum {
    DMP_INIT_MEM_LOAD_FAILED,
    DMP_CONF_UPDATES_FAILED,
    DMP_ERROR_UNKNOWN
} error_type;

static const uint16_t BLINK_PERIOD = 500;

void blink_pattern(const char* pattern) {
    while(1) {
        for(size_t index = 0; index < strlen(pattern); index++) {
            switch(pattern[index]) {
                case '0':
                    digitalWrite(13, LOW);
                    delay(BLINK_PERIOD);
                    break;
                case '1':
                    digitalWrite(13, HIGH);
                    delay(BLINK_PERIOD);
                    break;
                default:
                    Serial.println(F("Invalid pattern string!"));
                    return;
            }
        }
    }
}

void error_blink(error_type error, const char* message) {
    Serial.println(message);
    switch(error) {
        case DMP_INIT_MEM_LOAD_FAILED:
            blink_pattern("1100");
            break;
        case DMP_CONF_UPDATES_FAILED:
            blink_pattern("1010");
            break;
        case DMP_ERROR_UNKNOWN:
            blink_pattern("0001");
            break;
        default:
            blink_pattern("10100000");
            break;
    }
}

