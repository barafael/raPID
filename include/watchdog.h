#ifndef WATCHDOG_H
#define WATCHDOG_H

typedef struct {
} watchdog_t;

watchdog_t init_watchdog();
void watchdog_feed(watchdog_t *self);
void watchdog_set_prescale(watchdog_t *self, int prescaler);

#endif // WATCHDOG_H
