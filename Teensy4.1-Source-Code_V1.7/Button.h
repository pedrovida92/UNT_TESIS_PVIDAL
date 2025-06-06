#ifndef BUTTON_H_
#define BUTTON_H_

typedef struct {
    uint16_t period;
    uint32_t prevTime;
} Loop;

typedef struct {
    byte pin;
    uint16_t debounceTime;
    uint16_t period;
    uint16_t timeout;
    bool pullup;
    bool state;
    bool prevState;
    bool pressed;
    bool prevPushed;
    uint32_t debouncePrevTime;
    int32_t ticks;
} Button;

void button_handle(Button* btn, void (*callback)()) {
    if (!btn->debounceTime) return;
    if (millis() - btn->debouncePrevTime >= btn->debounceTime) {
        btn->debouncePrevTime += btn->debounceTime;
        /* Continuously pressed handle */
        if (btn->state) btn->ticks+=btn->debounceTime;
        if (btn->ticks >= btn->timeout) {
            btn->ticks = btn->timeout - btn->period;  // Avoid overflow
            btn->pressed = true;
            if (btn->period) (*callback)(); // If period is false, do not nothing!
        }
        /* Falling edge handle */
        btn->state = digitalRead(btn->pin);
        if (btn->pullup) btn->state = !btn->state;
        if (!btn->state && btn->prevState) {
            btn->ticks = 0;
            btn->pressed = false;
            (*callback)();
        }
        btn->prevState = btn->state;
    }
}

void button_init(
    Button* btn,
    Button _btn
) {
    pinMode(btn->pin = _btn.pin, (!(btn->pullup = _btn.pullup)) ? INPUT: INPUT_PULLUP);
    btn->debounceTime = _btn.debounceTime;
    btn->timeout = _btn.timeout;
    btn->period = _btn.period;
}


#endif  /* BUTTON_H_ */