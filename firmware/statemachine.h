enum mode_type { MODE_OFF = 0, MODE_PRECHARGE, MODE_CLOSING, MODE_ON };
extern int mode; // avoid multiple definitions https://stackoverflow.com/questions/33464242/c-multiple-definition-of-variable-even-with-extern
extern uint32_t last_mode_change; // defined in main program

void mode_off();
void mode_precharge();
void mode_closing();
void mode_on();
