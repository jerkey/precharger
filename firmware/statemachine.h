enum mode_type { MODE_OFF = 0, MODE_PRECHARGE, MODE_CLOSING, MODE_ON };
extern uint32_t last_mode_change; // defined in main program

int get_mode();
void set_mode(int mode_to_set);
void state_machine();
void mode_off();
void mode_precharge();
void mode_closing();
void mode_on();
