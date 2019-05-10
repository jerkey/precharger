#define BAUDRATE        9600

uint32_t last_mode_change = 0;
enum mode_type { MODE_OFF = 0, MODE_PRECHARGE, MODE_CLOSING, MODE_ON };
int mode = MODE_OFF;

#define PRECHARGE_PIN   13
#define DCDC_ENABLE_PIN 12
#define POWER_BUTTON    11
#define CONTACTOR_PIN   10

#define HV_BATT         A0
#define HV_PRECHARGE    A1
#define HV_DIVISOR      (1023./103900.*3900./5./1.042) // ADC / resistors total * shunt resistor / AREF / fudge
#define LV_SENSE        A2
#define LV_DIVISOR      (1023./8810.*2000./5.) // ADC / resistors total * shunt resistor / AREF
#define BATTERY_AMPS    A4 // current sensor inside Zero battery contactor
#define BATTERY_AMPS_DIVISOR    1.4 // 4.0A = 515.0
#define BATTERY_AMPS_ZERO       509.4 // at zero current flow.  increases with discharge current
#define OVERSAMPLES     25.0 // number of times to analogRead() for oversampling
#define MIN_TURNON_VOLTAGE      (2.9*28) // 2.9*28=81.2 below this voltage we just won't turn on
#define MAX_BATTERY_AMPS        500 // this shouldn't happen right?
#define CONTACTOR_V_LATCH       15 // voltage to click contactor shut hard for a second
#define CONTACTOR_V_HOLD        5 // voltage to keep contactor closed once it's closed
#define CONTACTOR_WAIT_TIME     3000 // how long to wait for amps to fall below CONTACTOR_WAIT_AMPS
#define CONTACTOR_WAIT_AMPS     3    // before opening contactor even though there's current
#define ALERT_VOLTAGE   (3.0*28) // battery warning light or something below this voltage while running

float hv_batt        = 0;
float hv_precharge   = 0;
float lv_sense       = 0;
float contactor_coil = 0; // voltage calculated at contactor coil
float battery_amps   = 0;
int contactor_pwm    = 0; // PWM setting of contactor coil
uint32_t battery_amps_adder; // global for printing raw ADC value accurately

void setup () {
  digitalWrite(POWER_BUTTON,HIGH); // enable pull-up function on power button pin
  pinMode(DCDC_ENABLE_PIN,OUTPUT);
  pinMode(PRECHARGE_PIN  ,OUTPUT);
  pinMode(CONTACTOR_PIN  ,OUTPUT);
  Serial.begin(BAUDRATE);
  Serial.println("https://github.com/jerkey/precharger");
  setPwmFrequency(CONTACTOR_PIN,64); // 31,250 ÷ 64 is audible, but necessary until better transistor gate drive happens
}

void loop () {
  getAnalogs();
  printDisplays();
  while (Serial.available() > 0) handleSerial();
  state_machine();
}

void getAnalogs() {
  hv_batt        = analogRead(HV_BATT) / HV_DIVISOR;
  hv_precharge   = analogRead(HV_PRECHARGE) / HV_DIVISOR;
  lv_sense       = analogRead(LV_SENSE) / LV_DIVISOR;
  contactor_coil = hv_batt * contactor_pwm / 274.0; //analogRead(CONTACTOR_COIL) / CONTACTOR_DIVISOR;
  battery_amps_adder = 0;
  for (int i=0; i<OVERSAMPLES; i++) battery_amps_adder += analogRead(BATTERY_AMPS);
  battery_amps   = (((float)battery_amps_adder/OVERSAMPLES)-BATTERY_AMPS_ZERO) / BATTERY_AMPS_DIVISOR;
}

void printDisplays() {
  static uint32_t lastPrintDisplaysTime = 0;
  if (millis() - lastPrintDisplaysTime > 500) {
    lastPrintDisplaysTime = millis();
    Serial.print("mode: ");
    Serial.print(mode);
    Serial.print("\thv_batt: ");
    Serial.print(hv_batt);
    Serial.print("\thv_precharge: ");
    Serial.print(hv_precharge);
    Serial.print("\tlv_sense: ");
    Serial.print(lv_sense);
    Serial.print("\tcontactor_coil: ");
    Serial.print(contactor_coil);
    Serial.print("V\tbattery_amps: ");
    Serial.print(battery_amps);
    Serial.println(" ("+String((float)battery_amps_adder/OVERSAMPLES)+")");
  }
}

void handleSerial() {
  char inChar = Serial.read(); // read a char
  if (inChar == 'R'){
    Serial.println("RESET mode TO ZERO");
    set_mode(0);
  } else if (inChar == 'C') { // set contactor PWM
    int inInt = Serial.parseInt(); // look for the next valid integer in the incoming serial stream:
    if (inInt < 256) {
      Serial.println(inInt);
      setContactorPwm(inInt);
    }
  } else if (inChar == 'P'){
    digitalWrite(PRECHARGE_PIN,!digitalRead(PRECHARGE_PIN));
    Serial.println("PRECHARGE_PIN: "+String(digitalRead(PRECHARGE_PIN)));
  } else if (inChar == 'D'){
    digitalWrite(DCDC_ENABLE_PIN,!digitalRead(DCDC_ENABLE_PIN));
    Serial.println("DCDC_ENABLE_PIN: "+String(digitalRead(DCDC_ENABLE_PIN)));
  } else if ((inChar >= 'a')&&(inChar <= 'z')) {
    delay(300); // wait for the user to press the same key again
    if (Serial.available() && inChar == Serial.read()) { // only if the same char pressed twice rapidly
      byte pwmValue = constrain((inChar - 97) * 11, 0, 255);
      Serial.println(pwmValue);
      setContactorPwm(pwmValue);
    } else {
      printHelp();
    }
  } else {
    printHelp();
  }
}

void setContactorVoltage(float desiredVoltage) {
  int desired_contactor_pwm = constrain(274.0/hv_batt*desiredVoltage,0,50);
  if (abs(desired_contactor_pwm - contactor_pwm) > 1) {
    Serial.println("setting contactor_pwm to "+String(desired_contactor_pwm));
    setContactorPwm(desired_contactor_pwm);
  }
}

void setContactorPwm(int pwm) {
  analogWrite(CONTACTOR_PIN,pwm);
  contactor_pwm = pwm;
}

void printHelp() {
  Serial.println("(R)eset mode=0, (P)recharge toggle, (D)CDC toggle, C### to enter PWMval, aa - zz 0 to 255");
}

void set_mode(int mode_to_set) {
  mode = mode_to_set;
  last_mode_change = millis();
  Serial.println("Switching to mode "+String(mode));
  state_machine(); // call the function right away
}

void state_machine() {
  switch (mode) {
    case MODE_OFF: mode_off(); break;
    case MODE_PRECHARGE: mode_precharge(); break;
    case MODE_CLOSING: mode_closing(); break;
    case MODE_ON: mode_on(); break;
    default: return;
  }
}

void mode_off() {
  digitalWrite(DCDC_ENABLE_PIN,LOW); // turn off DCDC converter and thus motor controller
  digitalWrite(PRECHARGE_PIN,LOW); // turn off precharging
  if (millis() - last_mode_change < CONTACTOR_WAIT_TIME) { // wait for current draw to hit zero
    if (abs(battery_amps) < CONTACTOR_WAIT_AMPS) setContactorPwm(0); // turn off contactor
  } else if (contactor_pwm != 0) { // more than n seconds since mode change
    setContactorPwm(0); // turn off contactor even if there's current across it
    Serial.println("ALERT: opening contactor with "+String(battery_amps)+" amps across it!");
  }
  if ((digitalRead(POWER_BUTTON) == 0) && (millis() - last_mode_change > 3000)){ // power button pressed
    if (hv_batt < MIN_TURNON_VOLTAGE) { // can't turn on if voltage is too low
      Serial.println("ERROR: attempt to turn on but voltage is too low");
    } else {
      Serial.println("ALERT! power button pressed, turning on!");
      set_mode(MODE_PRECHARGE);
    }
  }
}

void mode_precharge() {
  digitalWrite(PRECHARGE_PIN,HIGH); // turn on precharging
  if (millis() - last_mode_change > 1000) {
    if (hv_batt - hv_precharge < 5) {
      set_mode(MODE_CLOSING);
    } else {
      Serial.println("Failed to precharge in time!");
      set_mode(MODE_OFF);
    }
  }
}

void mode_closing() {
  if (contactor_pwm == 0) {
    if (hv_batt - hv_precharge < 5) {
      setContactorVoltage(CONTACTOR_V_LATCH);
    } else {
      Serial.println("ERROR: mode_closing() called but not precharged");
    }
  } else if (millis() - last_mode_change > 1000) { // it's been a second fully clicked
    if (hv_batt - hv_precharge < 1) { // should be no voltage across contactor!
      setContactorVoltage(CONTACTOR_V_HOLD);
      set_mode(MODE_ON);
    } else {
      Serial.println("ERROR: more than 1 volt across contactor, abort closing!");
      set_mode(MODE_OFF);
    }
  }
}

void mode_on() {
  setContactorVoltage(CONTACTOR_V_HOLD); // adjust contactor PWM as necessary
  digitalWrite(DCDC_ENABLE_PIN,HIGH);
  if (hv_batt < ALERT_VOLTAGE) {
    Serial.print("#");
  }
  if (battery_amps > MAX_BATTERY_AMPS) { // in the event of an extreme problem!
    Serial.println("DANGER! TOO MUCH AMPERAGE ACROSS CONTACTOR!");
    set_mode(MODE_OFF);
  }
  if (hv_batt - hv_precharge > 5) { // should be no voltage across contactor!
    Serial.println("DANGER! VOLTAGE SEEN ACROSS CONTACTOR!");
    set_mode(MODE_OFF);
  }
  if ((digitalRead(POWER_BUTTON) == 0) && (millis() - last_mode_change > 3000)){ // power button pressed
    Serial.println("ALERT! power button pressed, turning off!");
    set_mode(MODE_OFF);
  }
}

// https://playground.arduino.cc/Code/PwmFrequency/
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
