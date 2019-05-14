#define BAUDRATE        38400
#define SUMRATE         500 // number of milliseconds between integrating analog values

uint32_t last_mode_change = 0;
enum mode_type { MODE_OFF = 0, MODE_PRECHARGE, MODE_CLOSING, MODE_ON };
int mode = MODE_OFF;
String modeString = "mode?";

#define PRECHARGE_PIN   13
#define DCDC_ENABLE_PIN 12
#define POWER_BUTTON    11
#define CONTACTOR_PIN   10
#define BEEPER_PIN      2

#define HV_BATT         A0
#define HV_PRECHARGE    A1
#define HV_DIVISOR      (1023./103900.*3900./5./1.042) // ADC / resistors total * shunt resistor / AREF / fudge
#define LV_SENSE        A2
#define LV_DIVISOR      (1023./8810.*2000./5.) // ADC / resistors total * shunt resistor / AREF
#define BATTERY_AMPS    A4 // current sensor inside Zero battery contactor
#define BATTERY_AMPS_DIVISOR    1.4 // 4.0A = 515.0
#define MIN_TURNON_VOLTAGE      (2.9*28) // 2.9*28=81.2 below this voltage we just won't turn on
#define MAX_BATTERY_AMPS        500 // this shouldn't happen right?
#define CONTACTOR_V_LATCH       15 // voltage to click contactor shut hard for a second
#define CONTACTOR_V_HOLD        5 // voltage to keep contactor closed once it's closed
#define CONTACTOR_WAIT_TIME     3000 // how long to wait for amps to fall below CONTACTOR_WAIT_AMPS
#define CONTACTOR_WAIT_AMPS     3    // before opening contactor even though there's current
#define ALERT_VOLTAGE   (3.0*28) // battery warning light or something below this voltage while running
#define PRECHARGE_TIMEOUT       10000 // milliseconds to wait before precharge is considered a failure
#define PRECHARGE_MINTIME       200 // milliseconds before precharge could possibly complete
#define PRECHARGE_MIN_RATIO     0.63 // ratio of hv_precharge ÷ hv_batt not to pass during PRECHARGE_MINTIME

float hv_batt        = 0;
float hv_precharge   = 0;
float lv_sense       = 0;
float battery_amps   = 0;
float battery_amps_adc = 0; // for printing and zero cal
float battery_amps_zero = 511; // zero is copied from battery_amps_adc at end of precharge
int contactor_pwm    = 0; // PWM setting of contactor coil
uint32_t hv_batt_adder, hv_precharge_adder, battery_amps_adder; // holds total ADC counts
uint16_t oversamples = 0; // counts how many ADC reads we've added
int32_t watt_seconds_raw = 0; // counts about 7.37*1.4 watt-seconds, max +/- 57.8KWH

void setup () {
  pinMode(BEEPER_PIN,OUTPUT); // beeper
  digitalWrite(POWER_BUTTON,HIGH); // enable pull-up function on power button pin
  pinMode(DCDC_ENABLE_PIN,OUTPUT);
  pinMode(PRECHARGE_PIN  ,OUTPUT);
  pinMode(CONTACTOR_PIN  ,OUTPUT);
  Serial.begin(BAUDRATE);
  Serial.println("https://github.com/jerkey/precharger");
  setPwmFrequency(CONTACTOR_PIN,64); // 31,250 ÷ 64 is audible, but necessary until better transistor gate drive happens
  tone(BEEPER_PIN,500,3000); // three-second tone
}

void loop () {
  getAnalogs();
  sumAndPrintData();
  while (Serial.available() > 0) handleSerial();
  state_machine();
}

void getAnalogs() {
  hv_batt_adder      += analogRead(HV_BATT);
  hv_precharge_adder += analogRead(HV_PRECHARGE);
  battery_amps_adder += analogRead(BATTERY_AMPS);
  oversamples        += 1; // count how many times we've sampled
  if ((hv_batt_adder > 0x7FFFF000) || (hv_precharge_adder > 0x7FFFF000) || (battery_amps_adder > 0x7FFFF000) || (oversamples > 65530)) Serial.println("adc adder overflow!");
}

void sumAndPrintData() {
  static uint32_t lastPrintDisplaysTime = 0;
  uint32_t timeSinceLastSum = millis() - lastPrintDisplaysTime;
  if (timeSinceLastSum > SUMRATE) {
    lastPrintDisplaysTime = millis();
    // adder-based high-accuracy values
    float hv_batt_adc = (float)hv_batt_adder / oversamples; // for power and energy calcs
    hv_batt      = hv_batt_adc / HV_DIVISOR;
    hv_precharge = ((float)hv_precharge_adder / oversamples) / HV_DIVISOR;
    battery_amps_adc = (float)battery_amps_adder / oversamples; // for printing and zero cal
    //lv_sense            = (float)analogRead(LV_SENSE) / LV_DIVISOR;
    if (contactor_pwm == 0) { // contactor is open
      battery_amps = 0; // contactor is open so sensor is just bouncing around
    } else { // contactor is closed
      battery_amps = (battery_amps_adc - battery_amps_zero) / BATTERY_AMPS_DIVISOR;
      watt_seconds_raw += ((battery_amps_adc - battery_amps_zero) * hv_batt_adc * timeSinceLastSum) / 1000.0; // ampsraw * voltsraw * milliseconds / 1000
    }
    oversamples = 0; battery_amps_adder = 0; hv_batt_adder = 0; hv_precharge_adder = 0;
    float kilowatt_hours = watt_seconds_raw / (BATTERY_AMPS_DIVISOR * HV_DIVISOR * 60 * 60 * 1000);

    if (digitalRead(DCDC_ENABLE_PIN)) Serial.print("DC ");
    if (digitalRead(PRECHARGE_PIN)) Serial.print("PRE ");
    Serial.print(modeString);
    Serial.print("\tbat: ");
    Serial.print(hv_batt,1);
    if (contactor_pwm == 0) {
      Serial.print("\tpre: ");
      Serial.print(hv_precharge,1);
    } else {
      Serial.print("\tKW: ");
      Serial.print(hv_batt * battery_amps / 1000.0,1);
      Serial.print("\tamp: ");
      Serial.print(battery_amps,1);
    }
    //Serial.print("\tlv: ");
    //Serial.print(lv_sense,1);
    //Serial.print("\tcon: ");
    //Serial.print(hv_batt * contactor_pwm / 274.0,1);
    Serial.print(" ("+String(battery_amps_adc)+")");
    Serial.print("\tkWh: ");
    Serial.println(kilowatt_hours,1);
  }
}

void handleSerial() {
  char inChar = Serial.read(); // read a char
  if (inChar == '0'){
    set_mode(MODE_OFF);
  } else  if (inChar == '1'){
    set_mode(MODE_PRECHARGE);
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
  Serial.println("0=MODE_OFF, 1=MODE_PRECHARGE, (P)recharge toggle, (D)CDC toggle, C### to enter PWMval, aa - zz 0 to 255");
}

void set_mode(int mode_to_set) {
  mode = mode_to_set;
  last_mode_change = millis();
  Serial.println("Switching to mode "+String(mode));
  state_machine(); // call the function right away
}

void state_machine() {
  switch (mode) {
    case MODE_OFF: modeString = "OFF"; mode_off(); break;
    case MODE_PRECHARGE: modeString = "PRECHARGE"; mode_precharge(); break;
    case MODE_CLOSING: modeString = "CLOSING"; mode_closing(); break;
    case MODE_ON: modeString = "ON "; mode_on(); break;
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
    tone(BEEPER_PIN,500,1000); // one second tone
  }
  if ((digitalRead(POWER_BUTTON) == 0) && (millis() - last_mode_change > 3000)){ // power button pressed
    delay(100); // debounce
    if (digitalRead(POWER_BUTTON) == 1) return; // false button press
    if (hv_batt < MIN_TURNON_VOLTAGE) { // can't turn on if voltage is too low
      Serial.println("ERROR: attempt to turn on but voltage is too low");
      tone(BEEPER_PIN,500,1000); // one second tone
    } else {
      Serial.println("ALERT! power button pressed, turning on!");
      tone(BEEPER_PIN,500,1000); // one second tone
      set_mode(MODE_PRECHARGE);
    }
  }
}

void mode_precharge() {
  digitalWrite(PRECHARGE_PIN,HIGH); // turn on precharging
  if (millis() - last_mode_change > PRECHARGE_MINTIME) {
    if (hv_batt - hv_precharge < 5) {
      battery_amps_zero = battery_amps_adc; // dynamically detected amps zero
      set_mode(MODE_CLOSING);
    } else if (millis() - last_mode_change > PRECHARGE_TIMEOUT) {
      Serial.println("ERROR: Failed to precharge after PRECHARGE_TIMEOUT!");
      tone(BEEPER_PIN,500,1000); // one second tone
      set_mode(MODE_OFF);
    }
  } else {
    if (hv_precharge / hv_batt > PRECHARGE_MIN_RATIO) {
      if (millis() - last_mode_change < 7) {
        Serial.println("ERROR: precharged too fast");
        tone(BEEPER_PIN,1000,500); // half-second high-tone
      }
      // set_mode(MODE_OFF); this is a problem if capacitors don't drain when turned off
    }
  }
}

void mode_closing() {
  if (contactor_pwm == 0) {
    if (hv_batt - hv_precharge < 5) {
      setContactorVoltage(CONTACTOR_V_LATCH);
    } else {
      Serial.println("ERROR: mode_closing() called but not precharged");
      tone(BEEPER_PIN,500,1000); // one second tone
    }
  } else if (millis() - last_mode_change > 1000) { // it's been a second fully clicked
    if (hv_batt - hv_precharge < 1) { // should be no voltage across contactor!
      set_mode(MODE_ON);
    } else {
      Serial.println("ERROR: more than 1 volt across contactor, abort closing!");
      tone(BEEPER_PIN,500,1000); // one second tone
      set_mode(MODE_OFF);
    }
  }
}

void mode_on() {
  setContactorVoltage(CONTACTOR_V_HOLD); // adjust contactor PWM as necessary
  digitalWrite(DCDC_ENABLE_PIN,HIGH);
  if (hv_batt < ALERT_VOLTAGE) {
    tone(BEEPER_PIN,2000,1000); // one second high tone
    Serial.print("#");
  }
  if (battery_amps > MAX_BATTERY_AMPS) { // in the event of an extreme problem!
    Serial.println("DANGER! TOO MUCH AMPERAGE ACROSS CONTACTOR!");
    tone(BEEPER_PIN,500,1000); // one second tone
    set_mode(MODE_OFF);
  }
  if (hv_batt - hv_precharge > 5) { // should be no voltage across contactor!
    Serial.println("DANGER! VOLTAGE SEEN ACROSS CONTACTOR!");
    tone(BEEPER_PIN,500,1000); // one second tone
    set_mode(MODE_OFF);
  }
  if ((digitalRead(POWER_BUTTON) == 0) && (millis() - last_mode_change > 3000)){ // power button pressed
    delay(100); // debounce
    if (digitalRead(POWER_BUTTON) == 1) return; // false button press
    Serial.println("ALERT! power button pressed, turning off!");
    tone(BEEPER_PIN,500,1000); // one second tone
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
