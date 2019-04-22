#define BAUDRATE        9600

#define PRECHARGE_PIN   13
#define DCDC_ENABLE_PIN 12
#define CONTACTOR_PIN   10

#define HV_BATT         A0
#define HV_PRECHARGE    A1
#define HV_DIVISOR      (1023./103900.*3900./5.) // ADC / resistors total * shunt resistor / AREF
#define LV_SENSE        A2
#define LV_DIVISOR      (1023./8810.*2000./5.) // ADC / resistors total * shunt resistor / AREF
#define CONTACTOR_COIL  A3 // series resistor on contactor coil
#define CONTACTOR_DIVISOR  (1023.) // 5Ω resistor, 1A==5V at ADC
#define BATTERY_AMPS    A4 // current sensor inside Zero battery contactor
#define BATTERY_AMPS_DIVISOR    1.0 // update later

float hv_batt        = 0;
float hv_precharge   = 0;
float lv_sense       = 0;
float contactor_coil = 0;
float battery_amps   = 0;



void setup () {
  pinMode(DCDC_ENABLE_PIN,OUTPUT);
  pinMode(PRECHARGE_PIN  ,OUTPUT);
  pinMode(CONTACTOR_PIN  ,OUTPUT);
  Serial.begin(BAUDRATE);
  Serial.println("https://github.com/jerkey/precharger");
  setPwmFrequency(CONTACTOR_PIN,64); // 31,250 ÷ 1
}

void loop () {
  getAnalogs();
  printDisplays();
  while (Serial.available() > 0) handleSerial();
}

void getAnalogs() {
  hv_batt        = analogRead(HV_BATT) / HV_DIVISOR;
  hv_precharge   = analogRead(HV_PRECHARGE) / HV_DIVISOR;
  lv_sense       = analogRead(LV_SENSE) / LV_DIVISOR;
  contactor_coil = analogRead(CONTACTOR_COIL) / CONTACTOR_DIVISOR;
  battery_amps   = analogRead(BATTERY_AMPS) / BATTERY_AMPS_DIVISOR;
}

void printDisplays() {
  static uint32_t lastPrintDisplaysTime = 0;
  if (millis() - lastPrintDisplaysTime > 500) {
    lastPrintDisplaysTime = millis();
    Serial.print("hv_batt: ");
    Serial.print(hv_batt);
    Serial.print("\thv_precharge: ");
    Serial.print(hv_precharge);
    Serial.print("\tlv_sense: ");
    Serial.print(lv_sense);
    Serial.print("\tcontactor_coil: ");
    Serial.print(contactor_coil);
    Serial.print("\tbattery_amps: ");
    Serial.println(battery_amps);
  }
}

void handleSerial() {
  char inChar = Serial.read(); // read a char
  if (inChar == 'C') { // set contactor PWM
    int inInt = Serial.parseInt(); // look for the next valid integer in the incoming serial stream:
    if (inInt < 256) {
      Serial.println(inInt);
      analogWrite(CONTACTOR_PIN,inInt);
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
      analogWrite(CONTACTOR_PIN,pwmValue);
    } else {
      printHelp();
    }
  } else {
    printHelp();
  }
}

void printHelp() {
  Serial.println("P toggle Precharge, D toggle DCDC, C### to enter PWMval, aa - zz 0 to 255");
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
