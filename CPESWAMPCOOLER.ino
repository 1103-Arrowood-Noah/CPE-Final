/*
 * CPE 301 Final Project
 * Noah Arrowood, Kenny Cardinalli
 * 
 * Pin Connections:
 * - Water Sensor: A0
 * - DHT11: Digital Pin 7
 * - LCD: Pins 12, 13, 30, 32, 34, 36
 * - RTC: I2C (SDA/SCL - Pins 20/21)
 * - Stepper Motor: Pins 8, 9, 10, 11
 * - Fan Motor: Pin 5 
 * - Yellow LED: Pin 22
 * - Green LED: Pin 24
 * - Blue LED: Pin 26
 * - Red LED: Pin 28
 * - Start Button: Pin 2
 * - Stop Button: Pin 3
 * - Reset Button: Pin 19
 */

#include <LiquidCrystal.h>
#include <EduIntro.h>
#include <RTClib.h>
#include <Stepper.h>

// DHT initialization
DHT11 dht11(7);

// LCD initialization
LiquidCrystal lcd(12, 13, 30, 32, 34, 36);

// RTC initialization
RTC_DS1307 rtc;

// Stepper motor initialization
#define STEPS 200
Stepper stepper(STEPS, 8, 9, 10, 11);

// Pin assignments
#define WATER_SENSOR A0
#define FAN_MOTOR 5
#define START_BUTTON 2
#define STOP_BUTTON 3
#define RESET_BUTTON 19

// LED assignements
#define YELLOW_LED 22
#define GREEN_LED 24
#define BLUE_LED 26
#define RED_LED 28

// Potentiometer (fan vent)
#define VENT_CONTROL A1

// Thresholds
#define WATER_THRESHOLD 100  
#define TEMP_THRESHOLD 25.0

// States
enum State {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR_STATE
};

// Global variables
volatile State currentState = DISABLED;
volatile bool stateChanged = false;
unsigned long lastTempUpdate = 0;
const unsigned long tempUpdateInterval = 60000; // 1 minute
int lastVentPosition = 0;
unsigned long lastStateChangeTime = 0;

// Debouncing 
volatile unsigned long lastStartPress = 0;
volatile unsigned long lastStopPress = 0;
volatile unsigned long lastResetPress = 0;
const unsigned long debounceDelay = 500;
const unsigned long stateChangeProtection = 1000;

// Port manipulation macros
volatile uint8_t* port_a = (uint8_t*) 0x22;
volatile uint8_t* ddr_a = (uint8_t*) 0x21;
volatile uint8_t* pin_a = (uint8_t*) 0x20;

// ADC
volatile uint8_t* my_ADMUX = (uint8_t*) 0x7C;
volatile uint8_t* my_ADCSRB = (uint8_t*) 0x7B;
volatile uint8_t* my_ADCSRA = (uint8_t*) 0x7A;
volatile uint16_t* my_ADC_DATA = (uint16_t*) 0x78;

// Timer
volatile uint8_t* my_TCCR3A = (uint8_t*) 0x90;
volatile uint8_t* my_TCCR3B = (uint8_t*) 0x91;
volatile uint16_t* my_OCR3A = (uint16_t*) 0x98;

// UART
volatile uint8_t* my_UCSR0A = (uint8_t*) 0xC0;
volatile uint8_t* my_UCSR0B = (uint8_t*) 0xC1;
volatile uint8_t* my_UCSR0C = (uint8_t*) 0xC2;
volatile uint16_t* my_UBRR0 = (uint16_t*) 0xC4;
volatile uint8_t* my_UDR0 = (uint8_t*) 0xC6;

void setup() {
  // Initialize UART
  U0Init(9600);
  
  // Initialize ADC
  adc_init();
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.print("Initializing...");
  
  // Initialize RTC
  if (!rtc.begin()) {
    lcd.clear();
    lcd.print("RTC Error!");
    while (1);
  }
  
  if (!rtc.isrunning()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // Set LED as output
  setupLEDs();
  
  // Set buttons as inputs
  setupButtons();
  
  // Fan motor pin setup
  *ddr_a |= (1 << 5); // Set pin 5 as output
  
  // Stepper motor setup
  stepper.setSpeed(60);
  
  // Interrupt setup
  attachInterrupt(digitalPinToInterrupt(START_BUTTON), startISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RESET_BUTTON), resetISR, RISING);
  
  // Set initial state as disabled
  currentState = DISABLED;
  updateLEDs();
  
  lcd.clear();
  lcd.print("System Ready");
  delay(1000);
  
  reportStateChange();
}

void loop() {
  if (stateChanged) {
    reportStateChange();
    stateChanged = false;
  }
  
  // Manual polling for stop button
  checkStopButtonManual();
  
  switch (currentState) {
    case DISABLED:
      handleDisabledState();
      break;
      
    case IDLE:
      handleIdleState();
      break;
      
    case RUNNING:
      handleRunningState();
      break;
      
    case ERROR_STATE:
      handleErrorState();
      break;
  }
  
  // Vent control for all states but disabled
  if (currentState != DISABLED) {
    handleVentControl();
  }
  
  delay(100);
}

// ISR
void startISR() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastStartPress < debounceDelay) {
    return;
  }
  
  if (currentTime - lastStateChangeTime < stateChangeProtection) {
    return;
  }
  
  lastStartPress = currentTime;
  
  if (currentState == DISABLED) {
    currentState = IDLE;
    stateChanged = true;
  }
}

void resetISR() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastResetPress < debounceDelay) {
    return;
  }
  
  if (currentTime - lastStateChangeTime < stateChangeProtection) {
    return;
  }
  
  lastResetPress = currentTime;
  
  if (currentState == ERROR_STATE) {
    if (readWaterLevel() > WATER_THRESHOLD) {
      currentState = IDLE;
      stateChanged = true;
    }
  }
}

// States
void handleDisabledState() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    lcd.clear();
    lcd.print("DISABLED");
    lcd.setCursor(0, 1);
    lcd.print("Press Start");
  }
}

void handleIdleState() {
  int waterLevel = readWaterLevel();
  
  if (waterLevel < WATER_THRESHOLD) {
    currentState = ERROR_STATE;
    stateChanged = true;
    return;
  }
  
  updateTempHumidity();
  
  dht11.update();
  float temp = dht11.readCelsius();
  
  if (temp > TEMP_THRESHOLD) {
    currentState = RUNNING;
    stateChanged = true;
    startFan();
  }
}

void handleRunningState() {
  int waterLevel = readWaterLevel();
  
  if (waterLevel < WATER_THRESHOLD) {
    stopFan();
    currentState = ERROR_STATE;
    stateChanged = true;
    return;
  }
  
  updateTempHumidity();
  
  dht11.update();
  float temp = dht11.readCelsius();
  
  if (temp <= TEMP_THRESHOLD) {
    stopFan();
    currentState = IDLE;
    stateChanged = true;
  }
}

void handleErrorState() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 1000) {
    lastUpdate = millis();
    lcd.clear();
    lcd.print("ERROR!");
    lcd.setCursor(0, 1);
    lcd.print("Water Level Low");
  }
}


void setupLEDs() {
  *port_a &= 0x00;
  *(port_a + 1) |= (1 << 0) | (1 << 2) | (1 << 4) | (1 << 6);
}

void setupButtons() {
  volatile uint8_t* ddr_e = (uint8_t*) 0x2D;
  *ddr_e &= ~((1 << 4) | (1 << 5));
  
  volatile uint8_t* ddr_d = (uint8_t*) 0x2A;
  *ddr_d &= ~(1 << 2);
}

void updateLEDs() {
  *port_a &= ~((1 << 0) | (1 << 2) | (1 << 4) | (1 << 6));
  
  switch (currentState) {
    case DISABLED:
      *port_a |= (1 << 0);
      break;
    case IDLE:
      *port_a |= (1 << 2);
      break;
    case RUNNING:
      *port_a |= (1 << 4);
      break;
    case ERROR_STATE:
      *port_a |= (1 << 6);
      break;
  }
}

void updateTempHumidity() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - lastTempUpdate >= tempUpdateInterval) {
    lastTempUpdate = currentMillis;
    
    dht11.update();
    float humidity = dht11.readHumidity();
    float temp = dht11.readCelsius();
    
    lcd.clear();
    lcd.print("Temp: ");
    lcd.print(temp);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(humidity);
    lcd.print("%");
  }
}

int readWaterLevel() {
  return adc_read(0);
}

void handleVentControl() {
  int potValue = adc_read(1);
  int ventPosition = map(potValue, 0, 1023, -100, 100);
  
  if (abs(ventPosition - lastVentPosition) > 10) {
    int steps = ventPosition - lastVentPosition;
    stepper.step(steps);
    lastVentPosition = ventPosition;
    reportVentChange(ventPosition);
  }
}

void startFan() {
  analogWrite(FAN_MOTOR, 255);
  U0putstr("Fan Motor: ON\n");
}

void stopFan() {
  analogWrite(FAN_MOTOR, 0);
  U0putstr("Fan Motor: OFF\n");
}

void checkStopButtonManual() {
  volatile uint8_t* pin_e = (uint8_t*) 0x2C;
  static bool lastButtonState = false;
  static unsigned long lastPressTime = 0;
  
  bool currentButtonState = (*pin_e & (1 << 5)) != 0;
  
  if (currentButtonState && !lastButtonState) {
    if (millis() - lastPressTime > 500) {
      lastPressTime = millis();
      
      if (millis() - lastStateChangeTime > stateChangeProtection) {
        if (currentState != DISABLED) {
          if (currentState == RUNNING) {
            stopFan();
          }
          currentState = DISABLED;
          stateChanged = true;
        }
      }
    }
  }
  
  lastButtonState = currentButtonState;
}

void reportStateChange() {
  DateTime now = rtc.now();
  
  lastStateChangeTime = millis();
  
  U0putstr("\n=== State Change ===\n");
  U0putstr("Time: ");
  printDateTime(now);
  U0putstr("New State: ");
  
  lcd.clear();
  
  switch (currentState) {
    case DISABLED:
      U0putstr("DISABLED\n");
      lcd.print("DISABLED");
      lcd.setCursor(0, 1);
      lcd.print("Press Start");
      break;
    case IDLE:
      U0putstr("IDLE\n");
      lcd.print("IDLE");
      lcd.setCursor(0, 1);
      lcd.print("Monitoring...");
      lastTempUpdate = 0;
      break;
    case RUNNING:
      U0putstr("RUNNING\n");
      lcd.print("RUNNING");
      lcd.setCursor(0, 1);
      lcd.print("Fan ON");
      lastTempUpdate = 0;
      break;
    case ERROR_STATE:
      U0putstr("ERROR\n");
      lcd.print("ERROR!");
      lcd.setCursor(0, 1);
      lcd.print("Water Level Low");
      break;
  }
  
  updateLEDs();
}

void reportVentChange(int position) {
  DateTime now = rtc.now();
  
  U0putstr("Vent Position Changed: ");
  char buffer[10];
  sprintf(buffer, "%d", position);
  U0putstr(buffer);
  U0putstr(" at ");
  printDateTime(now);
}

void printDateTime(DateTime dt) {
  char buffer[30];
  sprintf(buffer, "%02d/%02d/%04d %02d:%02d:%02d\n", 
          dt.month(), dt.day(), dt.year(),
          dt.hour(), dt.minute(), dt.second());
  U0putstr(buffer);
}

// ADC
void adc_init() {
  *my_ADCSRA |= (1 << 7);
  *my_ADCSRA &= ~(1 << 5);
  *my_ADCSRA |= (1 << 2) | (1 << 1) | (1 << 0);
  *my_ADCSRB &= ~((1 << 3) | (1 << 2) | (1 << 1) | (1 << 0));
  *my_ADMUX |= (1 << 6);
  *my_ADMUX &= ~(1 << 7);
}

unsigned int adc_read(unsigned char channel) {
  *my_ADMUX &= 0xF0;
  *my_ADMUX |= (channel & 0x0F);
  *my_ADCSRA |= (1 << 6);
  while ((*my_ADCSRA & (1 << 6)) != 0);
  return *my_ADC_DATA;
}

// UART
void U0Init(unsigned long baud) {
  unsigned long FCPU = 16000000;
  unsigned int tbaud = (FCPU / 16 / baud - 1);
  *my_UCSR0A = 0x20;
  *my_UCSR0B = 0x18;
  *my_UCSR0C = 0x06;
  *my_UBRR0 = tbaud;
}

unsigned char U0kbhit() {
  return (*my_UCSR0A & (1 << 7));
}

unsigned char U0getchar() {
  while (!U0kbhit());
  return *my_UDR0;
}

void U0putchar(unsigned char c) {
  while ((*my_UCSR0A & (1 << 5)) == 0);
  *my_UDR0 = c;
}

void U0putstr(const char* str) {
  while (*str) {
    U0putchar(*str++);
  }
}
