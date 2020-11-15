/*
  Title:    Raspbery Pi Timelapse Photography
  Date:     November 12, 2020
  Author:   Adam Garbo

  Description:
  - This code is responsible for power management of the Raspberry Pi 4 (RPi).
  - It makes use of rolling RTC alarms to power on the RPi at set intervals
  in order to capture a photo.
  - Added functionality includes enabling the ATmega328p internal Watchdog
  Timer (WDT).

  Components:
  - Raspberry Pi 4B
  - Raspberry Pi HQ Camera
  - Sleepy Pi 2

  Comments:
  - This code

  To test on the RPi without power cycling and using the Arduino IDE
  to view the debug messages, comment out the following lines:

  //SleepyPi.enablePiPower(true);
  //SleepyPi.piShutdown();
  //SleepyPi.enableExtPower(false);

  Also either fit the Power Jumper or enable self-power:
  http://spellfoundry.com/sleepy-pi/programming-arduino-ide/

  Shutdown current threshold in mA.
  when the RPi is below this, it is "shutdown"
  This will vary from RPi model to RPi model
  and you will need to fine tune it for each RPi
*/

// Libraries
//#include <LowPower.h>   // https://github.com/rocketscream/Low-Power
#include <avr/interrupt.h>  // https://www.nongnu.org/avr-libc/user-manual/group__avr__interrupts.html
#include <avr/power.h>      // https://www.nongnu.org/avr-libc/user-manual/group__avr__power.html
#include <avr/sleep.h>      // https://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
#include <avr/wdt.h>        // https://www.nongnu.org/avr-libc/user-manual/group__avr__watchdog.html
#include <PCF8523.h>        // https://github.com/SpellFoundry/PCF8523
#include <SleepyPi2.h>      // https://github.com/SpellFoundry/SleepyPi2
#include <TimeLib.h>        // https://github.com/PaulStoffregen/Time
#include <Wire.h>

// Defined constants
#define DEBUG     true
#define LED_PIN   13

// Object instantiations
tmElements_t tm;

// User defined global variable declarations
unsigned long awakeTime         = 15000; // Max RPi awake time in milliseconds (ms)
unsigned int  currentThreshold  = 110;  // Shutdown current threshold in milliamps (mA)

// Global variable and constant declarations
volatile bool alarmFlag         = false;  // Flag for alarm interrupt service routine
volatile bool watchdogFlag      = false;  // Flag for Watchdog Timer interrupt service routine
volatile int  watchdogCounter   = 0;      // Watchdog Timer interrupt counter

bool          ledState        = LOW;    // Flag to toggle LED in blinkLed() function
bool          piPowerFlag     = true;   // Flag to indicate if Raspberry Pi is running
byte          alarmMinutes    = 5;
byte          alarmHours      = 0;
byte          alarmDay        = 0;
unsigned long previousMillis  = 0;      // Global millis() timer

// RTC alarm interrupt service routine
void alarmIsr() {
  alarmFlag = true;
}

// Setup
void setup() {

  // Pin assignments
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);   // Switch off LED

  Wire.begin();

  Serial.begin(9600); // Begin serial at 9600 baud
  delay(5000); // Delay to allow user to open Serial Monitor

  // Configure Sleepy Pi 2
  SleepyPi.rtcInit(true); // Initialize PCF8523 RTC
  SleepyPi.enablePiPower(false); // Disable power to Raspberry Pi
  SleepyPi.enableExtPower(false); // Disable the external expansion power

  // Print the RTC's current date and time
  printDateTime();

  // Configure the Watchdog Timer
  configureWdt();

  // Set initial RTC alarm to hour rollover
  //SleepyPi.setAlarm(0, 0);

  // Set the RTC alarm
  DateTime now = SleepyPi.readTime();
  SleepyPi.setAlarm((now.day() + alarmDay) % 31,
                    (now.hour() + alarmHours) % 24,
                    (now.minute() + 1) % 60);

#if DEBUG
  //PrintRtcRegisters(); // Debugging
#endif

  goToSleep();
}

// Loop
void loop() {

  if (alarmFlag) {

    // Disable external pin interrupt on wake up pin
    detachInterrupt(0);

    // Missing description
    SleepyPi.ackAlarm();

    // Set the RTC alarm
    DateTime now = SleepyPi.readTime();
    SleepyPi.setAlarm((now.day() + alarmDay) % 31,
                      (now.hour() + alarmHours) % 24,
                      (now.minute() + alarmMinutes % 60));

    // Enable power to Raspberry Pi
    SleepyPi.enablePiPower(true);

    digitalWrite(LED_PIN, HIGH); // Turn on LED

    Serial.print("Alarm trigger: "); printDateTime();

    digitalWrite(LED_PIN, LOW);   // Turn off LED

    // Do something on the Rpi here
    // Example: Take a Picture
    // This is the time that the Rpi will be "awake" for

    // Manage the RPi shutdown
    // -----------------------
    // Maximum time limit to stay awake if it hasn't already shut down
    delay(10000); // Delay to allow RPi to boot up (add delay to MAX_RPI_TIME_TO_STAY_AWAKE_MS)

    unsigned long loopStartTime = millis();

    // Check the current draw of the RPi to deteremine if is is running
    piPowerFlag = SleepyPi.checkPiStatus(currentThreshold, false);

    // Wait until either the RPi shuts down through user action (i.e sudo shutdown -h now)
    // or the timer expires
    while (piPowerFlag && millis() - loopStartTime < awakeTime) {

      // Monitor the current draw of the RPi to deteremine if is is running
      piPowerFlag = SleepyPi.checkPiStatus(currentThreshold, false);
      petDog();
      delay(1000);

      // For debugging issue a "sudo shutdown -h now" command and monitor the current draw
      // Use this to work out a shutdown current threshold
      // Remember that the Timer will backstop this method and cut the power on time out
      Serial.print(piPowerFlag);
      Serial.print(" ");
      Serial.print(SleepyPi.rpiCurrent());
      Serial.println(F(" mA"));
    }

    // Begin shutdown
    if (piPowerFlag) {
      SleepyPi.piShutdown(); // Issue a shutdown command to the Raspberry Pi
      SleepyPi.enableExtPower(false); // Disable the External Expansion power
    }
    else {
      // If RPi is already shutdown, disable the power
      SleepyPi.enablePiPower(false); // Disale power to the Raspberry Pi
      SleepyPi.enableExtPower(false); // Disable the external expansion power
    }
    alarmFlag = false;
  }

  // Check for watchdog interrupt
  if (watchdogFlag) {
    petDog();
  }

  blinkLed(1, 100);
  goToSleep();
}

// Enable sleep and await RTC alarm interrupt
void goToSleep() {

#if DEBUG
  Serial.println(F("Entering deep sleep..."));
  Serial.flush();
#endif // DEBUG

  // Allow alarm to trigger interrupt on falling edge
  attachInterrupt(0, alarmIsr, FALLING);

  // Enable RTC alarm
  SleepyPi.enableWakeupAlarm(true);

  byte adcsra = ADCSRA;   // Save ADC Control and Status Register A (ADCSRA)
  ADCSRA = 0;             // Disable the ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();                  // Disable all interrupts
  sleep_enable();         // Set the SE (sleep enable) bit
  wdt_reset();            // "Pet" the dog
  sleep_bod_disable();    // Disable BOD before going to sleep
  sei();                  // Enable all interrupts
  sleep_cpu();            // Put the device into sleep mode. Must be executed within 3 clock cycles in order to disable BOD
  sleep_disable();        // Clear the SE (sleep enable) bit. Program will execute from this point once awake
  ADCSRA = adcsra;        // Restore ADCSRA

  /*
    Sleep and await RTC interrupt
  */
}

// Print the RTC's current date and time
void printDateTime() {
  DateTime now = SleepyPi.readTime();
  char dateTimeBuffer[25];
  sprintf(dateTimeBuffer, "%d-%02d-%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(),
          now.hour(), now.minute(), now.second());
  Serial.println(dateTimeBuffer);
}

// Print the RTC's current date and time
void printAlarm() {
  char alarmBuffer[25];
  //sprintf(alarmBuffer, "%02d %02d:%02d",
  //getAlarm(days), SleepyPi.getAlarm(hours), SleepyPi.getAlarm(minutes));
  Serial.println(alarmBuffer);
}

// Blink LED (non-blocking)
void blinkLed(byte ledFlashes, unsigned int ledDelay) {

  pinMode(LED_PIN, OUTPUT);
  byte i = 0;

  while (i < ledFlashes * 2) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > ledDelay) {
      previousMillis = currentMillis;
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
      i++;
    }
  }
  digitalWrite(LED_PIN, LOW);
}

// Configure Watchdog Timer
void configureWdt() {
  cli();                                // Disable all interrupts
  wdt_reset();                          // Reset the Watchdog Timer
  MCUSR = 0;                            // Clear Reset Flags of MCU Status Register (MCUSR)
  WDTCSR |= (1 << WDCE) | (1 << WDE);   // Start timed sequence allowing alterations to Watchdog Timer Control Register (WDTCSR)
  WDTCSR = (1 << WDIE) | (0 << WDE) |   // Set Watchdog Interrupt Enable (WDIE) and clear Watchdog System Reset Enable (WDE) and
           (1 << WDP3) | (0 << WDP2) |  // Set Watchdog Timer Prescaler WDP3 and WDP0 to select an 8 s timeout period
           (0 << WDP1) | (1 << WDP0);
  sei();                                // Enable all interrupts
}

// Watchdog Timer interrupt service routine
ISR(WDT_vect) {

  watchdogFlag = true; // Set the watchdog flag
  watchdogCounter++; // Increment watchdog interrupt counter

  // Perform system reset after 15 watchdog interrupts (should not occur)
  if (watchdogCounter < 25 ) {
    wdt_reset(); // "Pet" the dog
  }
  else {
    // Enable WDT System Reset Mode
    MCUSR = 0;                            // Clear Reset Flags of MCU Status Register (MCUSR)
    WDTCSR |= (1 << WDCE) | (1 << WDE);   // Start timed sequence allowing changes to Watchdog Timer Control Register (WDTCSR)
    WDTCSR = (0 << WDIE) | (1 << WDE) |   // Clear Watchdog Interrupt Enable (WDIE) and set Watchdog System Reset Enable (WDE) and
             (0 << WDP3) | (0 << WDP2) |  // Clear Watchdog Timer Prescaler WDP3, WDP2, WDP1 and WDP0 to select a 16 ms timeout period
             (0 << WDP1) | (0 << WDP0);
    while (1);                            // System reset will occur after 16 ms
  }
}

void petDog() {
  wdt_reset(); // "Pet" the dog
  Serial.print(F("Watchdog interrupt: ")); Serial.println(watchdogCounter);
  watchdogFlag = false; // Clear watchdog flag
  watchdogCounter = 0; // Reset watchdog interrupt counter
}
void printRtcRegisters(void) {

  uint8_t reg_value;
  reg_value = SleepyPi.rtcReadReg(PCF8523_CONTROL_1);
  Serial.print("Control 1: 0x");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_CONTROL_2);
  Serial.print("Control 2: 0x");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_CONTROL_3);
  Serial.print("Control 3: 0x");
  Serial.println(reg_value, HEX);

  reg_value = SleepyPi.rtcReadReg(PCF8523_SECONDS);
  Serial.print("Seconds: ");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_MINUTES);
  Serial.print("Minutes: ");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_HOURS);
  Serial.print("Hours: ");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_DAYS);
  Serial.print("Days: ");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_WEEKDAYS);
  Serial.print("Week Days: ");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_MONTHS);
  Serial.print("Months: ");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_YEARS);
  Serial.print("Years: ");
  Serial.println(reg_value, HEX);

  reg_value = SleepyPi.rtcReadReg(PCF8523_MINUTE_ALARM);
  Serial.print("Minute Alarm: ");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_HOUR_ALARM);
  Serial.print("Hour Alarm: ");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_DAY_ALARM);
  Serial.print("Day Alarm: ");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_WEEKDAY_ALARM);
  Serial.print("Weekday Alarm: ");
  Serial.println(reg_value, HEX);

  reg_value = SleepyPi.rtcReadReg(PCF8523_OFFSET);
  Serial.print("Offset: 0x");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_TMR_CLKOUT_CTRL);
  Serial.print("TMR_CLKOUT_CTRL: 0x");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_TMR_A_FREQ_CTRL);
  Serial.print("TMR_A_FREQ_CTRL: 0x");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_TMR_A_REG);
  Serial.print("TMR_A_REG: 0x");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_TMR_B_FREQ_CTRL);
  Serial.print("TMR_B_FREQ_CTRL: 0x");
  Serial.println(reg_value, HEX);
  reg_value = SleepyPi.rtcReadReg(PCF8523_TMR_B_REG);
  Serial.print("TMR_B_REG: 0x");
  Serial.println(reg_value, HEX);
}
