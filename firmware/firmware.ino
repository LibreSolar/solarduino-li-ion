/**
 *  Solarduino Software
 *  Martin Jäger, 2015
 *
 *  Solar charger and integrated LiFePO battery management system
 *  - uC: ATmega 328P with Arduino compatible pin useage
 *  - BMS IC: Texas Instruments bq76920
 *  - MPPT solar charger: Buck converter, max. 8A output current
 *
 *  
 */

//----------------------------------------------------------------------------
// includes

#include <Arduino.h>
#include <Wire.h>     // I2C/TWI (for Battery Management IC)
#include <SPI.h>      // SPI (for Display)
#include "helper.h"   // helper functions

#include <bq769x0.h>    // Library for Texas Instruments bq76920 battery management IC

// Display
#include <dog_1701.h>
#include <font_16x32nums.h>
#include <font_6x8.h>
#include <font_8x16.h>
#include <font_8x8.h>

//----------------------------------------------------------------------------
// definitions

#define PWM_PIN 9           // PWM input of half bridge driver
#define PWM_ENABLE_PIN 8    // enable pin of half bridge driver
#define DISP_CS_PIN    10
#define DISP_CD_PIN    A1
#define DISP_RESET_PIN A0
#define SOLAR_VOLTAGE_PIN A6
#define SOLAR_CURRENT_PIN A7

#define BMS_ALERT_PIN 2     // attached to interrupt INT0
#define BMS_BOOT_PIN 7      // connected to TS1 input
#define BMS_I2C_ADDRESS 0x18

// DC/DC converter settings
//
// The PWM resolution (number of steps between 0 and 100%) is limited by the
// µC frequency and the used PWM mode
// 8 MHz µC frequency and 100 kHz PWM frequency: 80 steps
//
#define PWM_FREQUENCY 100 // kHz
#define PWM_RESOLUTION ((F_CPU/1000)/PWM_FREQUENCY)
#define PWM_MIN (60.0/100.0*PWM_RESOLUTION)     // defines maximum input voltage (D=Vout/Vin)
#define PWM_INC 1                             // PWM increment for MPP tracking

#define MIN_SOLAR_CURRENT 50  // mA    --> if lower, charger is switched off
#define SOLAR_VOLTAGE_OFFSET 2000 // mV  charging switched on if Vsolar > Vbat + offset

// misc
#define ON 1
#define OFF 0
#define NUM_AVG 10    // number of ADC values to read for averaging

#define TURN_ON_MOSFETS digitalWrite(PWM_ENABLE_PIN, HIGH)      // enable MOSFET driver
#define TURN_OFF_MOSFETS digitalWrite(PWM_ENABLE_PIN, LOW)      // disable MOSFET driver

#define CUTOFF_VOLTAGE_CHG 3550  // mV    recommended value for ECC 45 Ah: 3550
#define CUTOFF_VOLTAGE_DSG 3000  // mV    recommended value for ECC 45 Ah: 3000
#define MAX_CHG_CURRENT 8000  // mA       PCB maximum: 8 A

//----------------------------------------------------------------------------
// global variables

dog_1701 DOG;   // display object
bq769x0 BMS(bq76920, BMS_I2C_ADDRESS);    // battery management system object

int pwm_duty = 63;  // PWM duty cycle (= PWM_RESOLUTION - 1 for 100%)
int solar_voltage;      // mV
int solar_current;      // mA

long solar_voltage_total;
long solar_current_total;
int solar_voltage_readings[NUM_AVG];
int solar_current_readings[NUM_AVG];
int i_readings = 0;

unsigned int solar_power_prev = 0;   // unit: 1/100 W
unsigned int solar_power = 0;        // unit: 1/100 W
int pwm_delta = PWM_INC;    // variable used to modify pwm duty cycle for the MPPT algorithm

enum charger_states {STANDBY, CHG_CC, CHG_CV, BAT_ERROR, BALANCING};     // enumerated variable that holds state for charger state machine
int state = BAT_ERROR;


//----------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600);  // start serial for output  

  int err = BMS.begin(BMS_ALERT_PIN, BMS_BOOT_PIN);

  // ToDo: Ensure that these settings are set even in case of initial communication error
  BMS.setTemperatureLimits(-20, 45, 0, 45);
  BMS.setShuntResistorValue(5);
  BMS.setShortCircuitProtection(14000, 200);  // delay in us
  BMS.setOvercurrentChargeProtection(8000, 200);  // delay in ms
  BMS.setOvercurrentDischargeProtection(8000, 320); // delay in ms
  BMS.setCellUndervoltageProtection(2600, 2); // delay in s
  BMS.setCellOvervoltageProtection(3650, 2);  // delay in s

  BMS.setBalancingThresholds(0, 3300, 20);  // minIdleTime_min, minCellV_mV, maxVoltageDiff_mV
  BMS.setIdleCurrentThreshold(100);
  BMS.enableAutoBalancing();
  BMS.enableDischarging();
/*
  if (err > 0) {
    enter_state(BAT_ERROR);
  }
  else {
    enter_state(STANDBY);
  }
  */
  //initPWM();
  // TEST BEGIN
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWM_ENABLE_PIN, OUTPUT);
  digitalWrite(PWM_ENABLE_PIN, HIGH);
  Serial.println("PWM PIN is now highg");
  // TEST END
  
  // display
  DOG.initialize(DISP_CS_PIN, 0, 0, DISP_CD_PIN, DISP_RESET_PIN, DOGS102); // 0,0 = Hardware SPI, EA DOGS102-6 (=102x64 dots)
  //DOG.initialize(DISP_CS_PIN, 11, 13, DISP_CD_PIN, DISP_RESET_PIN, DOGS102); // 0,0 = Hardware SPI, EA DOGS102-6 (=102x64 dots)
  DOG.view(VIEW_BOTTOM);  //default viewing direction

  // initialize variables with zeros
  for (int i = 0; i < NUM_AVG; i++) {
    solar_voltage_readings[i] = 0;
    solar_current_readings[i] = 0;
  }
}

//----------------------------------------------------------------------------
// Sets Timer1 to run PWM output at Arduino pin 9
// 
void initPWM()
{
  // PWM_PIN must be pin 9 (OC1A of ATmega328P)
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWM_ENABLE_PIN, OUTPUT);

  // set mode 14: Fast PWM, TOP stored in ICR1
  TCCR1A = _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12);
  
  // no prescaling
  TCCR1B |= _BV(CS10);

  // disable interrupts for 16-bit register access
  unsigned char sreg;  
  sreg = SREG;  // Save global interrupt flag
  cli();        // Disable interrupts

  // set TOP to define PWM frequency
  ICR1 = PWM_RESOLUTION - 1;
  
  SREG = sreg;  // Restore global interrupt flag

  Serial.print("PWM cycles: ");
  Serial.println(PWM_RESOLUTION);
}

//----------------------------------------------------------------------------
void loop()
{

  BMS.update();  // should be called at least every 250 ms

  update_solar_power();
  //state_machine();
  update_screen();
  
  //print_status();
  
  //BMS.printRegisters();

  // TEST BEGIN
  //TURN_ON_MOSFETS;
  //enter_state(CHG_CC);
  // TEST END
  
  /*
  // PWM testing
  char incoming = 0; 
  if (Serial.available() > 0) {
    // read the incoming byte:
    incoming = Serial.read();

    if (incoming == '+') {
      pwm_duty++;
      set_pwm_duty();
    }
    else if (incoming == '-') {
      pwm_duty--;
      set_pwm_duty();
    }
    
    Serial.print("PWM: ");
    Serial.print(pwm_duty);
    Serial.print("      Solar Power [mA]: ");
    Serial.println(solar_power);
  }

*/
  delay(100);
}

// actions to be performed upon entering a state
void enter_state(int next_state)
{
  switch (next_state)
  {
    case STANDBY:
      Serial.println(F("Enter State: STANDBY"));
      TURN_OFF_MOSFETS; 
      if (BMS.enableDischarging() == false) {
        return;
      }
      break;
    case CHG_CC:
      Serial.println(F("Enter State: CHG_CC"));
      if (BMS.enableCharging() == true)
      {
        //pwm_duty = PWM_RESOLUTION * BMS.getBatteryVoltage() / solar_voltage + 1;  // start with proper PWM duty cycle
        pwm_delta = -PWM_INC;
        set_pwm_duty();
        TURN_ON_MOSFETS;
        Serial.println(F("OK, laden muesste jetzt laufen..."));
      }
      else {
        Serial.println("Laden nicht möglich!");
        return;   // charging not possible
      }
      break;
    case CHG_CV:
      Serial.println("Enter State: CHG_CV");
      // coming from CHG_CC --> DC/DC FETs already on and charging enabled
      break;
    case BAT_ERROR:
      Serial.println("Enter State: BAT_ERROR");
      TURN_OFF_MOSFETS;
      break;
    case BALANCING:
      Serial.println("Enter State: BALANCING");
      TURN_OFF_MOSFETS; 
      break;
    default:
      TURN_OFF_MOSFETS; 
      break;
  }
  state = next_state;
}

/*****************************************************************************
 *  Charger state machine
 *
 *  INIT       Initial state when system boots up. Checks communication to BMS
 *             and turns the CHG and DSG FETs on
 *  STANDBY    Battery can be discharged, but no charging power is available
 *             or battery is fully charged
 *  CHG_CC     Constant current (CC) charging with maximum current possible
 *             (MPPT algorithm is running)
 *  CHG_CV     Constant voltage (CV) charging at maximum cell voltage
 *  BAT_ERROR  An error occured. Charging disabled.
 *  BALANCING  After a period of no load in STANDBY state, balancing is 
 *             started. Goes back to STANDBY as soon as a load is detected.
 */
void state_machine()
{
  int err = 0;
  
  switch (state)
  {
    case STANDBY:
      if (BMS.checkStatus() == 0)
      {
        //Serial.println("Standby");
        if  ((solar_voltage > BMS.getBatteryVoltage() + SOLAR_VOLTAGE_OFFSET)) {      
          enter_state(CHG_CC);
        }
      }
      /*else if (BMS.balancingAllowed() == true)
      {
        enter_state(BALANCING);
      }*/
      break;
      
    case CHG_CC:
      if (BMS.checkStatus() == 0)
      {
        //Serial.println("CHG_CC state still alive");
        if (BMS.getMaxCellVoltage() > CUTOFF_VOLTAGE_CHG)
        {
          enter_state(CHG_CV);
        }/*
        else if (BMS.getBatteryCurrent() > MAX_CHG_CURRENT)
        {
          // increase PV voltage
          pwm_duty--;
          pwm_delta = -PWM_INC;
          set_pwm_duty();
          solar_power_prev = solar_power;
        }
        else // start MPPT
        {
          if (solar_power_prev > solar_power)
          {
            pwm_delta = -pwm_delta;
          }
          pwm_duty += pwm_delta;
          set_pwm_duty();
          solar_power_prev = solar_power;
        }*/      
      }
      else
      {
        enter_state(BAT_ERROR);
      }
      break;
      
    case CHG_CV:
      if (BMS.checkStatus() == 0)
      {
        if (solar_current < MIN_SOLAR_CURRENT)
        {
          enter_state(STANDBY);
          return;
        }
        else {
          if (BMS.getMaxCellVoltage() > CUTOFF_VOLTAGE_CHG)
          {
            // increase input voltage and decrease PV current
            pwm_duty--;
            set_pwm_duty();
          }
          else
          {
            pwm_duty++;
            set_pwm_duty();
          }
        }
      }
      else {
        enter_state(BAT_ERROR);
      }
      break;
      
    case BAT_ERROR:
      err = BMS.checkStatus();
      if (err == 0) {
        enter_state(STANDBY);
      }
      break;
      
    default:
      TURN_OFF_MOSFETS; 
      break;
  }
}

//----------------------------------------------------------------------------
void set_pwm_duty()
{
  if (pwm_duty >= PWM_RESOLUTION - 1)
  {
    pwm_duty = PWM_RESOLUTION - 2;            // 100% not possible because of charge pump
  }
  else if (pwm_duty < PWM_MIN)
  {       
    pwm_duty = PWM_MIN;
  }
  
  // disable interrupts for 16-bit register access
  unsigned char sreg = SREG;
  cli();

  // set
  OCR1A = pwm_duty;
  
  // restore global interrupt flag
  SREG = sreg;                  

  // Compare Output Mode: Fast PWM, non-inverting mode
  TCCR1A |= _BV(COM1A1);
  
  //Serial.print("PWM duty cycle: ");
  //Serial.println((pwm_duty+1.0)/(PWM_RESOLUTION) * 100.0);
}

//----------------------------------------------------------------------------
//create a sample sceen content
void update_screen(void)
{
  char str[20];
  char buf[20];
  
  DOG.clear();  //clear whole display
  //DOG.string(71,0,font_8x16,"DOGS");      //font size 8x16 first page         
  //DOG.rectangle(71,2,127,2,0x03);         //show small line (filled pattern = 0x03), to the left and right of 'DOGL128'            
  
  format_double(solar_voltage/1000.0, 2, buf, 20);
  sprintf(str, "Sol: %sV", buf);
  DOG.string(0,0,font_6x8, str);

  format_double(solar_current/1000.0, 2, buf, 20);
  sprintf(str, "%sA", buf);
  DOG.string(12*6,0,font_6x8, str);
  
  format_double(BMS.getBatteryVoltage()/1000.0, 2, buf, 20);
  sprintf(str, "Bat: %sV", buf);
  DOG.string(0,1,font_6x8, str);

  format_double(BMS.getBatteryCurrent()/1000.0, 2, buf, 20);
  sprintf(str, "%sA", buf);
  DOG.string(12*6,1,font_6x8, str);

  format_double(BMS.getTemperatureDegC(), 1, buf, 20);
  sprintf(str, "Temp: %s C", buf);
  DOG.string(0,2,font_6x8, str);
  
  format_double(solar_power/100.0, 2, buf, 20);
  sprintf(str, "Power: %s W", buf);
  DOG.string(0,3,font_6x8, str);
  
  format_double((pwm_duty+1.0)/(PWM_RESOLUTION) * 100.0, 2, buf, 20);
  sprintf(str, "PWM duty: %s %%", buf);
  DOG.string(0,4,font_6x8, str);
  
  DOG.string(0,5,font_6x8,"State:");
  switch (state)
  {
    case STANDBY:
      DOG.string(7*6,5,font_6x8,"Standby");
      break;
    case CHG_CC:
      DOG.string(7*6,5,font_6x8,"CC charge");
      break;
    case CHG_CV:
      DOG.string(7*6,5,font_6x8,"CV charge");
      break;
    case BAT_ERROR:
      byte sys_stat;
      sys_stat = BMS.checkStatus();
      if (sys_stat & B00100000)
        DOG.string(7*6,5,font_6x8,"XR Error");
      if (sys_stat & B00010000)
        DOG.string(7*6,5,font_6x8,"Alert Error");
      if (sys_stat & B00001000)
        DOG.string(7*6,5,font_6x8,"UV Error");
      if (sys_stat & B00000100)
        DOG.string(7*6,5,font_6x8,"OV Error");
      if (sys_stat & B00000010)
        DOG.string(7*6,5,font_6x8,"SCD Error");
      if (sys_stat & B00000001)
        DOG.string(7*6,5,font_6x8,"OCD Error");
      break;
    case BALANCING:
      DOG.string(7*6,5,font_6x8,"Balancing");
      break;    
  }

  format_double(BMS.getCellVoltage(1)/1000.0, 3, buf, 20);
  sprintf(str, "1:%sV", buf);
  DOG.string(0,6,font_6x8, str);

  format_double(BMS.getCellVoltage(2)/1000.0, 3, buf, 20);
  sprintf(str, "2:%sV", buf);
  DOG.string(51,6,font_6x8, str);
  
  format_double(BMS.getCellVoltage(3)/1000.0, 3, buf, 20);
  sprintf(str, "3:%sV", buf);
  DOG.string(0,7,font_6x8, str);

  format_double(BMS.getCellVoltage(5)/1000.0, 3, buf, 20);
  sprintf(str, "4:%sV", buf);
  DOG.string(51,7,font_6x8, str);
  
}

//----------------------------------------------------------------------------
// Print out current status of the system
void print_status(void)
{
  char str[20];
  char buf[20];
    
  format_double(solar_voltage/1000.0, 2, buf, 20);
  sprintf(str, "Sol: %sV ", buf);
  Serial.print(str);

  format_double(solar_current/1000.0, 2, buf, 20);
  sprintf(str, "%sA", buf);
  Serial.println(str);
  
  format_double(BMS.getBatteryVoltage()/1000.0, 2, buf, 20);
  sprintf(str, "Bat: %sV ", buf);
  Serial.print(str);

  format_double(BMS.getBatteryCurrent()/1000.0, 2, buf, 20);
  sprintf(str, "%sA", buf);
  Serial.println(str);

  format_double(BMS.getTemperatureDegC(), 1, buf, 20);
  sprintf(str, "Temp: %s °C", buf);
  Serial.println(str);
  
  format_double(solar_power/100.0, 2, buf, 20);
  sprintf(str, "Power: %s W", buf);
  Serial.println(str);
  
  format_double((pwm_duty+1.0)/(PWM_RESOLUTION) * 100.0, 2, buf, 20);
  sprintf(str, "PWM duty: %s %%", buf);
  Serial.println(str);
  
  Serial.print("State:");
  switch (state)
  {
    case STANDBY:
      Serial.println("Standby");
      break;
    case CHG_CC:
      Serial.println("CC charge");
      break;
    case CHG_CV:
      Serial.println("CV charge");
      break;
    case BAT_ERROR:
      byte sys_stat;
      sys_stat = BMS.checkStatus();
      if (sys_stat & B00100000)
        Serial.println("XR Error");
      if (sys_stat & B00010000)
        Serial.println("Alert Error");
      if (sys_stat & B00001000)
        Serial.println("UV Error");
      if (sys_stat & B00000100)
        Serial.println("OV Error");
      if (sys_stat & B00000010)
        Serial.println("SCD Error");
      if (sys_stat & B00000001)
        Serial.println("OCD Error");
      break;
    case BALANCING:
      Serial.println("Balancing");
      break;    
  }

  format_double(BMS.getCellVoltage(1)/1000.0, 3, buf, 20);
  sprintf(str, "1:%sV", buf);
  Serial.print(str);

  format_double(BMS.getCellVoltage(2)/1000.0, 3, buf, 20);
  sprintf(str, "  2:%sV", buf);
  Serial.print(str);
  
  format_double(BMS.getCellVoltage(3)/1000.0, 3, buf, 20);
  sprintf(str, "  3:%sV", buf);
  Serial.print(str);

  format_double(BMS.getCellVoltage(5)/1000.0, 3, buf, 20);
  sprintf(str, "  4:%sV", buf);
  Serial.println(str);
  
}

//----------------------------------------------------------------------------
int update_solar_power(void)
{
  int vcc;
  vcc = read_vcc();  // using internal reference voltage

  // subtract the last reading:
  solar_voltage_total -= solar_voltage_readings[i_readings];
  solar_current_total -= solar_current_readings[i_readings];

  // save new reading
  solar_voltage_readings[i_readings] = analogRead(SOLAR_VOLTAGE_PIN)/1023.0 * vcc * 106.8/6.8;  // mV
  
  // 220 kOhm resistor at INA168 --> gain 44
  // 5 mOhm shunt resistor
  solar_current_readings[i_readings] = analogRead(SOLAR_CURRENT_PIN)/1023.0 * vcc / 5.0 * 44;  // mA

  // add the reading to the total:
  solar_voltage_total += solar_voltage_readings[i_readings];
  solar_current_total += solar_current_readings[i_readings];
  
  i_readings++;
  if (i_readings >= NUM_AVG) {
    i_readings = 0;
  }

  solar_voltage = solar_voltage_total / NUM_AVG;
  solar_current = solar_current_total / NUM_AVG;

  solar_power = solar_voltage / 100.0 * solar_current / 100.0;
}
