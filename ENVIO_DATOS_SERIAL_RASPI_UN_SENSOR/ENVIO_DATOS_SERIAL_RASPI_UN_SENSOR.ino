/*
  CN0357_example.ino - Example code for CN0357 - Toxic Gas Detector
 Created by Analog Devices Inc. - Circuits from the Lab, December 2014.
 */

#include <Arduino.h>
#include <SPI.h>
#include "AD5270_CO.h"
#include "AD7790_CO.h"

// AD5270 Pin Assignments
#define AD5270_SS_CO      7  // value of the CS pin assignment

// AD7790 Pin Assignments
#define AD7790_SS_CO      8  // value of the CS pin assignment
int ppm;
//  Gas Sensor Variables
uint16_t ui16sensorRange_CO = 2000;      //value is in units (PPM)
uint16_t ui16sensitivity_CO =  69.5;      //value is in units (nA/ppm)

// AD5270 variables
float fResistorValue_CO = 0;
uint16_t ui16RdacWord_CO = 0;

// AD7790 variables
uint16_t ui16Adcdata_CO = 0;

// Main variables
float fAdcVoltage_CO = 0;
float fConcentration_CO = 0;

void setup() {
  
  // open digital communication protocols
  Serial.begin(9600);
  SPI.begin();

        // initialize pins
  pinMode(AD5270_SS_CO,OUTPUT);         //set CS pin for sensor
  digitalWrite(AD5270_SS_CO,HIGH);    //brings chip select high 
    
  pinMode(AD7790_SS_CO,OUTPUT);         //set CS pin for LCD
  digitalWrite(AD7790_SS_CO,HIGH);    //brings chip select high 
    
        // initialize AD5270
        AD5270.AD5270_SPI_Configuration();
  Ad5270INIT();
  
  // set digipot value
  fResistorValue_CO = calculateFeedbackResistor();
  ui16RdacWord_CO = setResistorValue(fResistorValue_CO);
  AD5270.writeAd5270 (WRITE_RDAC, ui16RdacWord_CO);
        AD5270.writeAd5270 (HI_Z_PREP, 0x8001);  // Putting Rheostat into high Z mode on SDO line
        AD5270.writeAd5270 (HI_Z, 0x0000);
  //  Serial.print("Calculated Digitpot Value = ");
  //Serial.println(fResistorValue_CO); 
 //Serial.print("Actual Digitpot Value = ");
  //Serial.println(ui16RdacWord_CO); 

  // initialize AD7790
  AD7790.AD7790_SPI_Configuration();
  Ad7790INIT(); 
}

void loop() {

  do
        {
          ui16Adcdata_CO = AD7790.readAd7790(STATUS_READ);
         // Serial.print("ADC Status Reg value = ");
    //Serial.println(ui16Adcdata_CO); 
        }while (ui16Adcdata_CO & 0x80); 

        ui16Adcdata_CO = AD7790.readAd7790(DATA_READ);
  fAdcVoltage_CO = ((ui16Adcdata_CO / pow(2,15))-1)*1.2;    // Formula for input voltage using bipolar configuration
  fConcentration_CO = (abs(fAdcVoltage_CO)/ (ui16RdacWord_CO*(20000/1024))) / (ui16sensitivity_CO*pow(10,-9));
   //Serial.print("valor flotante: ");  //display gas concentration
 // Serial.print(fConcentration_CO);  //display gas concentration
 // ppm = int(fConcentration_CO);
  //Serial.print("ADC Data Reg value = ");
       // Serial.println(ui16Adcdata_CO);
       // Serial.print("Sensor Voltage value = ");
  //Serial.println(fAdcVoltage_CO);
 if (Serial.available()){
      char c = Serial.read();
      if (c=='B') {
        //Serial.print("Carbon Monoxide (CO) concentration = ");
  //Serial.print("valor entero: ");      
  Serial.print(fConcentration_CO);  //display gas concentration
  Serial.print(",");
  Serial.print(analogRead(1));
  Serial.print(",");
  Serial.println(analogRead(0));
  //Serial.print(",");
  
  //Serial.println(" PPM");
 }
 }
}

void Ad7790INIT(void)
{
  AD7790.writeAd7790 (RESET, 0xFF);       //Resets the part for initial use
  //delay(100);
  AD7790.writeAd7790 (MODE_WRITE, 0x00);      //Mode register value (single conversion, +/- Vref input, unbuffered mode)
  AD7790.writeAd7790 (FILTER_WRITE, 0x07);    // Filter register value (clock not divided down, 9.5 Hz update rate)
}

void Ad5270INIT(void)
{
  AD5270.writeAd5270 (WRITE_CTRL_REG, 0x02);    //Enable RDAC writes
}

float calculateFeedbackResistor(void)
{
  float fFeedback = 0;
  fFeedback = 1.2 / (ui16sensorRange_CO * ui16sensitivity_CO * pow(10,-9));     //1.2 is the Vref of the circuit
  return fFeedback;
}

uint16_t setResistorValue(float resistor)
{
  uint16_t ui16RdacCode = 0;
  ui16RdacCode = int(resistor / (20000/1024));
  return ui16RdacCode;
}
