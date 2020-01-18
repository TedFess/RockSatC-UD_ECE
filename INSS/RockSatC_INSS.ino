#include "SparkFun_LIS331.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

LIS331 xl;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

void setup() 
{
  // put your setup code here, to run once:
  Serial.println("INSS Test"); Serial.println("");

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055*/
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  bno.setExtCrystalUse(true);
  
  pinMode(9,INPUT);       // Interrupt pin input
  Wire.begin();
  xl.setI2CAddr(0x19);
  xl.begin(LIS331::USE_I2C); // Selects the bus to be used and sets
                          //  the power up bit on the accelerometer.
                          //  Also zeroes out all accelerometer
                          //  registers that are user writable.

  xl.intSrcConfig(LIS331::INT_SRC, 1); // Select the source of the
                          //  signal which appears on pin INT1. 
  xl.setIntDuration(50, 1); // Number of samples a value must meet
                          //  the interrupt condition before an
                          //  interrupt signal is issued. At the
                          //  default rate of 50Hz, this is one sec.
  xl.setIntThreshold(2, 1); // Threshold for an interrupt. 
  xl.enableInterrupt(LIS331::Z_AXIS, LIS331::TRIG_ON_HIGH, 1, true);
                          // Enable the interrupt.
  Serial.begin(115200);
}


void loop() 
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");
  
  delay(100);
  
  static long loopTimer = 0;
  int16_t x, y, z;
  if (millis() - loopTimer > 1000)
  {
    loopTimer = millis();
    xl.readAxes(x, y, z);  // The readAxes() function transfers the
                           //  current axis readings into the three
                           //  parameter variables passed to it.
    
    Serial.println(z);

    Serial.println(xl.convertToG(100,z)); // raw value and the current
    Serial.println(" ");                // maximum g-rating.
  }
  if (digitalRead(9) == HIGH)
  {
    Serial.println("Interrupt");
  }
}
