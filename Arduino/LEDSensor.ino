/*
  DigitalReadSerial with on-board Pushbutton
  Reads a digital input on pin 5, prints the result to the serial monitor 
 
  Harware Required:
  * MSP-EXP430FR2355 LaunchPad
  
  This example code is in the public domain.
 */

int pushButton    = P4_1;
int LED_ANODE     = P1_2;
int LED_CATHODE   = P1_3;
int LED_SENSE     = P1_4;  //Junction of LED anode and R

void setup() 
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200); // msp430g2231 must use 4800
  
  // make the on-board pushbutton's pin an input pullup:
  pinMode(pushButton, INPUT_PULLUP);
  
  pinMode(LED_ANODE, OUTPUT);
  pinMode(LED_CATHODE, OUTPUT);
}

/**  
 * Experiment: Basic blinky to test the limit of perception of a
 *             flicker free blinking LED
 * Observations:
 *   1. Less than 50Hz has a noticeable flicker
*/
void LEDTestPerception(void) 
{
  //Turn ON LED
  digitalWrite(LED_CATHODE, 0);
  digitalWrite(LED_ANODE, 1);
  delay(100);

  //Turn OFF LED
  digitalWrite(LED_CATHODE, 0);
  digitalWrite(LED_ANODE, 0);
  delay(19);
}

/** 
 * Experiment: Reverse bias the LED to 3.3V through a 100E resistor
 *             and monitor the voltage on the anode by setting the PIN 
 *             as INPUT and measure the time taken to read it as HIGH.
 * Observations:
 *   1. Medium ambient light - 360ms to 400ms
 *   2. Low ambient light (holding hand over led) - more than 5s
 *   3. Flashlight shining - 10ms
 */
void GraphLEDDigitalAnode() 
{
  unsigned long time;

  pinMode(LED_ANODE, OUTPUT);
  digitalWrite(LED_CATHODE, HIGH);      // Reverse bias LED
  digitalWrite(LED_ANODE, LOW);
  delay(5);                            // wait few ms to charge capacitance

  time = millis();                      // Grab the current time
  pinMode(LED_ANODE, INPUT);            // Let go of LED and allow it to discharge
  while (digitalRead(LED_ANODE) == 0);  // wait for LED_ANODE to go HIGH
  time = millis() - time;               // How long has it been?

  Serial.println(time);                 // Output the elapsed time
  delay(1);
}

/** 
 * Experiment: Reverse bias the LED to 3.3V through a 100E resistor
 *             and monitor the voltage on the cathode by setting the PIN 
 *             as INPUT and measure the time taken to read it as HIGH.
 * Observations:
 *   1. Medium ambient light - 320ms to 340ms
 *   2. Low ambient light (holding hand over led) - 500ms to 600ms
 *   3. Flashlight shining - 10ms
 */
void GraphLEDDigitalCathode() 
{
  unsigned long time;

  pinMode(LED_CATHODE, OUTPUT);
  digitalWrite(LED_CATHODE, HIGH);    // Reverse bias LED
  digitalWrite(LED_ANODE, LOW);
  delay(5);                           // wait a ms to charge capacitance

  time = millis();                    // Grab the current time
  pinMode(LED_CATHODE, INPUT);        // Let go of LED and allow it to discharge

  while (digitalRead(LED_CATHODE));   // wait for LED_CATHODE to go LOW
  time = millis() - time;             // How long has it been?

  Serial.println(time);               // Output the elapsed time
  delay(1);
}

/**
 * Experiment: Measure the voltage generated across the LED. 
 *             Connect Cathode to LOW and monitor voltage at anode.
 * Observations:
 *  a) 10ms interval
 *      1. Medium ambient light - 250 to 280
 *      2. Low ambient light (holding hand over led) - 180 to 200
 *      3. Flashlight shining - 330 to 350
 *  b) 100ms interval
 *      1. Medium ambient light - 250 to 280
 *      2. Low ambient light (holding hand over led) - 180 to 200
 *      3. Flashlight shining - 330 to 350
 *  Not much changes with change in interval
 */
void GraphLEDAnalogAnode1() {
  int analogVal;

  pinMode(LED_ANODE, INPUT);
  digitalWrite(LED_CATHODE, LOW);     
  
  analogVal = analogRead(LED_ANODE);  // Read ADC value of Cathode
  Serial.println(analogVal);          // Output the analogVal
  
  delay(100);
}

/**
 * Experiment: Measure the voltage generated across the LED. 
 *             Connect Cathode to HIGH and monitor voltage at anode.
 * Observations:
 *  a) 10ms interval
 *      1. Medium ambient light - 260 to 300
 *      2. Low ambient light (holding hand over led) - 210 to 240
 *      3. Flashlight shining - 900 to 920
 *  b) 100ms interval
 *      1. Medium ambient light - 640 to 720
 *      2. Low ambient light (holding hand over led) - 320 to 350
 *      3. Flashlight shining - 900 to 920
 *  This config give the maximum delta between light and dark conditions
 */
void GraphLEDAnalogAnode2() {
  int analogVal;
  
  pinMode(LED_ANODE, INPUT);
  digitalWrite(LED_CATHODE, HIGH);     

  analogVal = analogRead(LED_ANODE);  // Read ADC value of Cathode, this takes less than 400us
  Serial.println(analogVal);          // Output the analogVal
  
  delay(200);
}

/**
 * Experiment: Measure the voltage generated across the LED. 
 *             Connect Anode to LOW and monitor voltage at Cathode.
 * Observations:
 *  a) 10ms interval
 *      1. Medium ambient light - 75 to 150
 *      2. Low ambient light (holding hand over led) - 125 to 200
 *      3. Flashlight shining - 0
 *      High frequency oscillations
 *  b) 100ms interval
 *      1. Medium ambient light - 10 to 70
 *      2. Low ambient light (holding hand over led) - 34 to 86
 *      3. Flashlight shining - 0
 *      Low frequency oscillations
 */
void GraphLEDAnalogCathode1() {
  int analogVal;

  pinMode(LED_CATHODE, INPUT);
  digitalWrite(LED_ANODE, LOW);     
  
  analogVal = analogRead(LED_CATHODE);  // Read ADC value of Cathode
  Serial.println(analogVal);            // Output the analogVal
  
  delay(100);
}

/**
 * Experiment: Measure the voltage generated across the LED at a fixed interval. 
 *             Connect Anode to HIGH and monitor voltage at Cathode.
 * Observations:
 *  a) 10ms interval
 *      1. Medium ambient light - 260 to 300
 *      2. Low ambient light (holding hand over led) - 210 to 240
 *      3. Flashlight shining - 900 to 920
 *  b) 100ms interval
 *      1. Medium ambient light - 630 to 656
 *      2. Low ambient light (holding hand over led) - 630 to 668
 *      3. Flashlight shining - 624 to 632
 *  Not much changes with change in interval
 */
void GraphLEDAnalogCathode2() {
  int analogVal;

  pinMode(LED_CATHODE, INPUT);
  digitalWrite(LED_ANODE, HIGH);     
  
  analogVal = analogRead(LED_CATHODE);  // Read ADC value of Cathode
  Serial.println(analogVal);            // Output the analogVal
  
  delay(10);
}

/**
 * Experiment: 
 */
void GraphLEDAnalogAnodeCharged()
{
  int analogVal;
  pinMode(LED_ANODE, OUTPUT);
  digitalWrite(LED_ANODE, LOW);       // Reverse bias the LED
  digitalWrite(LED_CATHODE, HIGH);
  delay(5);                           // wait a ms to charge capacitance
  
  pinMode(LED_ANODE, INPUT);          // Let go of LED and allow it to discharge
  delay(20);                          // Wait
  analogVal = analogRead(LED_ANODE);  // Read ADC value of Cathode
  Serial.println(analogVal);          // Output the analogVal
  
  delay(100);
}

void AutoLight_withAnode(void) 
{
  // Reverse bias the LED
  pinMode(LED_ANODE, OUTPUT);      
  digitalWrite(LED_ANODE, LOW);    
  digitalWrite(LED_CATHODE, HIGH);

  // wait few ms to charge capacitance
  delay(5);                   
  // Let go of LED and allow it to discharge
  pinMode(LED_ANODE, INPUT);       

  // Wait
  delay(70);                  
  Serial.println(digitalRead(LED_ANODE));

  // Check if charged past logic threshold
  // No, turn on LED
  if (!digitalRead(LED_ANODE)) {   
    pinMode(LED_ANODE, OUTPUT);    
    digitalWrite(LED_ANODE, HIGH);
    digitalWrite(LED_CATHODE, LOW);
  }
  
  delay(10);
}

void AutoLight_withCathode(void) 
{
  // Reverse bias the LED
  pinMode(LED_CATHODE, OUTPUT);      
  digitalWrite(LED_CATHODE, HIGH);    
  digitalWrite(LED_ANODE, LOW);

  // wait few ms to charge capacitance
  delay(1);                   
  // Let go of LED and allow it to discharge
  pinMode(LED_CATHODE, INPUT);       

  // Wait
  delay(360);                  
  Serial.println(digitalRead(LED_CATHODE));

  // Check if not discharged below logic threshold turn on LED
  if (digitalRead(LED_CATHODE)) {   
    pinMode(LED_CATHODE, OUTPUT);    
    digitalWrite(LED_CATHODE, LOW);
    digitalWrite(LED_ANODE, HIGH);
    delay(10);
    digitalWrite(LED_ANODE, LOW);
    delay(400);
    digitalWrite(LED_ANODE, HIGH);
    delay(10);
    digitalWrite(LED_ANODE, LOW);
  }
  
  delay(1000);
}

void SenseAndLight() {
  unsigned long time;

  digitalWrite(LED_CATHODE, HIGH);  // Reverse bias LED
  digitalWrite(LED_ANODE, LOW);
  
  delay(5);                   // wait a ms to charge capacitance

  time = millis();            // Grab the current time

  pinMode(LED_ANODE, INPUT);       // Let go of LED and allow it to discharge

  while (!digitalRead(LED_ANODE)); // wait for LED_ANODE to go HIGH

  time = millis() - time;     // How long has it been?

  pinMode(LED_ANODE, OUTPUT);      // Turn on LED
//  digitalWrite(LED_CATHODE, LOW);
//  digitalWrite(LED_ANODE, HIGH);

  delay(1);

  Serial.println(time);       // Output the elapsed time
}

void LEDSenseLight(void) {
  volatile unsigned long t1, t2;

  pinMode(LED_ANODE, OUTPUT);
  
  //Discharge the LED
  digitalWrite(LED_CATHODE, 0);
  digitalWrite(LED_ANODE, 0);
  delay(10);
  
  //Charge the LED
  digitalWrite(LED_CATHODE, 0);
  digitalWrite(LED_ANODE, 1);
  delay(10);

  //Monitor the LED discharge
  pinMode(LED_ANODE, INPUT);
  t1 = millis();
  while(digitalRead(LED_ANODE)); //This can take 60-120 ms
  t2 = millis();
  Serial.print("delta t=");
  Serial.println(t2-t1);

  //Turn ON LED if it is dark
  if((t2-t1) > 110) {
    digitalWrite(LED_CATHODE, 1);
    digitalWrite(LED_ANODE, 0);
    pinMode(LED_ANODE, OUTPUT);
  } 

  //Wait before getting the next sample
  delay(100);   
}

// the loop routine runs over and over again forever:
void loop() {
//  LEDSenseLight();
//  LEDTestPerception();
//  AutoLight();
    AutoLight_withCathode();
//  SenseAndLight();
//  GraphLEDDigitalAnode();
//  GraphLEDDigitalCathode();
//  GraphLEDAnalogAnode1();
//  GraphLEDAnalogAnode2();
//  GraphLEDAnalogCathode1();
//  GraphLEDAnalogCathode2();
//  GraphLEDAnalogAnodeCharged();
}
