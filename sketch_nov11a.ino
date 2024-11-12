// Include the required Arduino libraries:
#include <OneWire.h>
#include <DallasTemperature.h>

// Define to which pin of the Arduino the 1-Wire bus is connected:
#define ONE_WIRE_BUS 4

// Create a new instance of the oneWire class to communicate with any OneWire device:
OneWire oneWire(ONE_WIRE_BUS);

// Defining PID control variables
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp, ki, kd;


void Compute()
{
/*How long since we last calculated*/
unsigned long now = millis();
double timeChange = (double)(now - lastTime);
 
/*Compute all the working error variables*/
double error = Setpoint - Input;
errSum += (error * timeChange);
double dErr = (error - lastErr) / timeChange;
 
/*Compute PID Output*/
Output = kp * error + ki * errSum + kd * dErr;
 
/*Remember some variables for next time*/
lastErr = error;
lastTime = now;
}

// Pass the oneWire reference to DallasTemperature library:
DallasTemperature sensors(&oneWire);

void setup() {
  kp = 1;
  ki = 0;
  kd = 1;

  // Begin serial communication at a baud rate of 9600:
  Serial.begin(9600);
  // Start up the library:
  sensors.begin();
}

void loop() {
  // Send the command for all devices on the bus to perform a temperature conversion:
  sensors.requestTemperatures();

  // Fetch the temperature in degrees Celsius for device index:
  float tempC = sensors.getTempCByIndex(0); // the index 0 refers to the first device
  // Fetch the temperature in degrees Fahrenheit for device index:
  float tempF = sensors.getTempFByIndex(0);

  // Print the temperature in Celsius in the Serial Monitor:
  Serial.print("Temperature: ");
  Serial.print(tempC);
  Serial.print(" \xC2\xB0"); // shows degree symbol
  Serial.print("C  |  ");

  // Print the temperature in Fahrenheit
  Serial.print(tempF);
  Serial.print(" \xC2\xB0"); // shows degree symbol
  Serial.println("F");

  Input = tempC;
  Compute();

  Serial.print(Output);

  // Wait 1 second:
  delay(1000);
}