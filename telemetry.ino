#include <Arduino_LPS22HB.h> //barometer
#include "Arduino_BMI270_BMM150.h" //IMU
#include "LED.h"

////////////////////////////////

// Forward declarations
float initialiseAltimeter(float launchAltitude);
float readAltitude(float P0);
float readTemperature();
float readPressure();
void writeToSerial();
int checkState(int &state, float altitude, float acceleration);
float readAcceleration();
float readRotation();
void intialiseSensor(char Sensor, char Text);
void blnkTest();

// initialise leds
LED redLED(4);
LED greenLED(5);
LED whiteLED(6);
LED whiteGROUP(7);

LEDGroup allLEDS;



#define LAUNCH_THRESHOLD 1.3
#define BURNOUT_THRESHOLD 1.2
#define LANDING_ROT_THRESHOLD 1
#define ARRAY_SIZE 5


// Variables
float accelerations[ARRAY_SIZE];  // Array to store last 5 readings
float altitudes[ARRAY_SIZE];
float rotations[ARRAY_SIZE];
const float P0 = 100.3;  // assumed Sea level pressures
const float launchAltitude = 127;
float altitudeCorrection = 0;
float temperature = 0;
int state = 0; //0 = ground idle, 1 = powered flight, 2 = gliding up, 3 = ballistic descent, 4 = chute decent, 5 = landed, 6 = error
float apogee = 0;
// sensor initialisation
int wait = 5; // initialisation delay
const int maxAttempts = 5; //how many times do we try to initialise sensors before moving on?
int attempt = 0;
///////////////////////////////////

void setup() {

//initialise serial but dont wait
Serial.begin(9600);


blinkTest();


Serial.println("serial initialised");

// Initialize IMU
initialiseSensor(IMU, "IMU");

// Initialize Barometer
initialiseSensor(BARO, "Pressure sensor");
delay(500);

//initialise altimeter
altitudeCorrection = initialiseAltimeter(launchAltitude); 

//print headers
  Serial.print("Timestamp (s),");
  Serial.print("Pressure (kPa),");
  Serial.print("Altitude (m),");
  Serial.print("Temperature (c),");
  Serial.print("Acceleration (g),");
  Serial.print("Rotation (degrees/sec),");
  Serial.print("Flight State,");
  Serial.println("Apogee (m)");
Serial.println();

// setup complete
  greenLED.turnOn(); 

}

void loop() {

float altitude = readAltitude(P0);

float temperature = readTemperature();

float pressure = readPressure();

// collect acceleration
float acceleration = readAcceleration();

// collect yaw
float rotation = readRotation();

// check state

int currentState = checkState(state, altitude, acceleration, rotation);

if (currentState == 5) {
       while (1) {
      whiteGROUP.turnOn();
      delay(500);
      whiteGROUP.turnOff();
      delay(500);
      }
}

//output to serial
writeToSerial(pressure, altitude, temperature, acceleration, rotation, currentState, apogee);

  // wait .1 second to check sensors again. This produces logging c. every 150msec
  delay(100);
}

/////////////////////////////////////////////

// state machine
int checkState(int &state, float altitude, float acceleration, float rotation){
switch (state) {
  case 0:
    if (accelerations[0] >= LAUNCH_THRESHOLD && accelerations[1] >= LAUNCH_THRESHOLD) { //we have lift off!
    state = 1;
    Serial.println("launch detected!");
    redLED.turnOn();
    greenLED.turnOff();  
    }
    return state;
    break;
  case 1:
    if (altitudes[0] > altitudes[1] && altitudes[1] > altitudes[2] && accelerations[0] <= BURNOUT_THRESHOLD && accelerations[1] <= BURNOUT_THRESHOLD) { //motor burnt out
    state = 2;
    Serial.println("burnout detected");
    redLED.turnOn();  
    greenLED.turnOn();  
    }
    return state;
    // break; we can fall through
  case 2:
    // have we reached/passed apogee?
    if (altitudes[0] > altitudes[1] && altitudes[1] > altitudes[2] && altitudes[2] > altitudes[3] && altitudes[3] > altitudes[4]) { //we are descending
    redLED.turnOff(); 
    greenLED.turnOff();  
    whiteLED.turnOn();
    state = 3;
    }
    return state;
    break;
  case 3:
    // have we deployed parachutes? is our rate of decent falling    
    // if so move to state 4
    // state = 4
    // return state;
    // no break;, allow to fall through if we dont detect parachutes
  case 4:
    // have we landed?
    if (rotations[0] <= LANDING_ROT_THRESHOLD && rotations[1] <= LANDING_ROT_THRESHOLD) { // the rocket is still, have we landed?
    // if so move to state 5
    whiteLED.turnOff();
    state = 5;
    }
    return state;
    break;
  default:
    Serial.println("Error detecting state");
    state = 6;
    return state;
    break;
  }
}

///////////////////////////////////////

void writeToSerial(float pressure, float altitude, float temperature, float acceleration, int rotation, int currentState, float apogee){

  // Get the current timestamp in milliseconds and convert to seconds
  float elapsedTime = (float)millis() / 1000.0;  // Convert to seconds with tenths

  // Print the timestamp
  Serial.print(elapsedTime,2);
  Serial.print(",");

  // print the pressure value
  //Serial.print("Pressure")
  Serial.print(pressure);
  Serial.print(",");
  //Serial.println(" kPa");
  // print the calculated altitude
  //Serial.print("Altitude = ");
  Serial.print(altitude,1);
  Serial.print(",");
  //Serial.println(" meters");

  // print the temperature value
  //Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.print(",");
  //Serial.println(" C");

 // print the acceleration value
 //Serial.print("Accelaration = ");
  Serial.print(acceleration);
  Serial.print(",");
  //Serial.println(" G");

  Serial.print(rotation);
  Serial.print(",");

  //print the state
  //Serial.print("State = ");
  Serial.print(currentState);
  Serial.print(",");
  
  //Serial.print("Apogee = ");
  Serial.print(apogee);

  // print an empty line
  Serial.println();

}

///////////////////////////////////////

float readAcceleration(){
float x, y, z, acceleration;

 if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x,y,z);
  acceleration = sqrt(x * x + y * y + z * z); //magnitude of acceleration to colate data from 3 axis
  }

  for (int i = ARRAY_SIZE - 2; i >= 0; i--) {
    accelerations[i + 1] = accelerations[i];  // Move value at i to i+1
  }

  // Store the new reading in index 0
  accelerations[0] = acceleration;

return acceleration;

}

//////////////////////////

float readRotation(){

float rotation, x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    rotation = sqrt(x * x + y * y + z * z); //magnitude of yaw/pitch/roll to colate data from 3 axis
  }

  for (int i = ARRAY_SIZE - 2; i >= 0; i--) {
    rotations[i + 1] = rotations[i];  // Move value at i to i+1
  }

  // Store the new reading in index 0
  rotations[0] = rotation;


return rotation;


}

//////////////////////////

float readTemperature(){
  // read the temperature value
  float temperature = BARO.readTemperature();
  return temperature;
}

float readPressure() {

    // Read the pressure value
  float pressure = BARO.readPressure();
  return pressure;
}

///////////////////////////////

float readAltitude(float P0) {
  // Read the pressure value
  float pressure = BARO.readPressure();


  // Calculate altitude using the barometric formula
  float altitude = 44330 * (1.0 - pow(pressure / P0, 0.1903));

  // Apply altitude correction
  altitude += altitudeCorrection;

  if (altitude > apogee) {
    apogee = altitude;
  }

for (int i = ARRAY_SIZE - 2; i >= 0; i--) {
    altitudes[i + 1] = altitudes[i];  // Move value at i to i+1
  }

  // Store the new reading in index 0
  altitudes[0] = altitude;

return altitude;
}

///////////////////////////////////

//function to generate altimeter offset
float initialiseAltimeter(float launchAltitude) {

// read the pressure
float pressure = BARO.readPressure();
//calculate alltitude
float altitude = 44330 * (1.0 - pow(pressure / P0, 0.1903));

// create alltitude correction in M
  altitudeCorrection = launchAltitude - altitude;

return altitudeCorrection;
}

////////////////////////////////////////
// Corrected function definition
template<typename T>
void initialiseSensor(T& sensor, const char* text) {
  int attempt = 0;
  const int maxAttempts = 5;

  while (attempt < maxAttempts) {
    if (sensor.begin()) {
      Serial.print(text); 
      Serial.println(" initialized successfully!");
      break;
    } else {
      attempt++;
      Serial.print("Failed to initialize ");
      Serial.print(text);
      Serial.println("! Retrying...");
      delay(1000);
    }
  }

  if (attempt == maxAttempts) {
    Serial.print("Failed to initialize ");
    Serial.print(text);
    Serial.println(" after multiple attempts!");
    while (1); // Halt the program
  }
}


////////////////////////////////////////

//blink test
void blinkTest() {

allLEDS.addLED(4);
allLEDS.addLED(5);
allLEDS.addLED(6);
allLEDS.addLED(7);

///
int wait = 300;
redLED.turnOn();
delay(wait);
greenLED.turnOn();
delay(wait);
whiteLED.turnOn();
delay(wait);
whiteGROUP.turnOn();
delay(wait);

allLEDS.turnAllOff();
delay(500);

allLEDS.blinkAll(500, 2);
allLEDS.blinkAll(100, 1);
}