#include <Arduino_LPS22HB.h> //barometer
#include "Arduino_BMI270_BMM150.h" //IMU

////////////////////////////////

// Forward declarations
float initialiseAltimeter(float launchAltitude);
float readAltitude(float P0);
float readTemperature();
float readPressure();
void writeToSerial();
int checkState(int &state, float altitude, float acceleration);
float readAcceleration();

#define LAUNCH_THRESHOLD 1.1
#define ARRAY_SIZE 5
#define GREEN_PIN 5 
#define RED_PIN 4 

// Variables
float accelerations[ARRAY_SIZE];  // Array to store last 5 readings
const float P0 = 100.3;  // assumed Sea level pressures
const float launchAltitude = 127;
float altitudeCorrection = 0;
float temperature = 0;
int state = 0; //0 = ground idle, 1 = powered flight, 2 = gliding up, 3 = ballistic descent, 4 = chute decent, 5 = landed, 6 = error
float apogee = 0;

///////////////////////////////////

void setup() {
//initialise serial
  Serial.begin(9600);
  while (!Serial);

// Initialize the LED pin
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);

  //blink test
  digitalWrite(RED_PIN, HIGH);  
  digitalWrite(GREEN_PIN, HIGH);  
  delay(100);
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, LOW);


//initialise IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

delay(500);

//initialise barometer
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

delay(500);

//initialise altimeter
altitudeCorrection = initialiseAltimeter(launchAltitude); 

//print headers
  Serial.print("Timestamp (s),");
  Serial.print("Pressure (kPa),");
  Serial.print("Altitude (m),");
  Serial.print("Temperature (c),");
  Serial.print("Acceleration (g),");
  Serial.print("Flight State,");
  Serial.println("Apogee (m)");
Serial.println();

// setup complete
  digitalWrite(GREEN_PIN, HIGH); 

}

void loop() {

float altitude = readAltitude(P0);

float temperature = readTemperature();

float pressure = readPressure();

// collect acceleration
float acceleration = readAcceleration();

// collect yaw

// check state

int currentState = checkState(state, altitude, acceleration);

//output to serial
writeToSerial(pressure, altitude, temperature, acceleration, currentState, apogee);

  // wait .1 second to check sensors again. This produces logging c. every 150msec
  delay(100);
}

/////////////////////////////////////////////

// state machine
int checkState(int &state, float altitude, float acceleration){
switch (state) {
  case 0:
    if (accelerations[0] >= LAUNCH_THRESHOLD && accelerations[1] >= LAUNCH_THRESHOLD) { //we have lift off!
    state = 1;
    Serial.println("launch detected!");
    digitalWrite(RED_PIN, HIGH);  // Turn on the red LED
    }
    return state;
    break;
  case 1:
    // are we still in powered flight?
    // if not move to state 2
    // state = 2;
    return state;
    break;
  case 2:
    // have we reached apogee?
    // if so move to state 2
    // case = 3
    return state;
    break;
  case 3:
    // have we deployed parachutes? is our rate of decent falling    
    // if so move to state 4
    // case = 4
    return state;
    break;
  case 4:
    // have we landed? is there 0 yaw and our altitude +/- 10m of launch
    // if so move to state 5
    // case = 5
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

void writeToSerial(float pressure, float altitude, float temperature, float acceleration, int currentState, float apogee){

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
    IMU.readAcceleration(x,y,z); //only using the x value, y & z silently ignored
  acceleration = x;
  }

  for (int i = ARRAY_SIZE - 2; i >= 0; i--) {
    accelerations[i + 1] = accelerations[i];  // Move value at i to i+1
  }

  // Store the new reading in index 0
  accelerations[0] = acceleration;

return acceleration;

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

// record altiude values to suitable data structure here for use in checkState

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