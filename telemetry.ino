#include <Arduino_LPS22HB.h> //barometer
#include "Arduino_BMI270_BMM150.h" //IMU

////////////////////////////////

// Forward declarations
float initialiseAltimeter(float launchAltitude);
float readAltitude(float P0);
float readTemperature();
float readPressure();
void writeToSerial();
int checkState();
float readAcceleration();

//vars
const float P0 = 100.3;  // assumed Sea level pressures
const float launchAltitude = 127;
float altitudeCorrection = 0;
float temperature = 0;
int state = 0; //0 = ground idle, 1 = powered flight, 2 = gliding up, 3 = ballistic descent, 4 = chute decent, 5 = landed, 6 = error


///////////////////////////////////

void setup() {
//initialise serial
  Serial.begin(9600);
  while (!Serial);

//initialise barometer
  if (!BARO.begin()) {
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
  }

//initialise altimeter
altitudeCorrection = initialiseAltimeter(launchAltitude); 

//initialise IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

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
writeToSerial(pressure, altitude, temperature, acceleration, currentState);

  // wait 1 second to print again
  delay(100);
}



int checkState(int state, float altitude, float acceleration){
switch (state) {
  case 0:
    if (acceleration >=1.5) { //we have lift off!
    state = 1;
    Serial.println("launch detected!");
    }
    return state;
    break;
  case 1:
    // are we still in powered flight?
    // if not move to state 2
    // case = 2
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

void writeToSerial(float pressure, float altitude, float temperature, float acceleration, int currentState){

  // print the pressure value
  Serial.print("Pressure = ");
  Serial.print(pressure);
  Serial.println(" kPa");

  // print the calculated altitude
  Serial.print("Altitude = ");
  Serial.print(altitude,1);
  Serial.println(" meters");

  // print the temperature value
  //Serial.print("Temperature = ");
  //Serial.print(temperature);
  //Serial.println(" C");

 // print the acceleration value
  //Serial.print("Accelaration = ");
  Serial.print(acceleration);
  //Serial.println(" G");

  //print the state

  //Serial.print("State = ");
  //Serial.println(currentState);

  // print an empty line
  Serial.println();

}

float readAcceleration(){
float x, y, z, acceleration;

 if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x,y,z); //only reading the x value, y & z silently ignored
  acceleration = x;
  }
return acceleration;

}

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

float readAltitude(float P0) {
  // Read the pressure value
  float pressure = BARO.readPressure();


  // Calculate altitude using the barometric formula
  float altitude = 44330 * (1.0 - pow(pressure / P0, 0.1903));

  // Apply altitude correction
  altitude += altitudeCorrection;

// record altiude values to suitable data structure here for use in checkState

return altitude;
}


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

