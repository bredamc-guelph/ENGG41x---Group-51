// Declare time variables

const unsigned long SECONDS = 1000;
const unsigned long MINUTES = 60 * SECONDS;

const unsigned long MAX_TIME_VIBRATION = 30 * SECONDS; // Update
const unsigned long MAX_TIME_HC = 15 * MINUTES;

// Debounce

const int debounceDelay = 200;

// Declare button pins

const byte E_STOP = 2; // check 1st
volatile bool stop_pressed = LOW; // track button state

const byte vibe_button = 19; // check 3rd
volatile bool vibe_pressed = LOW;

const byte hc_button = 3; // check 2nd
volatile bool hc_pressed = LOW; 

const byte function_button = 18; // check 4th
volatile bool function_pressed = LOW; 

// Declare e-stop variables

const int e_stop_pin = 51; // trigger e-stop indicator

// Declare vibration variables

const int vibe_pin = 29; // pin to toggle function

int vibe_state = 0; // 0 for off, 1 for on
unsigned long vibe_prevTime; // record start time for vibration function

// Declare heating/cooling variables

const int hc_togglePin = 47; // pin to toggle function
const int hc_functionPin = 43; // pin to switch between heating and cooling
const int cooling_LED = 42; // indicate when cooling is on
const int heating_LED = 39; // indicate when heating is on

int hc_state = 0; // 0 for off, 1 for on
int hc_function = 0; // 0 for cooling, 1 for heating
unsigned long hc_prevTime; // record start time for hc function

// Declare temperature sensor variables and libraries

#include <OneWire.h>
#include <DallasTemperature.h>

const int TEMP_BUS = 35;
OneWire oneWire(TEMP_BUS);
DallasTemperature sensors(&oneWire);
int numSensors;

const float heat_setpoint = 37.0; // in deg C
const float cool_setpoint = 9.0; // in deg C
const float heat_targetTemp = 36.0; // in deg C
const float cool_targetTemp = 12.0; // in deg C
const float max_heat = 40.0; // start cooling if max temp reached * test how long it takes to cool down after turning on
int maxTempReached = 0;
int targetTempReached = 0;

void setup() {
  pinMode(e_stop_pin, OUTPUT);
  pinMode(vibe_pin, OUTPUT);
  pinMode(hc_functionPin, OUTPUT);
  pinMode(hc_togglePin, OUTPUT);
  pinMode(cooling_LED, OUTPUT);
  pinMode(heating_LED, OUTPUT);

  // connect buttons between pins and ground
  pinMode(E_STOP, INPUT_PULLUP);
  pinMode(vibe_button, INPUT_PULLUP);
  pinMode(hc_button, INPUT_PULLUP);
  pinMode(function_button, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(E_STOP), checkESTOP, FALLING);
  attachInterrupt(digitalPinToInterrupt(vibe_button), checkVibration, FALLING);
  attachInterrupt(digitalPinToInterrupt(hc_button), checkHC, FALLING);
  attachInterrupt(digitalPinToInterrupt(function_button), checkFunction, FALLING);

  sensors.begin();
  numSensors = sensors.getDeviceCount();

  Serial.begin(9600);
  Serial.print("Sensors found: ");
  Serial.println(numSensors);
  Serial.println("Start!");
}

void loop() {  
  // check if e-stop has been pressed
  if (stop_pressed){
    Serial.println("E-STOP PRESSED!");
    
    digitalWrite(hc_togglePin, LOW);
    hc_state = 0;
    digitalWrite(vibe_pin, LOW);
    vibe_state = 0;
    hcLED(hc_state);
    digitalWrite(e_stop_pin, HIGH);
    
    delay(10 * SECONDS);
    digitalWrite(e_stop_pin, LOW);
    Serial.println("Reset complete");
    
    stop_pressed = !stop_pressed;
  }

  // check if heating/cooling toggle has been pressed
  if (hc_pressed){ // check if hc toggle button is pressed
    if (!hc_state) { // turn on hc function if off
      digitalWrite(hc_togglePin, HIGH);
      hc_state = 1; // store state as on
          
      Serial.println("Run Heating/Cooling");
    }
    else if (hc_state){ // turn off hc function if on
      digitalWrite(hc_togglePin, LOW);
      hc_state = 0; // store state as off
      targetTempReached = 0; // ensures timer can start again
    
      Serial.println("End Heating/Cooling (switch hit)");
    }
    
    hc_pressed = !hc_pressed;
  }

  // check if vibration toggle button has been pressed
  if (vibe_pressed){ 
    if (!vibe_state){ // turn on vibration function if off
      digitalWrite(vibe_pin, HIGH);
      vibe_prevTime = millis();
      vibe_state = 1; // store state as on
      
      Serial.println("Run Vibration");
    }
    else if (vibe_state){ // turn off vibration function if on
      digitalWrite(vibe_pin, LOW);
      vibe_state = 0; // store state as off
    
      Serial.println("End Vibration (switch hit)");
    }
    
    vibe_pressed = !vibe_pressed;
  }

  // check if heating/cooling function button has been pressed
  if (function_pressed){ 
    if (!hc_function){ // switch to heating if previously cooling
      digitalWrite(hc_functionPin, HIGH);
      hc_function = 1; // store state as heating
  
      Serial.println("Switched to heating");
    }
    else if (hc_function){ // switch to cooling if previously heating
      digitalWrite(hc_functionPin, LOW);
      hc_function = 0; // store state as cooling
  
      Serial.println("Switched to cooling");
    }
    
    function_pressed = !function_pressed;
  }

  // if vibration is running, check if time limit has been reached
  if ((vibe_state) and (millis() - vibe_prevTime > MAX_TIME_VIBRATION)){
    digitalWrite(vibe_pin, LOW);
    vibe_state = 0; // store state as off
    
    Serial.println("End Vibration (time limit)");
  }

  // check if heating/cooling is running
  if (hc_state){ 
    if (tempOutOfRange()){ // deactivate relay if temp not within range
      digitalWrite(hc_togglePin, LOW);
      //Serial.println("Temperature out of range, pause heating/cooling.");
    }

    // note: set second range for when to reactivate??
    else if (!tempOutOfRange()){ // activate relay if temp within range
      digitalWrite(hc_togglePin, HIGH);
    }
    if ((millis() - hc_prevTime > MAX_TIME_HC) and targetTempReached){ // check if hc time limit reached 
      digitalWrite(hc_togglePin, LOW);
      hc_state = 0; // store state as off
      targetTempReached = 0; // ensures timer can start again

      Serial.println("End Heating/Cooling (time limit)");
    }
  }

  hcLED(hc_state); // update hc LED indicators
}


// button interrupts
void checkESTOP(){
  static unsigned long lastStopTime = 0; // define once
  unsigned long currStopTime = millis(); // update each time

  if (currStopTime - lastStopTime > debounceDelay){
    if (stop_pressed){
      Serial.println("Wait until reset!");
    }
    else {
      stop_pressed = !stop_pressed;
    }
  }
  lastStopTime = currStopTime;
}

void checkVibration(){
  static unsigned long lastVibeTime = 0; // define once - only for interrupt
  unsigned long currVibeTime = millis(); // update each time

  if (currVibeTime - lastVibeTime > debounceDelay){
    if (stop_pressed){
      Serial.println("Wait until reset!");
    }
    else {
      vibe_pressed = !vibe_pressed;
    }
  }
  lastVibeTime = currVibeTime;
}

void checkHC(){
  static unsigned long lastHCTime = 0; // define once
  unsigned long currHCTime = millis(); // update each time

  if (currHCTime - lastHCTime > debounceDelay){
    if (stop_pressed){
      Serial.println("Wait until reset!");
    }
    else {
      hc_pressed = !hc_pressed;
    }
  }
  lastHCTime = currHCTime;
}

void checkFunction(){
  static unsigned long lastFuncTime = 0; // define once
  unsigned long currFuncTime = millis(); // update each time

  if (currFuncTime - lastFuncTime > debounceDelay){
    if (stop_pressed){
      Serial.println("Wait until reset!");
    }
    else if (hc_state){
      Serial.println("Shut off before changing setting!");
    }
    else {
      function_pressed = !function_pressed;
    }
  }
  lastFuncTime = currFuncTime;
}

// other functions

int tempOutOfRange(){ // return true if temperature is out of range
  sensors.requestTemperatures();
  static float temp = 0;
  float minTemp = 20.0; // initialize to these values bc only values above and below will affect function
  float maxTemp = 20.0;

  for (int i = 0; i < numSensors; i++){
    temp = sensors.getTempCByIndex(i);
    Serial.print("Sensor "); Serial.print(i+1); Serial.print(" : ");
    Serial.print(temp); Serial.println("*C");
    if (temp > maxTemp){
      maxTemp = temp;
    }
    if (temp < minTemp){
      minTemp = temp;
    }

    if (hc_function){ // if heat on
      if (!targetTempReached and temp > heat_targetTemp){ // start timer when target temp reached
        targetTempReached = 1;
        hc_prevTime = millis();
        Serial.println("Target temperature reached, start timer.");
        return 0;
      }
      if (temp > max_heat){ // activate cooling if max temp reached
        digitalWrite(hc_functionPin, LOW); // heating circuit will aready be off when this flips
        maxTempReached = 1; // track if saftey cooling is running
        return 0;
      }
      if (temp > heat_setpoint){ // deactivate heating if setpoint reached
        return 1;
      }
    }

    if (!hc_function){ // if cool on
      if (!targetTempReached and temp < cool_targetTemp){ // start timer when temp reached
        targetTempReached = 1;
        hc_prevTime = millis();
        return 0;
      }
      if (temp < cool_setpoint){ // deactivate cooling if setpoint reached
        return 1;
      }
    }
  }

  if (maxTemp < max_heat and maxTempReached and hc_function){ // deactivate cooling if under max temp only after reaching max temp
    digitalWrite(hc_togglePin, LOW); // deactivate plates
    digitalWrite(hc_functionPin, HIGH); // reactivate heating function
    maxTempReached = 0;
    return 1;
  }

  return 0;
}

void hcLED(int off_on){ // 0 for off, 1 for on
  if (hc_function){ // 1 for heating
    digitalWrite(heating_LED, off_on);
  }
  else{
    digitalWrite(cooling_LED, off_on);
  }
}
