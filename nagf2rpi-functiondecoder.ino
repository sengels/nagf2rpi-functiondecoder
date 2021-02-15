// NMRA Dcc Function Decoder
//
// Author: Alex Shepherd 2019-03-30
// additional changes:
//         Patrick Spendrin 2021-01-12
// 
// This example requires these Arduino Libraries:
//
// 1) The NmraDcc Library from: http://mrrwa.org/download/
//
// These libraries can be found and installed via the Arduino IDE Library Manager
//
// This is a simple demo of how to drive and motor speed and direction using PWM and a motor H-Bridge
// It uses vStart and vHigh CV values to customise the PWM values to the motor response 
// It also uses the Headling Function to drive 2 LEDs for Directional Headlights
// Apart from that there's nothing fancy like Lighting Effects or a function matrix or Speed Tables - its just the basics...
//

#ifndef ARDUINO_AVR_ATTINYX5
#define ARDUINO_AVR_ATTINYX5 defined(__AVR_ATtiny85__)
#endif

#if not (defined(digitalPinToInterrupt) && defined(__AVR_ATtiny85__))
#define digitalPinToInterrupt(x) (0)
#endif

#include <NmraDcc.h>
// Uncomment any of the lines below to enable debug messages for different parts of the code
#define DEBUG_FUNCTIONS
//#define DEBUG_SPEED
//#define DEBUG_PWM
//#define DEBUG_DCC_ACK
//#define DEBUG_DCC_MSG

#if defined(DEBUG_FUNCTIONS) or defined(DEBUG_SPEED) or defined(DEBUG_PWM) or defined(DEBUG_DCC_ACK) or defined(DEBUG_DCC_MSG)
#define DEBUG_PRINT
#endif

// This is the default DCC Address
#define DEFAULT_DECODER_ADDRESS 3

// Uncomment if you want to use this as a "proper" decoder
//#define MOTOR_ENABLED

// set to 1 if you need a pullup resistor on the DCC_PIN
#define ENABLE_PULLUP 0

// This section defines the Arduino UNO Pins to use 
#ifdef __AVR_ATmega328P__ 

#define DCC_PIN     2

#define LED_PIN_F0_FWD 5
#define LED_PIN_F0_REV 6
#ifdef MOTOR_ENABLED
#define MOTOR_DIR_PIN 12
#define MOTOR_PWM_PIN 3
#else
#define LED_PIN_F1 9
#define LED_PIN_F2 8
#endif

// This section defines the Arduino ATTiny85 Pins to use 
#elif ARDUINO_AVR_ATTINYX5 

#define DCC_PIN     2

#define LED_PIN_F0_FWD 0
#define LED_PIN_F0_REV 1
#ifdef MOTOR_ENABLED
#define MOTOR_DIR_PIN 3
#define MOTOR_PWM_PIN 4
#else
#define LED_PIN_F1 3
#define LED_PIN_F2 4
#endif

#else
#error "Unsupported CPU, you need to add another configuration section for your CPU"
#endif 

// Some global state variables
uint8_t newLedState = 0;
uint8_t lastLedState = 0;
#ifndef MOTOR_ENABLED
uint8_t newLed1State = 0;
uint8_t lastLed1State = 0;
uint8_t newLed2State = 0;
uint8_t lastLed2State = 0;
#endif

uint8_t newDirection = 0;
uint8_t lastDirection = 0;

#ifdef MOTOR_ENABLED
uint8_t newSpeed = 0;
uint8_t lastSpeed = 0;
uint8_t numSpeedSteps = SPEED_STEP_128;

uint8_t vStart;
uint8_t vHigh;
#endif

// Structure for CV Values Table
struct CVPair
{
  uint16_t  CV;
  uint8_t   Value;
};

// CV Addresses we will be using
#define CV_VSTART  2
#define CV_VHIGH   5

// Default CV Values Table
CVPair FactoryDefaultCVs [] =
{
	// The CV Below defines the Short DCC Address
  {CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_DECODER_ADDRESS},
#ifdef MOTOR_ENABLED
  // Three Step Speed Table
  {CV_VSTART, 120},
  {CV_VHIGH, 255},
#endif

  // These two CVs define the Long DCC Address
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, 0},
  {CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, DEFAULT_DECODER_ADDRESS},

// ONLY uncomment 1 CV_29_CONFIG line below as approprate
//  {CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
  {CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
//  {CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION}, // Long  Address 28/128 Speed Steps  
};

NmraDcc  Dcc;

uint8_t FactoryDefaultCVIndex = 0;

// This call-back function is called when a CV Value changes so we can update CVs we're using
void notifyCVChange( uint16_t CV, uint8_t Value)
{
  switch(CV)
  {
#ifdef MOTOR_ENABLED
    case CV_VSTART:
      vStart = Value;
      break;
      
    case CV_VHIGH:
      vHigh = Value;
      break;
#endif
  }
}

void notifyCVResetFactoryDefault()
{
  // Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset 
  // to flag to the loop() function that a reset to Factory Defaults needs to be done
  FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs)/sizeof(CVPair);
};

// This call-back function is called whenever we receive a DCC Speed packet for our address 
void notifyDccSpeed( uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps )
{
  #ifdef DEBUG_SPEED
  Serial.print("notifyDccSpeed: Addr: ");
  Serial.print(Addr,DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? "-S" : "-L" );
  Serial.print(" Speed: ");
  Serial.print(Speed,DEC);
  Serial.print(" Steps: ");
  Serial.print(SpeedSteps,DEC);
  Serial.print(" Dir: ");
  Serial.println( (Dir == DCC_DIR_FWD) ? "Forward" : "Reverse" );
  #endif

  newDirection = Dir;
#ifdef MOTOR_ENABLED
  newSpeed = Speed;
  numSpeedSteps = SpeedSteps;
#endif
};

// This call-back function is called whenever we receive a DCC Function packet for our address 
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState)
{
  #ifdef DEBUG_FUNCTIONS
  Serial.print("notifyDccFunc: Addr: ");
  Serial.print(Addr, DEC);
  Serial.print( (AddrType == DCC_ADDR_SHORT) ? 'S' : 'L' );
  Serial.print("  Function Group: ");
  Serial.print(FuncGrp, DEC);
  #endif

  if(FuncGrp == FN_0_4)
  {
    newLedState = (FuncState & FN_BIT_00) ? 1 : 0;
#ifndef MOTOR_ENABLED
    newLed1State = (FuncState & FN_BIT_01) ? 1 : 0;
    newLed2State = (FuncState & FN_BIT_02) ? 1 : 0;
#endif
    #ifdef DEBUG_FUNCTIONS
    Serial.print(" FN 0: ");
    Serial.print(newLedState);
    #endif
  }
  #ifdef DEBUG_FUNCTIONS
  Serial.println();
  #endif
}

// This call-back function is called whenever we receive a DCC Packet
#ifdef  DEBUG_DCC_MSG
void notifyDccMsg( DCC_MSG * Msg)
{
  Serial.print("notifyDccMsg: ") ;
  for(uint8_t i = 0; i < Msg->Size; i++)
  {
    Serial.print(Msg->Data[i], HEX);
    Serial.write(' ');
  }
  Serial.println();
}
#endif

// This call-back function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
// So we will just turn the motor on for 8ms and then turn it off again.

void notifyCVAck(void)
{
  #ifdef DEBUG_DCC_ACK
  Serial.println("notifyCVAck") ;
  #endif

#ifdef MOTOR_ENABLED
  digitalWrite(MOTOR_DIR_PIN, HIGH);
  digitalWrite(MOTOR_PWM_PIN, HIGH);

  delay( 8 );  

  digitalWrite(MOTOR_DIR_PIN, LOW);
  digitalWrite(MOTOR_PWM_PIN, LOW);
#endif
}

void setup()
{
  #ifdef DEBUG_PRINT
  Serial.begin(115200);
  Serial.println("NMRA Dcc Multifunction Motor Decoder Demo");
  #endif

  // Setup the Pins for the Fwd/Rev LED for Function 0 Headlight
  pinMode(LED_PIN_F0_FWD, OUTPUT);
  pinMode(LED_PIN_F0_REV, OUTPUT);

#ifdef MOTOR_ENABLED
  // Setup the Pins for the Motor H-Bridge Driver
  pinMode(MOTOR_DIR_PIN, OUTPUT);
  pinMode(MOTOR_PWM_PIN, OUTPUT);
#else
  pinMode(LED_PIN_F1, OUTPUT);
  pinMode(LED_PIN_F2, OUTPUT);
#endif

#ifdef DEBUG_FUNCTIONS
  digitalWrite(LED_PIN_F0_FWD, HIGH);
  digitalWrite(LED_PIN_F0_REV, HIGH);
  digitalWrite(LED_PIN_F1, HIGH);
  digitalWrite(LED_PIN_F2, HIGH);
  delay(500);
  digitalWrite(LED_PIN_F0_FWD, LOW);
  digitalWrite(LED_PIN_F0_REV, LOW);
  digitalWrite(LED_PIN_F1, LOW);
  digitalWrite(LED_PIN_F2, LOW);
  delay(500);
  digitalWrite(LED_PIN_F0_FWD, HIGH);
  digitalWrite(LED_PIN_F0_REV, HIGH);
  digitalWrite(LED_PIN_F1, HIGH);
  digitalWrite(LED_PIN_F2, HIGH);
  delay(500);
#endif
  digitalWrite(LED_PIN_F0_FWD, LOW);
  digitalWrite(LED_PIN_F0_REV, LOW);
  digitalWrite(LED_PIN_F1, LOW);
  digitalWrite(LED_PIN_F2, LOW);

// Setup which External Interrupt, the Pin it's associated with that we're using and enable the Pull-Up 
  Dcc.pin(digitalPinToInterrupt(DCC_PIN), DCC_PIN, ENABLE_PULLUP);
  
  Dcc.init( MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0 );

  // Uncomment to force CV Reset to Factory Defaults
//  notifyCVResetFactoryDefault();

#ifdef MOTOR_ENABLED
  // Read the current CV values for vStart and vHigh
  vStart = Dcc.getCV(CV_VSTART);
  vHigh = Dcc.getCV(CV_VHIGH);
#endif
}

void loop()
{
  // You MUST call the NmraDcc.process() method frequently from the Arduino loop() function for correct library operation
  Dcc.process();

#ifdef MOTOR_ENABLED
  // Handle Speed changes
  if(lastSpeed != newSpeed)
  {
    lastSpeed = newSpeed;
    // Stop if speed = 0 or 1
    
    if(newSpeed <= 1) {
      digitalWrite(MOTOR_PWM_PIN, LOW);

    // Calculate PWM value in the range 1..255   
    } else {
      uint8_t vScaleFactor;
      
      if((vHigh > 1) && (vHigh > vStart))
        vScaleFactor = vHigh - vStart;
      else
        vScaleFactor = 255 - vStart;

      uint8_t modSpeed = newSpeed - 1;
      uint8_t modSteps = numSpeedSteps - 1;
      
      uint8_t newPwm = (uint8_t) vStart + modSpeed * vScaleFactor / modSteps;

      #ifdef DEBUG_PWM
      Serial.print("New Speed: vStart: ");
      Serial.print(vStart);
      Serial.print(" vHigh: ");
      Serial.print(vHigh);
      Serial.print(" modSpeed: ");
      Serial.print(modSpeed);
      Serial.print(" vScaleFactor: ");
      Serial.print(vScaleFactor);
      Serial.print(" modSteps: ");
      Serial.print(modSteps);
      Serial.print(" newPwm: ");
      Serial.println(newPwm);
      #endif
            
      analogWrite(MOTOR_PWM_PIN, newPwm);
    }
  }
#endif // MOTOR_ENABLED
  
  // Handle Direction and Headlight changes
  if((lastDirection != newDirection) || (lastLedState != newLedState))
  {
    lastDirection = newDirection;
    lastLedState = newLedState;

#ifdef MOTOR_ENABLED
    digitalWrite(MOTOR_DIR_PIN, newDirection);
#endif

    if(newLedState)
    {
      #ifdef DEBUG_FUNCTIONS
      Serial.println("LED On");
      #endif
      digitalWrite(LED_PIN_F0_FWD, newDirection ? LOW : HIGH);
      digitalWrite(LED_PIN_F0_REV, newDirection ? HIGH : LOW);
    }
    else
    {
      #ifdef DEBUG_FUNCTIONS
      Serial.println("LED Off");
      #endif
      digitalWrite(LED_PIN_F0_FWD, LOW);
      digitalWrite(LED_PIN_F0_REV, LOW);
    }
  }
#ifndef MOTOR_ENABLED
  if(lastLed1State != newLed1State) {
    lastLed1State = newLed1State;
    if(newLed1State)
    {
      #ifdef DEBUG_FUNCTIONS
      Serial.println("LED1 On");
      #endif
      digitalWrite(LED_PIN_F1, HIGH);
    }
    else
    {
      #ifdef DEBUG_FUNCTIONS
      Serial.println("LED1 Off");
      #endif
      digitalWrite(LED_PIN_F1, LOW);
    }
  }
  if(lastLed2State != newLed2State) {
    lastLed2State = newLed2State;
    if(newLed2State)
    {
      #ifdef DEBUG_FUNCTIONS
      Serial.println("LED2 On");
      #endif
      digitalWrite(LED_PIN_F2, HIGH);
    }
    else
    {
      #ifdef DEBUG_FUNCTIONS
      Serial.println("LED2 Off");
      #endif
      digitalWrite(LED_PIN_F2, LOW);
    }
  }
#endif

  // Handle resetting CVs back to Factory Defaults
  if( FactoryDefaultCVIndex && Dcc.isSetCVReady())
  {
    FactoryDefaultCVIndex--; // Decrement first as initially it is the size of the array 
    Dcc.setCV( FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
  }
}
