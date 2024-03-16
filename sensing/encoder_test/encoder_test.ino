/********************************************************
 * 
 * CODE USED TO MEASURE WHEEL VELOCITY
 * TWO WHEELS CONTROLLED IN OPEN LOOP BY A POTENTIOMETER
 * USED TO DEBUG YOUR OPTICAL INCREMENTAL ENCODER
 * 
 ********************************************************/

// encoder pin
int encoderPinDx=2;
int encoderPinSx=3;

// motor SX pins
int ENA = 6;
int IN1 = 5;
int IN2 = 4;

// motor DX pins
int ENB = 9;
int IN3 = 7;
int IN4 = 8;
 
// Sensing using encoder with 20 holes (angle/pulse =360/20 = 18 degrees per pulse)
volatile int anglePerPulse=18;

// total pulses from start of FW
volatile unsigned long totalPulsesDx = 0;
volatile unsigned long totalPulsesSx = 0;

// total angle calculated from totalPulses
volatile long totalAngleDx=0;
volatile long totalAngleSx=0;

// variables for measuring time
volatile unsigned long lastTimeDx=0;
volatile unsigned long currentTimeDx=0;
// deltaT = currentTime-lastTime
volatile double deltaTDx=0; 

// variables for measuring time
volatile unsigned long lastTimeSx=0;
volatile unsigned long currentTimeSx=0;
// deltaT = currentTime-lastTime
volatile double deltaTSx=0; 
 
 
// angular velocity calculation
volatile double angularVelocityDx=0;
// total time until sumPulses reaches averageSample
volatile unsigned long timeAverageAngularVelocityDx=0;
// sumPulses goes from 0 to averageSample, and then it is set back to zero
volatile int sumPulsesDx=0;
// how many pulses we will take to calculate the average angular velocity
volatile int averageSampleDx=50;

volatile double angularVelocitySx=0;
// total time until sumPulses reaches averageSample
volatile unsigned long timeAverageAngularVelocitySx=0;
// sumPulses goes from 0 to averageSample, and then it is set back to zero
volatile int sumPulsesSx=0;
// how many pulses we will take to calculate the average angular velocity
volatile int averageSampleSx=50;
 
void setup() {
  // set the motor pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
 
  // set the encoder pin
  pinMode(encoderPinDx, INPUT);
  pinMode(encoderPinSx, INPUT);
  
  // attach the interrupt for tracking the pulses
  attachInterrupt(digitalPinToInterrupt(encoderPinSx), interruptFunctionSx, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinDx), interruptFunctionDx, RISING);
 
  Serial.begin(9600);
  
  // obtain the current time
  lastTimeDx=millis();
  lastTimeSx=millis();
}
 
void loop() {
  int potValue = analogRead(A0);

  // Mappa il valore del potenziometro (0-1023) a un valore PWM (0-255)
  int mappedValue = map(potValue, 0, 1023, 0, 255);
  
  // set the motor speed
  analogWrite(ENA, mappedValue);
  // direction
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2, LOW);
  
  // set the motor speed
  analogWrite(ENB, mappedValue);
  // direction
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4, LOW);
}
 
void interruptFunctionDx(){
        // get the current time
        currentTimeDx=millis();
        // time among interrupts
        deltaTDx=currentTimeDx-lastTimeDx;
        lastTimeDx=currentTimeDx;
 
        // increment the total number of pulses
        totalPulsesDx = totalPulsesDx + 1;
        
        // calculate the total angle
        totalAngleDx = totalPulsesDx*anglePerPulse;
 
        // this is another sum of pulses that is used to calculate
        // the average angular velocity
        // this sum is set to zero after averageSample
        sumPulsesDx=sumPulsesDx+1;
        // track the total time between computing the averages
        timeAverageAngularVelocityDx=timeAverageAngularVelocityDx+deltaTDx;
 
       // here we calculate the average angular velocity
       if (sumPulsesDx>=averageSampleDx)
       {
          angularVelocityDx=1000*((double)(sumPulsesDx * anglePerPulse) )/((double) timeAverageAngularVelocityDx );
          Serial.println(angularVelocityDx);
          sumPulsesDx=0;
          timeAverageAngularVelocityDx=0;
       }  
}

void interruptFunctionSx(){
        // get the current time
        currentTimeSx=millis();
        // time among interrupts
        deltaTSx=currentTimeSx-lastTimeSx;
        lastTimeSx=currentTimeSx;
 
        // increment the total number of pulses
        totalPulsesSx = totalPulsesSx + 1;
        
        // calculate the total angle
        totalAngleSx = totalPulsesSx*anglePerPulse;
 
        // this is another sum of pulses that is used to calculate
        // the average angular velocity
        // this sum is set to zero after averageSample
        sumPulsesSx=sumPulsesSx+1;
        // track the total time between computing the averages
        timeAverageAngularVelocitySx=timeAverageAngularVelocitySx+deltaTSx;
 
       // here we calculate the average angular velocity
       if (sumPulsesSx>=averageSampleSx)
       {
          angularVelocitySx=1000*((double)(sumPulsesSx * anglePerPulse) )/((double) timeAverageAngularVelocitySx );
         //Serial.println(angularVelocitySx);
          //Serial.print(",");
          //Serial.println(angularVelocityDx);
          sumPulsesSx=0;
          timeAverageAngularVelocitySx=0;
       }  
}
