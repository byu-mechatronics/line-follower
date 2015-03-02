/*
Line Follower utilizing a PID algorithm
Written 2/28/15 by Walter Coe, BYU Mechatronics Club
A great explanation of PID can be found here...
http://www.pc-control.co.uk/feedback_control.htm
 - -----------> +
|-|     s      |-|
| |------------| |
|-|    |||     |-|
       |||
       |||
  Assumptions for using this code....
  Using one photo-transistor sensor (i.e. QRD1114 or similar)
  The sensor will need to be tuned, by choosing the pull-up resistor
    A bigger resistor will make for a faster transition between 0-5v output, but will result in a narrow detection range
    A smaller resistor will give a slower (possibly better) transition, resulting in a larger range
  kp, ki, kd must be selected by individual user
  There is no "wait for user input" before it starts after powering up
    You may want to consider waiting for a button push of setting a 2 second delay before it moves
  
  1) Place the line follower on a dark-light transition with the light on the right
  2) Connect power (can be done with a switch)
  3) Arduino will wait 2 seconds before starting to follow the line
  4) If the Arduino moves to fast to react to the line, you can slow the speed by lowering the motor_avg value
sensor reads light (higher value output from 'pos' equation) when it needs to turn left
sensor reads dark (lower value output from 'pos' equation) where it needs to turn right
*/

//Define Sensor Pins
const int s1 = A0;

//Theoretical Max for PID value
const float PID_max = 2.5;

//Define PID gains
const float kp = 1.0;
const float kd = 1.0;
const float ki = 0.1;

//Motor Constraints
//Set motor_low to lowest duty cycle % that turns the motor
const float motor_low = .25;
const float motor_avg = .75;

//Used for digital filtering
const float tau = .005;

//When to turn on/off integrator?
//Using an integrator when error is large may result in an unstable system
const float intLimit = .25;

//Define motor drive pins
//Must be chosen for your wiring, connect these pins to switches(MOSFET of BJT)
const int motor_L = 3;
const int motor_R = 4;

//What is the maximum pos value while still 'seeing' the line
const float e_thresh = .95;

//Store last time derivative was calculated. For PID
//The "_d1" is a d "one", not a lower case L
float time_d1;

//Tracks position and error, changes
//The "_d1" is a d "one", not a lower case L
float pos;
float pos_d1;
float posError;
float posError_d1;
float Ts;

//Variables that hold current values for PID
float differentiator;
float integrator;

//Output variable telling drive_motor() where to go...
float pid_out;

//Initial line position
float ref;

//-----------------------------------------------------------------------------------------------------------------------------

void setup()
{
  //Setup serial terminal printing
  Serial.begin(9600);
  
  //Setup the pins that will drive the switches (MOSFET or BJT)
  pinMode(motor_L, OUTPUT);
  pinMode(motor_R, OUTPUT);
  
  //Record current time for use in initial differentiation and integration
  time_d1 = millis();
  
  //Initial values to 0
  differentiator = 0;
  integrator = 0;
  posError_d1 = 0;
  
  //Read line's initial position and set it as home position
  //Sensor (in our specific configuration) outputs close to 1023 (2^10) when there is no
  // reflection and close 0 when there is reflection (the line being black reflects very
  // little). This line sets 'ref' to a value between 0 and 1, 0 when there is little reflection
  // and 1 when there is lots of reflection... 
  ref = 1 - (analogRead(s1)/1023.0);
}

//-----------------------------------------------------------------------------------------------------------------------------

void loop()
{
  //Determine where the line is relative to the car.
  // Value between 0 and 1, higher is 
  pos = 1 - (analogRead(s1)/1023.0);
 
 //for debugging, uncomment the three lines below and open the terminal by clicking the magnifying
 //glass in the upper right hand corner of the window with your Arduino plugged in.
/* 
  Serial.print(ref);
  Serial.print("Pos");
  Serial.println(pos);
*/  
  
  //As long as the sensor is detecting the line still, proceed
  if ( pos < e_thresh )
  {
    //If error is negative, we are to far right (must turn left)
    //If error is positive, we are to far left (must turn right
    posError = ref - pos ;
      
      //dT in seconds, used for integration and differentiation
      Ts = (millis() - time_d1)*1000.0;
      
      //To far left when negative
      //To far right when positive
      differentiator = (2*tau - Ts)/(2*tau+Ts)*differentiator + (2/(2*tau+Ts))*(posError_d1 - posError);
      
        
      //If Integrating is safe      
      if (posError < intLimit)
      {
      //Negative when negative (forward path loop)  
      integrator = integrator + (Ts/2)*(posError + posError_d1);
      }
      
      //Don't integrate
      else
      {
       integrator = 0; 
      }
      
      //Turn left when negative
      //Turn right when positive
      pid_out = kp*posError - kd*differentiator + ki*integrator; 
      
 //for debugging, uncomment the three lines below and open the terminal by clicking the magnifying
 //glass in the upper right hand corner of the window with your Arduino plugged in. 
     /*
      Serial.print("\n\n");
      Serial.print("The raw PID output is ");
      Serial.println(pid_out);
      Serial.print("\n\n");
     */
      
      //Function that interprets pid_out value and drives motors
      drive_motor(pid_out, PID_max, motor_low, motor_avg, motor_L, motor_R);
      
      //Set current values to the "delayed by one" values for use in next loop
      time_d1 = millis();
      posError_d1 = posError;
      pos_d1 = pos;
  }
  
  //If the line is not 'seen' by the sensor, then just stop driving
  //This is mostly so you don't have to chase down a rouge line-follower
  else
  {
    analogWrite(motor_L, 0);
    analogWrite(motor_R, 0);
  }
  
}

//-----------------------------------------------------------------------------------------------------------------------------

//Input will be between +-PID_max 
//      *note: You will need to adjust PID_max to the actual maximum value it outputs
//             This value will change depending on the sensor, distance to line, line characteristics
//             and the values of Kp, Kd, and Ki. It is recommended to use Serial.print to see the actual
//             values output by the PID equation above.
//
//    Turn left when PID is negative
//    Turn right when PID is positive
//    Otherwise, go straight
void drive_motor(float PID, float PID_max, float pwm_min, float pwm_avg, int motorL, int motorR)
{
 
  float PWM_L;
  float PWM_R;
  
 if (PID == 0.0)
 {
   PWM_L = pwm_avg;
   PWM_R = pwm_avg; 
 }
 
 //PID is positive, turn right
 else if ( PID > 0.0 )
 {
   PWM_R = pwm_avg + (abs(PID)/PID_max)/2.0;
   PWM_L = pwm_avg - (abs(PID)/PID_max)/2.0;
 }
 
 //PID is negative, turn left
 else
 {
   PWM_R = pwm_avg - (abs(PID)/PID_max)/2.0;
   PWM_L = pwm_avg + (abs(PID)/PID_max)/2.0;
 }
  
 if (PWM_L < pwm_min)
 {
   PWM_L = pwm_min;
 }
 
 if (PWM_L > 1.0)
 {
  PWM_L = 1.0; 
 }
 
 if (PWM_R < pwm_min)
 {
   PWM_R = pwm_min;
 }
 
 if (PWM_R > 1.0)
 {
  PWM_R = 1.0; 
 }
 
 analogWrite( motorL, 255.0 * PWM_L);
 analogWrite( motorR, 255.0 * PWM_R);
    
 Serial.println(PWM_L);
 Serial.println(PWM_R);
    
}
