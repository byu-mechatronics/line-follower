/*

Line Follower control using Bang-Bang algorithm
Written 3/2/15 by Walter Coe, BYU Mechatronics Club
For more explanation on Bang-Bang control, see here...
http://en.wikipedia.org/wiki/Bang%E2%80%93bang_control

This code is written for Arduino

Explanation for use of this code...

    Use two sensors positioned so that they straddle the line to be followed
    The assumed sensor is a photo-transistor sensor (i.e. QRD1114 or similar)
    There must be a threshold determined by the user, i.e. the value of variable 's_thresh'
    Builder must verify that the sensor threshold is not met by both sensors simultaneously


	An easy and useful addition to this code would include 'motor_high' and 'motor_stop' variables and
	time how long the follower has seen the line with a given sensor. If the follower sees the line 
	for too long, then add speed to one wheel and stop the other to make a tighter turn.
	
*/

//Define Sensor Pins
//A0 is on the left
//A1 is on the right
const int s0 = A0;
const int s1 = A1;


//Motor Constraints
//Set motor_low to lowest duty cycle % that turns the motor
const float motor_low = .25;
const float motor_avg = .75;

//Define motor drive pins
//Must be chosen for your wiring, connect these pins to switches(MOSFET of BJT)
const int motor_L = 3;	//Pin for PWM. Could be 3, 5, 6, 9, 10, or 11.
const int motor_R = 5;	//Pin for PWM. Could be 3, 5, 6, 9, 10, or 11.

//Define the sensor threshold
const float s_thresh = .25;

//Variable used to to store sensor data
int turn;

//Holds time of last line detection
//Used to avoid runaway robots
long t_last;

//Time in millis allowed between line sightings
//	If the line isn't seen, the follower will pause for the defined time
const int t_thresh = 10000;	//If you don't see the line for 10 seconds (10000 milliseconds), stop.
const int pause = 10000;	//Pause for 10 seconds, then start again.

//----------------------------------------------------------------------------------------------------------------------------

void setup()
{
  //Setup serial terminal printing
  Serial.begin(9600);
  
  //Setup the pins that will drive the switches (MOSFET or BJT)
  pinMode(motor_L, OUTPUT);
  pinMode(motor_R, OUTPUT);

 //Initialize
  turn = 0;
  
  //Initialize
  t_last = millis();
}

void loop()
{
  //Check for the line
	turn = line_check(s0, s1);
  
  //Based on sensor reading, turn or go straight
	drive_motor(turn, motor_L, motor_R, motor_low, motor_avg);
  
  //If you saw the line, reset timer
	if( turn != 0)
	{
		t_last = millis(); 
	}
  
  //If the line hasn't been seen, stop for some time
  if( millis() - t_last > t_thresh )
    {
		Serial.println("Line has not been seen in a while... I think I'm lost!");
		delay(pause);
		t_last = millis(); 
    }
    
}

//----------------------------------------------------------------------------------------------------------------------------

int line_check(int s0, int s1)
{
   
  //Line seen by left sensor
   if(analogRead(s0) <= s_thresh)
   {
     return(0); 
   }
   
   //Line seen by right sensor
   else if(analogRead(s1) <= s_thresh)
   {
     return(1);
   }
   
   //Line not seen
   else
   {
     return(-1); 
   }
  
}

void drive_motor(int turn_dir, int motor_L, int motor_R, float motor_low, float motor_avg)
{
  //Turn right
  if( turn_dir == 0 )
  {
     analogWrite(motor_L, motor_avg);
     analogWrite(motor_R, motor_low);
	 Serial.println("Turn Right");
  }
  
  //Turn left
  else if( turn_dir == 1)
  {
     analogWrite(motor_L, motor_low);
     analogWrite(motor_R, motor_avg);
	 Serial.println("Turn Left");
  }
  
  //Go straight
  else
  {
     analogWrite(motor_L, motor_avg);
     analogWrite(motor_R, motor_avg);
	 Serial.println("Straight Away!");
  }
  
}



