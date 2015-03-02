/*

Line Follower control using Bang-Bang algorithm
Written 3/2/15 by Walter Coe, BYU Mechatronics Club
For more explaination on Bang-Bang control, see here...
http://en.wikipedia.org/wiki/Bang%E2%80%93bang_control

Ecplaination for use of this code...
    Use two sensors positioned so that they straddle the line to be followed
    The assumed sensor is a photo-transistor sensor (i.e. QRD1114 or similar)
    There must be a threshold determined by the user, i.e. the value of variable 's_thresh'
        Builder must verify that the sensor theshold is not met by both sensors simultaniously


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
const int motor_L = 3;
const int motor_R = 4;

//Define the sensor threshold
const float s_thresh = .25;

//Variable used to to store sensor data
int turn;

//Holds time of last line detection
//Used to avoid runaway robots
long t_last;
const int t_thresh = 10000;
const int pause = 10000;

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
     delay(pause); 
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
  }
  
  //Turn left
  else if( turn_dir == 1)
  {
     analogWrite(motor_L, motor_low);
     analogWrite(motor_R, motor_avg); 
  }
  
  //Go straight
  else
  {
     analogWrite(motor_L, motor_avg);
     analogWrite(motor_R, motor_avg);
  }
  
}



