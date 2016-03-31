#include <PID_v1.h> //PID library
#include <uSTimer2.h>
#include <I2CEncoder.h>
#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_FrontRightMotor;
Servo servo_FrontLeftMotor;
Servo servo_BackLeftMotor;
Servo servo_BackRightMotor;

Servo servo_BaseArmMotor;
Servo servo_TopArmMotor;
Servo servo_GripMotor;

Servo left_grip_servo; //these two are for the grip to open and close
Servo right_grip_servo;

I2CEncoder encoder_FrontRightMotor;
I2CEncoder encoder_FrontLeftMotor;
I2CEncoder encoder_BackRightMotor;
I2CEncoder encoder_BackLeftMotor;
I2CEncoder encoder_GripMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_LINE_TRACKERS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_LINE_TRACKER_CALIBRATION
//#define DEBUG_MOTOR_CALIBRATION

boolean bt_Motors_Enabled = true;

//port pin constants
const int ci_Mode_Button = 7000;                                     //place the port pins here (ultrasonic, motors)
const int ci_FrontRight_Motor = 12;
const int ci_FrontLeft_Motor = 13;
const int ci_BackRight_Motor = 10;
const int ci_BackLeft_Motor = 11;


//problem pins are 4 and 5
const int ci_Front_Ultrasonic_Data = 3;
const int ci_Back_Ultrasonic_Data = 5; //DONT CHANGE
const int ci_Left_Ultrasonic_Data = 7;
const int ci_Right_Ultrasonic_Data = 8;

const int ci_Front_Ultrasonic_Ping = 2;
const int ci_Back_Ultrasonic_Ping = 4; //DONT CHANGE
const int ci_Left_Ultrasonic_Ping = 6;
const int ci_Right_Ultrasonic_Ping = 9;

const int ci_Grip_Motor = 11;
//const int ci_Motor_Enable_Switch = 12;
const int ci_Right_Line_Tracker = A0;
const int ci_Middle_Line_Tracker = A1;
const int ci_Left_Line_Tracker = A2;
const int ci_Light_Sensor = A3;
const int ci_I2C_SDA = A4;         // I2C data = white
const int ci_I2C_SCL = A5;         // I2C clock = yellow
//<<< <<< < HEAD
//== == == =



//might have to go on seperate board
int ISRPin = 13;




// Charlieplexing LED assignments
const int ci_Heartbeat_LED = 1;
const int ci_Indicator_LED = 10;
/*const int ci_Right_Line_Tracker_LED = 6;
  const int ci_Middle_Line_Tracker_LED = 9;
  const int ci_Left_Line_Tracker_LED = 12;*/

//constants

// EEPROM addresses

const int ci_Front_Left_Motor_Offset_Address_L = 12;
const int ci_Front_Left_Motor_Offset_Address_H = 13;
const int ci_Front_Right_Motor_Offset_Address_L = 14;
const int ci_Front_Right_Motor_Offset_Address_H = 15;

const int ci_Back_Left_Motor_Offset_Address_L = 16;
const int ci_Back_Left_Motor_Offset_Address_H = 17;       //not sure if 16-19 are correct
const int ci_Back_Right_Motor_Offset_Address_L = 18;
const int ci_Back_Right_Motor_Offset_Address_H = 19;

//used for line tracking code (mode 1)
boolean pauseHere;

int stageCounter = 0;
const int ci_Front_Left_Motor_Stop = 1500;        // 200 for brake mode; 1500 for stop
const int ci_Front_Right_Motor_Stop = 1500;
const int ci_Back_Left_Motor_Stop = 1500;
const int ci_Back_Right_Motor_Stop = 1500;

const int ci_Grip_Motor_Stop = 1500;
const int ci_Grip_Motor_Open = 180;         // Experiment to determine appropriate value
const int ci_Grip_Motor_Zero = 80;          //  "
const int ci_Grip_Motor_Closed = 80;       //  "

const int ci_BaseArm_Servo_Retracted = 55;      //  "
const int ci_BaseArm_Servo_Extended = 120;      //  "

const int ci_TopArm_Servo_Retracted = 55;      //  "
const int ci_TopArm_Servo_Extended = 120;

const int ci_Display_Time = 500;
/*const int ci_Line_Tracker_Calibration_Interval = 100;
  const int ci_Line_Tracker_Cal_Measures = 20;
  const int ci_Line_Tracker_Tolerance = 100; // May need to adjust this
  const int ci_Motor_Calibration_Time = 5000;*/

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long ul_Echo_Time;
/*unsigned int ui_Left_Line_Tracker_Data;
  unsigned int ui_Middle_Line_Tracker_Data;
  unsigned int ui_Right_Line_Tracker_Data;*/
unsigned int ui_Motors_Speed = 1900;        // Default run speed

unsigned int ui_Front_Left_Motor_Speed;
unsigned int ui_Front_Right_Motor_Speed;
unsigned int ui_Back_Right_Motor_Speed;
unsigned int ui_Back_Left_Motor_Speed;

unsigned long ul_Front_Left_Motor_Position;
unsigned long ul_Front_Right_Motor_Position;
unsigned long ul_Back_Left_Motor_Position;
unsigned long ul_Back_Right_Motor_Position;

unsigned long ul_Grip_Motor_Position;

unsigned long ul_3_Second_timer = 0;
unsigned long ul_Display_Time;
unsigned long ul_Calibration_Time;

unsigned long ui_Front_Left_Motor_Offset;
unsigned long ui_Front_Right_Motor_Offset;
unsigned long ui_Back_Left_Motor_Offset;
unsigned long ui_Back_Right_Motor_Offset;

/*unsigned int ui_Cal_Count;
  unsigned int ui_Left_Line_Tracker_Dark;
  unsigned int ui_Left_Line_Tracker_Light;
  unsigned int ui_Middle_Line_Tracker_Dark;
  unsigned int ui_Middle_Line_Tracker_Light;
  unsigned int ui_Right_Line_Tracker_Dark;
  unsigned int ui_Right_Line_Tracker_Light;
  unsigned int ui_Line_Tracker_Tolerance;*/

unsigned int  ui_Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  ui_Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  ui_Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {
  1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 65536
};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;
boolean grab_object_mode = false; //used for lab04, sets robot into target aquisition mode
boolean target_aquired = false; //usef for lab04, to determine which stage of target aquisition bot is in
boolean coarse_target_direction = false; //used for lab04, coarse grain direction to target

int x_degrees_turn_position = 10; //value associated with reading a 90 degree turn
int light_sensor_data = 0; //globabl variable to hold light sensor output data
int light_tolerance = 0; //might need to change this
int light_sensor_bright = 300;
int distance_to_object = 10; //EXPERIMENTAL VALUE
int arm_target_length = 20; //value of arm length
unsigned long max_light_position = 0;
int which_case = 0;
unsigned long current_position = 0;


/////////////////////////////////////////////////////////////
//JULIAN'S GLOBAL VARIABLES
/////////////////////////////////////////////////////////////
//for opening and closing the grip
int open_pos = 40; //angle associated with claw open
int closed_pos = 160; //angle associated with claw closed

//for navigation
int current_pos[3]; //0th position is x coordinate, 1st is y, 2nd is directionality register
int last_known_cube_pos [3];
int home_pos[2]; //no directuionality b/c we know always pointing in the positive direction
//just x and y coordinates {[x][y]}
int cubesCollected; //keeps track of how many cubes have been collected
const int sideLength = 200; //total side length of track in cm


//for pinging:
int cmFront, cmBack, cmLeft, cmRight; //variables hold distances of ultrasonic sensors
int rightDuration, leftDuration, frontDuration, backDuration; //used to send a length of pings out

const int delayTime = 10; //milisecond time

//for checking cube:
const int NOFIELD = 0; //have to change this


int numberOfPasses; //keeps track of how many times we've driven in the y-direction

int distance[7]; //array to hold the distances coming serially from board 2


///////////////
//PID CONTROL SYSTEM VARIABLES
double Setpoint, Input, Output;
//pass pointers so we the pid can modify and read updated values
PID motorControl(&Input, &Output, &Setpoint, 2, 5, 1, DIRECT); //the values are P, I, D, currently are default, we need to tune these



void setup() {

  ////////////////////////
  //SETTING UP PID CONTROL
  ////////////////////////

  motorControl.SetMode(AUTOMATIC);
  motorControl.SetSampleTime(50); //pid updates every 50ms (default is 200, Arduino recommends shorter for robotics)
  Setpoint = 0;



  ///////////////////////
  //setting up ISR
  //////////////////////
  pinMode(ISRPin, OUTPUT);
  // attachInterrupt(digitalPinTOInterrupt(ISRPin), CheckCube(), RISING); //setting up ISR from LOW to HIGH on ISRPin



  pauseHere = true;

  /////////////////////////////////////////////////
  //initializing navigational functions and variables
  cubesCollected = 0;
  numberOfPasses = 1;
  //initPos(); //initialze the home position only when the robot starts each round
  current_pos[2] = 0; //sets the direction to positive y orientation
  // TURN 180 FUNCTION SHOULD FLIP THIS VALUE TO 1
  //TO INDICATE TRAVELLING IN THE NEGATIVE DIRECTION
  //cubeFound = false;
  /////////////////////////////////////////////////////




  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);



  // set up ultrasonic
  pinMode(ci_Front_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Front_Ultrasonic_Data, INPUT);

  pinMode(ci_Back_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Back_Ultrasonic_Data, INPUT);

  pinMode(ci_Left_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Left_Ultrasonic_Data, INPUT);

  pinMode(ci_Right_Ultrasonic_Ping, OUTPUT);
  pinMode(ci_Right_Ultrasonic_Data, INPUT);


  // set up drive motors, need to reinitialize names
  pinMode(ci_FrontRight_Motor, OUTPUT);
  servo_FrontRightMotor.attach(ci_FrontRight_Motor);
  pinMode(ci_FrontLeft_Motor, OUTPUT);
  servo_FrontLeftMotor.attach(ci_FrontLeft_Motor);
  pinMode(ci_BackRight_Motor, OUTPUT);
  servo_BackRightMotor.attach(ci_BackRight_Motor);
  pinMode(ci_BackRight_Motor, OUTPUT);
  servo_BackLeftMotor.attach(ci_BackLeft_Motor);


  ///////////////////////////////////////////////////////
  // setting up grip servos
  left_grip_servo.attach(4); //the pins here are arbitrary and can be changed
  right_grip_servo.attach(5);

  ///////////////////////////////////////////////////////  ///////////////////////////////////////////////////////  ///////////////////////////////////////////////////////
  //NOTE FOR WHOEEVR SETS UP TEH ARM SERVOS:
  //THE PINS NEED TO BE SET FOR EACH SERVO LINK

  // set up arm motors
  // pinMode(ci_Base_Arm_Motor, OUTPUT);
  // servo_ArmMotor.attach(ci_Base_Arm_Motor);
  //pinMode(ci_Top_Arm_Motor, OUTPUT);
  //servo_ArmMotor.attach(ci_Top_Arm_Motor);

  //pinMode(ci_Grip_Motor, OUTPUT);
  // servo_GripMotor.attach(ci_Grip_Motor);
  //servo_GripMotor.write(ci_Grip_Motor_Zero);

  // set up motor enable switch
  //pinMode(ci_Motor_Enable_Switch, INPUT);
  //digitalWrite(ci_Motor_Enable_Switch, HIGH); //pullup resistor

  // set up encoders. Must be initialized in order that they are chained together,
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_FrontLeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_FrontLeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_FrontRightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_FrontRightMotor.setReversed(true);  // adjust for positive count when moving forward
  encoder_BackLeftMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_BackLeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_BackRightMotor.init(1.0 / 3.0 * MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_BackRightMotor.setReversed(false);  // adjust for positive count when moving forward

  //encoder_GripMotor.init(MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);

  // read saved values from EEPROM
  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //EEPROM NEEDS TO BE SORTED OUT ASAP SO WE CAN CALIBRATE THE 2 IR SENSORS
  /*
    b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_L);
    <<<<<<< HEAD
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
    ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
    b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
    ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
  */
  /*
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Left_Line_Tracker_Light_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Dark_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Middle_Line_Tracker_Light_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Dark_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Right_Line_Tracker_Light_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Line_Tracker_Dark_Address_H);
    ui_Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Left_Motor_Offset_Address_L);
    b_HighByte = EEPROM.read(ci_Left_Motor_Offset_Address_H);
    ui_Left_Motor_Offset = word(b_HighByte, b_LowByte);
    b_LowByte = EEPROM.read(ci_Right_Motor_Offset_Address_L);
    b_HighByte = EEPROM.read(ci_Right_Motor_Offset_Address_H);
    ui_Right_Motor_Offset = word(b_HighByte, b_LowByte);
  */




  ///////////////////////////////////////////////
  //ADDING THIS TO SEE IF THE CODE IS BEING KEPT IN THE SETUP FUNCTION
  Serial.println("code reaching this point");

}

void loop()
{




  if ((millis() - ul_3_Second_timer) > 3000)
  {
    bt_3_S_Time_Up = true;
  }

  // button-based mode selection
  if (ci_Mode_Button) //chanmged from: CharliePlexM::ui_Btn

  {
    if (bt_Do_Once == false)
    {
      bt_Do_Once = true;
      ui_Robot_State_Index++;
      ui_Robot_State_Index = ui_Robot_State_Index & 7;
      ul_3_Second_timer = millis();
      bt_3_S_Time_Up = false;
      bt_Cal_Initialized = false;
    }
  }
  else
  {
    bt_Do_Once = LOW;
  }

  // check if drive motors should be powered
  //bt_Motors_Enabled = digitalRead(ci_Motor_Enable_Switch);

  // modes
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.  -
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.      - Won't need
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level. - Won't need
  // 4 = Press mode button four times to enter. Calibrate motor speeds to drive straight. - Might need
  switch (ui_Robot_State_Index)
  {
    case 0:    //Robot stopped
      {
        //readLineTrackers();
        //Ping();
        //servo_LeftMotor.writeMicroseconds(ci_Left_Motor_Stop);
        //servo_RightMotor.writeMicroseconds(ci_Right_Motor_Stop);
        //servo_ArmMotor.write(ci_Arm_Servo_Retracted);
        //servo_GripMotor.writeMicroseconds(ci_Grip_Motor_Stop);
        //encoder_LeftMotor.zero();
        //encoder_RightMotor.zero();
        //encoder_GripMotor.zero();
        ui_Mode_Indicator_Index = 0;
        Serial.print("light: ");
        Serial.println(analogRead(A3));
        break;
      }

    case 1:    //Robot Run after 3 seconds
      {
        if (bt_3_S_Time_Up)
        {
          //not declared in this scope
          //readLineTrackers();

#ifdef DEBUG_ENCODERS
          ul_Front_Left_Motor_Position = encoder_FrontLeftMotor.getPosition();
          ul_Front_Right_Motor_Position = encoder_FrontRightMotor.getPosition();
          ul_Back_Left_Motor_Position = encoder_BackLeftMotor.getPosition();
          ul_Back_Right_Motor_Position = encoder_BackRightMotor.getPosition();
          ul_Grip_Motor_Position = encoder_GripMotor.getPosition();

          Serial.print("Encoders FL: ");
          Serial.print(encoder_FrontLeftMotor.getPosition());
          Serial.print(", FR: ");
          Serial.print(encoder_FrontRightMotor.getPosition());
          Serial.print(", BR: ");
          Serial.print(encoder_BackRightMotor.getPosition());
          Serial.print(", BL: ");
          Serial.print(encoder_BackLeftMotor.getPosition());
          Serial.print(", G: ");
          Serial.println(ul_Grip_Motor_Position, DEC);
#endif

          // set motor speeds
          ui_Front_Left_Motor_Speed = constrain(ui_Motors_Speed - ui_Front_Left_Motor_Offset, 1600, 2100);
          ui_Front_Right_Motor_Speed = constrain(ui_Motors_Speed - ui_Front_Right_Motor_Offset, 1600, 2100);
          ui_Back_Left_Motor_Speed = constrain(ui_Motors_Speed - ui_Back_Left_Motor_Offset, 1600, 2100);
          ui_Back_Right_Motor_Speed = constrain(ui_Motors_Speed - ui_Back_Right_Motor_Offset, 1600, 2100);


          /***************************************************************************************
             THIS IS MODE 1 FOR MSE FINAL DESIGN ROBOT
            /*************************************************************************************/


           rotateClockwise(150,90); // assume started facing positive y direction
            while(cmFront > 99) // 99 will need to be changed 
            {
              forward(150); 
            }
            extend_arm();
            wait_for_cube();
            pick_up_cube();
            rotateClockwise(150, 90); // should be facing the platform now (negative y)
            go_to_platform();
            place_cube_to_platform();
            drive_to_starting_position();
            
            // after this it loops to the top of the code again




#ifdef DEBUG_MOTORS
          Serial.print("Motors: Default: ");
          Serial.print(ui_Motors_Speed);
          Serial.print(" , Front Left = ");
          Serial.print(ui_Front_Left_Motor_Speed);
          Serial.print(" . Front Right = ");
          Serial.println(ui_Front_Right_Motor_Speed);
          Serial.print(" . Back Right = ");
          Serial.println(ui_Back_Right_Motor_Speed);
          Serial.print(" . Back Left = ");
          Serial.println(ui_Back_Left_Motor_Speed);
#endif
          // ui_Mode_Indicator_Index = 1; // remember to chage this back to 1 for logiacal flow
          which_case = 0;
        }
        break;
      }



  }//end switch

  if ((millis() - ul_Display_Time) > ci_Display_Time)
  {
    ul_Display_Time = millis();

#ifdef DEBUG_MODE_DISPLAY
    Serial.print("Mode: ");
    Serial.println(ui_Robot_State_Index, DEC);
#endif
    bt_Heartbeat = !bt_Heartbeat;

    digitalWrite(13, bt_Heartbeat);
    Indicator();
  }
}

// set mode indicator LED state
void Indicator()
{
  //display routine, if true turn on led

  iArrayIndex++;
  iArrayIndex = iArrayIndex & 15;
}

#ifdef DEBUG_LINE_TRACKERS
Serial.print("Trackers: Left = ");
Serial.print(ui_Left_Line_Tracker_Data, DEC);
Serial.print(", Middle = ");
Serial.print(ui_Middle_Line_Tracker_Data, DEC);
Serial.print(", Right = ");
Serial.println(ui_Right_Line_Tracker_Data, DEC);
#endif



//*****Driving Functions*****///////////////////////////////////////////////////////////

void forward(int speed) {
  //zero encoders before we do anything
  encoder_FrontRightMotor.zero();
  encoder_FrontLeftMotor.zero();


  servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_FrontRightMotor.writeMicroseconds(1500 - speed); //forward
  servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackRightMotor.writeMicroseconds(1500 - speed); //forward


  //add negative casue its rotating backwards when going forward
  Input = (-encoder_FrontRightMotor.getRawPosition() - encoder_FrontLeftMotor.getRawPosition()); //input to PID ios dif between encoders

  //output is the return of the PID, it is a pwm signal
  //we can't write this directly to a wheel
  motorControl.Compute(); //calls a change or something to the PID

  //in this statement if right is more than left, ie: right turning more than left, ie need to correct right
  if (Output > 0)
  {
    //correcting to turn slightly right
    servo_FrontLeftMotor.writeMicroseconds(1500 + speed + 100); //forward
    servo_FrontRightMotor.writeMicroseconds(1500 - speed + 50); //forward
    servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
    servo_BackRightMotor.writeMicroseconds(1500 - speed); //forward
  }



//IF PID WORKS, WRITE THE REST OF THE PID CONTROL CODE HERE



  //////////////////////////////////
  //added from searchForCubes()
  //THIS CODE ALLOWS THE ROBOT TO TRAVEL IN A STRAIGHT LINE
  //////////////////////////////////////////////////////////////////////
  //NOTE: MIGHT HAVE TO USE PINGFORWARD HALFWAY THROUGH OUR INCREASE OF Y COORDINATE
  //////////////////////////////////////////////////////////////////////

  //if travelling in positive y-direction
  if (current_pos[2] == 0)
  {
    //pingBack();
    //pingLeft();
    getDistance(); //FUNCTION CALL TO READ SERAIL COMM PORTS-> NO IDEA HOW LONG THIS WILL ACTUALLY TAKE
    current_pos[1] = distance[1]; //update y coordinate (might not even need this)
    current_pos[0] = (distance[2] + distance[3]) / 2; //updates current x-coordinate (acverages the 2 left ultrasonics)

    //veerLeft() and veerRight() keep us driving relatively straight in y-direction
    //brings robot towards left wall if drifting right
    if ( current_pos[0] > ((10 * numberOfPasses) + 3))
    { //comparative value is standard robot width * number of passes
      getDistance;
      //pingLeft();
      veerLeft(150, (distance[2] + distance[3]) / 2); //new function to steer slightly to the left
      //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER LEFT FOR A LITTLE AMOUNT
      //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
      //TO THIS PART OF THE CODE

      servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
      servo_FrontRightMotor.writeMicroseconds(1500 - speed); //forward
      servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
      servo_BackRightMotor.writeMicroseconds(1500 - speed); //forward

    }//end if

    //brings robot away from left wall if drifting left
    if ( current_pos[0] < ((10 * numberOfPasses)) - 3)
    {
      getDistance;
      //pingLeft();
      veerRight(150, (distance[2] + distance[3]) / 2); //new function to steer slightly to the left
      //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER RIGHT FOR A LITTLE AMOUNT
      //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
      //TO THIS PART OF THE CODE

      servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
      servo_FrontRightMotor.writeMicroseconds(1500 - speed); //forward
      servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
      servo_BackRightMotor.writeMicroseconds(1500 - speed); //forward

    }//end if



    ///////////////////////////////////////////////////
    //adding theses two cases to keep the robot parallel with the edge of the course
    ///////////////////////////////////////////////////
    //if the front of the robot is facing away from the left wall,
    //we adjust by turning counter clockwise (ccw)
    if (distance[2] > distance[3])
    {
      while (distance[2] > distance[3] - 1) //-1 is for tolerance
      {
        rotateCounterClockwise(200, 1); //rotates the robot one degree at a time (rotate func uses encoders not ultras
      }//end while
    }//end if

    //if robot is turning towards left wall
    if (distance[2] < distance[3])
    {
      while (distance[2] < distance[3] + 1) //+1 is for tolerance
      {
        rotateCounterClockwise(200, 1); //rotates the robot one degree at a time (rotate func uses encoders not ultras
      }//end while
    }//end if



  }//end if(current_pos[2] == 0)

  ///////////////////////////////////////////////////////////////////
  //HAVE TO REWRITE ALL THE ABOVE CODE FOR WHEN WE TRAVEL IN NEGATIVE Y-DIRECTION
  ///////////////////////////////////////////////////////////////////





  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///CHECK SERIAL COMMUNICATION ON POSITIVE Y DIRECTION BEFORE i FINISH WRITING ALL THE SAME CODE BUT FOR THE NEGATIOVE Y DIRECTION
  //USING THE RIGHT ULTRASONIC SENSORS











  //if travelling in negative y-direction
  if (current_pos[2] == 1)
  {
    pingBack();
    pingRight();
    current_pos[1] = cmBack; //update y coordinate (might not even need this)
    current_pos[0] = cmRight; //the added value accounts for width of robot MIGHT NOT NEED TO ACCOUTN FOR ROBOT WIDTH

    //veerLeft() and veerRight() keep us driving relatively straight in y-direction
    //robot drifting left away from wall
    if ( current_pos[0] > (10 * numberOfPasses))
    {
      pingRight();
      //FUNCTION NOT WRITTEN YET, AS OF MARCH 27, 2016
      veerRight(150, cmRight); //new function to steer slightly to the left
      //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER LEFT FOR A LITTLE AMOUNT
      //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
      //TO THIS PART OF THE CODE
      servo_FrontLeftMotor.writeMicroseconds(1500 + speed - 10); //forward
      servo_FrontRightMotor.writeMicroseconds(1500 - speed); //forward
      servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
      servo_BackRightMotor.writeMicroseconds(1500 - speed); //forward




    }

    //brings drifting towards wall
    if ( current_pos[0] < (10 * numberOfPasses))
    {
      pingLeft();
      veerLeft(150, cmLeft); //new function to steer slightly to the left
      //THIS NEW FUNCTION SHOULD USE ENCODER POSITIONS TO ONLY VEER RIGHT FOR A LITTLE AMOUNT
      //THEN GO ABCK TO DRIVING FORWARD BEFORE EXITING AND PASSING CONTROL BACK
      //TO THIS PART OF THE CODE
      servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
      servo_FrontRightMotor.writeMicroseconds(1500 - speed); //forward
      servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
      servo_BackRightMotor.writeMicroseconds(1500 - speed); //forward


    }
  }//end if(current_pos[2] == 1)


}//END FORWARD()

void reverse(int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_FrontRightMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackRightMotor.writeMicroseconds(1500 - speed); //reverse
}

void moveLeft(int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_FrontRightMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackRightMotor.writeMicroseconds(1500 - speed); //reverse
}

void moveRight(int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_FrontRightMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackRightMotor.writeMicroseconds(1500 + speed); //forward
}

void rotateClockwise(int speed, int angle) {
  //change the numbers accordingly
  encoder_FrontRightMotor.zero();
  //MIGHT HAVE TO AVERAGE THE 4 ENCODER VALUES AND COMPARE THEM TO ANGLE PASSED
  while (encoder_FrontRightMotor.getRawPosition() > -(9 * angle)) //9 encoder ticks per degree
  {
    Serial.print("right encoder:");
    Serial.println(encoder_FrontRightMotor.getRawPosition());
    servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
    servo_FrontRightMotor.writeMicroseconds(1500 + speed); //forward
    servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
    servo_BackRightMotor.writeMicroseconds(1500 + speed); //forward
  }



}

void forwardLeftDiagonal(int speed) {
  servo_FrontRightMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackLeftMotor.writeMicroseconds(1500 + speed); //forward
}

void forwardRightDiagonal (int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 + speed); //forward
  servo_BackRightMotor.writeMicroseconds(1500 + speed); //forward
}

void reverseRightDiagonal(int speed) {
  servo_FrontRightMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackLeftMotor.writeMicroseconds(1500 - speed); //reverse
}

void reverseLeftDiagonal(int speed) {
  servo_FrontLeftMotor.writeMicroseconds(1500 - speed); //reverse
  servo_BackRightMotor.writeMicroseconds(1500 - speed); //reverse
}

void stop_motors() {
  servo_FrontLeftMotor.writeMicroseconds(1500);
  servo_FrontRightMotor.writeMicroseconds(1500);
  servo_BackLeftMotor.writeMicroseconds(1500);
  servo_BackRightMotor.writeMicroseconds(1500);
}

//*****Pinging Functions*****

//allow the user to ping the front ultrasonic sensor
void pingFront() { //took "int delayTime" out of argument list
  digitalWrite(ci_Front_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_Front_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people

  frontDuration = pulseIn(ci_Front_Ultrasonic_Data, HIGH, 10000);

  cmFront = microsecondsToCentimeters(frontDuration);
  Serial.print("Front distance = ");
  Serial.print(cmFront);
  Serial.print("cm  ");
  //Serial.println();
}


void pingBack() {

  digitalWrite(ci_Back_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_Back_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people
  backDuration = pulseIn(ci_Back_Ultrasonic_Data, HIGH, 10000);

  cmBack = microsecondsToCentimeters(backDuration);
  Serial.print("Back distance = ");
  Serial.print(cmBack);
  Serial.print("cm  ");
  //Serial.println();

}

void pingLeft() {

  digitalWrite(ci_Left_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_Left_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people

  leftDuration = pulseIn(ci_Left_Ultrasonic_Data, HIGH, 10000);

  cmLeft = microsecondsToCentimeters(leftDuration);

  Serial.print("Left distance = ");
  Serial.print(cmLeft);
  Serial.print("cm  ");
  //Serial.println();

}

void pingRight() {
  digitalWrite(ci_Right_Ultrasonic_Ping, HIGH); //giving a short pulse before hand to ensure a clean high pulse
  delayMicroseconds(10);
  digitalWrite(ci_Right_Ultrasonic_Ping, LOW); //keep in mind name for ultrasonic sensor might be different for other people

  rightDuration = pulseIn(ci_Right_Ultrasonic_Data, HIGH, 10000);

  cmRight = microsecondsToCentimeters(rightDuration);
  Serial.print("Right distance = ");
  Serial.print(cmRight);
  Serial.print("cm  ");
  Serial.println();
}


long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

//used to convert microseconds (the data inputted) into centimeters so distances can be better gauged when pinging



//////////////////////////////////////
//MARCH 20, 2016
//JULIAN ZANE
//function takes open and closed positions of grip
//closes the grip
//////////////////////////////////////
void grip_close(int open_pos, int closed_pos)
{
  for ( int n = open_pos; n < closed_pos; n++)
  {
    right_grip_servo.write(n);
    left_grip_servo.write(180 - n); //180-n makes this one count move backwards
    delay(15);
    Serial.println(right_grip_servo.read());
  }
}




//////////////////////////////////////
//MARCH 20, 2016
//JULIAN ZANE
//function takes open and closed positions of grip
//opens the grip
//////////////////////////////////////
void grip_open(int open_pos, int closed_pos)
{
  for ( int n = closed_pos; n > open_pos; n--)
  {
    right_grip_servo.write(n);
    left_grip_servo.write(180 - n); //180-n makes this one count move backwards
    delay(15);
    //Serial.println(n);
  }
}







//////////////////////////////////*
//JULIAN ZANE
//MARCH 19, 2016

//function initialzes the home position
//of the robot for future navigational use

//*/////////////////////////////////

void initPos()
{
  //ping left and read 10 times to let values stabilize
  for (int i = 0; i < 10; i++)
  {
    pingLeft();
    pingBack();
    home_pos[0] = cmLeft; //sets x coordinate
    home_pos[1] = cmBack; //sets y coordinate
  }
} //end function




////////////////////////////////////*
//JULIAN ZANE
//MARCH 19, 2016

//function takes care of driving around
//and trying to find a cube
//DOES NOT CONCERN ITSELF IF THE CUBE
//IS REAL OR NOT JSUT YET (SEPERATE FUNCTION)
//*////////////////////////////////////


//rotate counterclock wise with the speed and angle
void rotateCounterClockwise(int speed, int angle)
{
  //change and test numbers accordingly

  encoder_FrontRightMotor.zero();
  //MIGHT HAVE TO AVERAGE THE 4 ENCODER VALUES AND COMPARE THEM TO ANGLE PASSED
  while (encoder_FrontRightMotor.getRawPosition() > -(9 * angle)) //9 encoder ticks per degree
  {
    Serial.print("right encoder:");
    Serial.println(encoder_FrontRightMotor.getRawPosition());
    servo_FrontLeftMotor.writeMicroseconds(1500 - speed); //forward
    servo_FrontRightMotor.writeMicroseconds(1500 - speed); //forward
    servo_BackLeftMotor.writeMicroseconds(1500 - speed); //forward
    servo_BackRightMotor.writeMicroseconds(1500 - speed); //forward
  }

  // if the robot rotates 180 degrees, change the directionality register to
  // the opposite of whatever it currently is
  if (angle == 180) {
    if (current_pos[2] == 1) {
      current_pos[2] = 0;
    }
    else if (current_pos[2] == 0) {
      current_pos[2] = 1;
    }
  }
}









////////////////////////////////////////////////*
//JULIAN ZANE
//MARCH 25, 2016

//FUNCTION STARTS WHEN BOT IS FACING THE SIDE WALL AND STOPPED
//RETURNS NOTHING
//TAKES NO ARGUMENTS
//EXITS INTO INDEXING FUNCTION WHEN LINE TRACKER READS THE FIRST DARK LINE

//*////////////////////////////////////////////////





////////////////////////////////////*
//JULIAN ZANE
//INITAL CREATION: MARCH 25, 2016

//ROBOT ENTERS FUNCTION WHEN IT DROPS OFF THE CUBE AT HOME AND RETURNS ARM TO "DRIVING POSITION"
//RETURNS NOTHING
//TAKES NO ARGUMENTS
//GENERAL PLAN:
//-->ROTATE TO MATCH DIRECTION OF DIRECTIONALITY REGISTER
//-->DRIVE IN X AND Y DIRECTIONS UNTIL CURRENT UPDATES MATCH LAST KNOWN POSITION

//*////////////////////////////////////



///////////////////////////////////////////////////**
//JULIAN ZANE
//INITIAL CREATION: MARCH 25, 2016

//FUNCTION RETURNS NOTHING
//FUNCTION TAKES NO ARGUMENT
//DRIVES BACKWARDS TO THE EDGE OF THE TRACK AND TURNS AROUND, DUMPING THE CUBE OUTSIDE OF THE TRACK
//THEN RETURNING TO IT'S LAST KNOWN POSITION
//--> RESUSE CODE FOR THIS PART
//ROBOT ENTERS THIS FUNCTION STOPPED AND WITH A CUBE READY TO BE PICKED UP

//**////////////////////////////////////////////////////






//ADD SIMILAR CODE AS IN FOWARD() TO REVERSE() TO ALLOW THE ROBOT TO MOVE BACKWARD IN A "STRAIGHT" LINE






void veerRight(int speedy, int xDistance) {
  int slower = speedy - 50;
  int leftDistance = xDistance + 1; // may need to change the +3
  // for veerLeft, the right side motors are operating faster than the left side
  // this is done until we get to our set x distance away + a small value (2-3cm)
  // this is used to drive straight and correct the drift for the omniwheels
  getDistance();

  //pingLeft();
  while ((distance[2] + distance[3]) / 2 < leftDistance) { //average the two left readings
    servo_FrontLeftMotor.writeMicroseconds(1500 + speedy); // the difference in the veerRight and left
    servo_FrontRightMotor.writeMicroseconds(1500); // is the speed of the wheels and the
    servo_BackLeftMotor.writeMicroseconds(1500); // distance of the left wall to the ultrasonic
    servo_BackRightMotor.writeMicroseconds(1500 - slower);
    //pingLeft();
    getDistance();
  }
}


void veerLeft(int speedy, int xDistance) {
  int slower = speedy - 50;
  int leftDistance = xDistance - 1; // may need to change the +3
  // for veerLeft, the right side motors are operating faster than the left side
  // this is done until we get to our set x distance away + a small value (2-3cm)
  // this is used to drive straight and correct the drift for the omniwheels
  getDistance();
  //pingLeft();
  while ( (distance[2] + distance[3]) / 2 > leftDistance) {
    servo_FrontLeftMotor.writeMicroseconds(1500);
    servo_FrontRightMotor.writeMicroseconds(1500 - speedy);
    servo_BackLeftMotor.writeMicroseconds(1500 + speedy);
    servo_BackRightMotor.writeMicroseconds(1500);
    getDistance();
    //pingLeft();
  }
}


/////////////////////////////////////////////////////////////////
//JULIAN ZANE
//INITIALLY CREATED: MARCH 31, 2016

//FUNCTION TAKES NO ARGUMENTSRETURNS NOTHING,
//SERIAL COMM WITH BOARD 2, TO UPDATE distance[] AND GET
//"CURRENT" DISTANCE VALUES OF ALL UILTRASONIC SENSORS
/////////////////////////////////////////////////////////////////
void getDistance()
{

  ///NOTE: FOR THIS METHOD, WE MAY RUN INTO ERRORS WHERE THJE SERIAL BUFFER IS NOT BEING
  //CLEARED.  AS SUCH, WE MAY BE READING DISTANCE VALUES FROM FAR BACK IN THE FUTURE, OR THE
  //BUFFER MAY JUST FILL UP ENTIRELY
  //availabl() returns how many bits are ready to be read from the serial buffer
  //I assume the way the buffer works is that the bits stay in the buffer
  //until they are read() gfrom it, then they are cleared, making room for other bits
  if (Serial.available() > 5) //if there is at least 6 bits in the buffer ready to be read
  {
    for (int n = 0; n < 7; n++)
    {
      //read the values of each array from the buffer and hope that the two arrays sync up with one another
      distance[n] = Serial.read();
    }

    //this tries to give warning if the arrays are not synched up, if that is the case, the boards need to be reset
    //as far as I can tell (march 31, 2016), there is no other way of resynching the serial comm's
    if (distance[6] != -1)
    {
      Serial.println("ERROR!  COMMUNICATION ARRAYS NOT SYNCED. PLEASE RESET BOARDS AND TRY AGAIN!");
    }
  }
}

void extend_arm()
{
  // extend arm so ultrasonic is pointed at first holding spot
}

void wait_for_cube() // reads to grip ultrasonic to see if robot has come to drop off cube yet
{ 
  while(1)
  {
    if(gripCM < 20)   //this means the other robot has come to the loading area
      break;
  }
}

void pick_up_cube()
{
  //extend arm down and grip cube
}

void go_to_platform()
{
  while(!(whiskers)) // assume whiskers return true when triggered
  {
    forward(150);
  }
  // may have to reverse a bit more after whiskers are triggered
  // ^ will be tested experimentally
}

void place_cube_to_platform()
{
  // extend arm fully upward and release grip
}

void drive_to_starting_position()
{
  rotateClockwise(150, 90); // should be facing positive y now
  while(!(whiskers))
  {
    forward(150);
  }
  // may have to drive forward more a set distance after this 
}



