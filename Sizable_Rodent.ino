//define hbridge pins
#define EN1  12
#define EN2  11
#define IN1  5
#define IN2  6
#define IN3  9
#define IN4  10

//define indicator LED
#define LED  13

//define sensor pins
#define left  A0
#define right  A1

// Declare parameters of the robot to measure and control
int leftSpeed;
int rightSpeed;

int rightDistance;
int leftDistance;

int rightBaseline;
int leftBaseline;

// Parameters for the controller 
int rightError; 
int leftError;   
int totalError; 
int tolerance; 
 
int time; // Time between control loop iterations 
int integral; 
double kp; // Proportional gain 
double ki; // Integral gain 

// Functions to move and decide how to move 
int acquireLeft();
int acquireRight();
void moveStraight(); 
void turnRight();
void turnLeft();
void stopMoving(); 

void setup()
{
  //Configure the pins
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  digitalWrite(EN1, HIGH);
  digitalWrite(EN2, HIGH);
  
  pinMode(LED, OUTPUT);
  
  pinMode(left, INPUT);
  pinMode(right, INPUT);
  
  //set the starting parameters
  rightSpeed = 230;
  leftSpeed = 200;
  
  rightBaseline = acquireRight();
  leftBaseline = acquireLeft();
  
  //set the control parameters
  kp = 0.1;
  ki = 3;
  time = 10;
  totalError = 0;
  integral = 0;
  tolerance = 100;
  
  //begin serial for debugging
  Serial.begin(9600);
}

void loop()
{
  rightSpeed = 230;
  leftSpeed = 200;
  //Serial.println(analogRead(left));
  int currentRight = acquireRight();
  int currentLeft = acquireLeft();
  
  while (currentRight > 220 && currentLeft > 400)
  {
    moveStraight();
    currentRight = acquireRight();
    currentLeft = acquireLeft();
  }
    
  stopMoving();
  currentRight = acquireRight();
  currentLeft = acquireLeft();
  
  if (currentRight < 220)
  {
    for (int i = 0; i < 5000; i++)
    {
      analogWrite(IN1, 0);
      analogWrite(IN2, 230);
      analogWrite(IN3, 200);
      analogWrite(IN4, 0);
    }
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    turnRight();
  }
  else
  {
    for (int i = 0; i < 5000; i++)
    {
      analogWrite(IN1, 0);
      analogWrite(IN2, 230);
      analogWrite(IN3, 200);
      analogWrite(IN4, 0);
    }
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    turnLeft();
  }
}

int acquireLeft()
{
  unsigned int sum = 0;
  for (int i = 0; i < 10; i++)
    sum += analogRead(left);
  return sum/10;
}

int acquireRight()
{
  unsigned int sum = 0;
  for (int i = 0; i < 10; i++)
    sum += analogRead(right);
  return sum/10;
}

void moveStraight()
{
  //This is where our controller will be implemented since we only need it when going forward.
  
  //Find your position relative to the left and right walls
  rightDistance = acquireRight();
  leftDistance = acquireLeft();
  
  //Compute the error in these measurements
  rightError = rightBaseline - rightDistance;
  leftError = leftBaseline - leftDistance;
  
  //Compute the total error and integral error
  totalError = rightError - leftError;
  integral += totalError*time;
  
  //Decide how to correct based on the error
  if (totalError < -tolerance)
  {
    rightSpeed += abs(kp*totalError + ki*integral);  //shift right
    leftSpeed -= abs(kp*totalError + ki*integral);
  }
  else if (totalError > tolerance)
  {
    rightSpeed -= abs(kp*totalError + ki*integral);  //shift left
    leftSpeed += abs(kp*totalError + ki*integral);
  }
  else
  {
    rightSpeed = 230;
    leftSpeed = 200;
  }
  
  //Normalize speeds if they exceed the system limit
  if (rightSpeed < 0)
    rightSpeed = 0;
  if (rightSpeed > 255)
    rightSpeed = 255;
  if (leftSpeed < 0)
    leftSpeed = 0;
  if (leftSpeed > 255)
    leftSpeed = 255;
    
  //Set motor control signals
  analogWrite(IN1, 0);
  analogWrite(IN2, rightSpeed);
  analogWrite(IN3, leftSpeed);
  analogWrite(IN4, 0);
}

void turnRight()
{
  //right motor backward
  analogWrite(IN1, 255);
  analogWrite(IN2, 0);
  //left motor forward
  analogWrite(IN3, 255);
  analogWrite(IN4, 0);
  //wait some time
  delay(550);
  //go back to straight
  for (int i = 0; i < 10000; i++)
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, 230);
    analogWrite(IN3, 200);
    analogWrite(IN4, 0);
  }
}

void turnLeft()
{
  //right motor forward
  analogWrite(IN1, 0);
  analogWrite(IN2, 255);
  //left motor backward
  analogWrite(IN3, 0);
  analogWrite(IN4, 255);
  //wait some time
  delay(700);
  //go back to straight
  for (int i = 0; i < 10000; i++)
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, 230);
    analogWrite(IN3, 200);
    analogWrite(IN4, 0);
  }
}

void stopMoving()
{
  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);
}
