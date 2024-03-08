

#include <ros.h>
//include our custom msg folder
#include <forklift_simulation/ard_odom.h>

//For control message 
#include <geometry_msgs/Twist.h>


//controlpins 

int enable = 7;
int direction_enable_left = 8;
int direction_enable_right = 9;
int drive_motor_pwm = 10;

//steering pins
int en1 = 4;
int en2 = 5;
float steering_pwm = 6;

float vel = 0;
float pulses = 1000.0;
volatile long x , t_interval, timer = 0 , counter = 0;
volatile float Rad_per_pulse = (2 * 3.14) / pulses;
float counter_100 = 0;
float counter_10 = 0;
double deg = 0;

//control_variables
float top_speed = 1;

//controller callback variables
float speed_data = 0;
float speed_action = 0;
float steering_action = 0;
float steering_data =0;



//Create Ros Node Object
ros::NodeHandle nh;
forklift_simulation::ard_odom odometry;
ros::Publisher p("ard_odom_topic", &odometry);

// Callback function to handle incoming Twist messages
void twistCallback(const geometry_msgs::Twist& msg);

// ROS Subscriber for Twist messages
ros::Subscriber<geometry_msgs::Twist> controller_sub("controller_live", &twistCallback );



void setup()
{
  //Same as Rosserial Baud Rate
  Serial.begin(57600);

  //Two Channels of Encoders
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  //Potentiometer Pin
  pinMode(A0, INPUT);

  //Motor driver pins
  pinMode(enable, OUTPUT);
  pinMode(direction_enable_left, OUTPUT);
  pinMode(direction_enable_right, OUTPUT);
  pinMode(drive_motor_pwm, OUTPUT);

  //Steering driver pins
  pinMode(steering_pwm, OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  
  digitalWrite(enable, HIGH);
  digitalWrite(direction_enable_left,HIGH);
  digitalWrite(direction_enable_right,LOW);



  //Attach Interrupt for Encoder Pins
  attachInterrupt(0, A, RISING);
  attachInterrupt(1, B, RISING);

  //Ros Node Initialization
  nh.initNode();
  nh.advertise(p);

  nh.subscribe(controller_sub);

}

void loop() {

  //Angle Reading From Potentiometer
  deg = 1.5 * 180 * (analogRead(A0) - 512) / 1024.0;
  odometry.angle = deg;

  if ( counter != x )
  {
    odometry.distance = 0.1 * counter * Rad_per_pulse;

    if (counter < x)
    {
      odometry.velocity = -vel;
    }
    else
    {
      odometry.velocity = vel;
    }
    x = counter;
  }
  if (abs(counter_100 - counter) > 100)
  {
    t_interval = micros() - timer;
    vel = 10 * pow(10.0, 6) * (Rad_per_pulse) / t_interval;
    timer = micros();
    counter_100 = counter;
  }
  if (micros() - timer > 1000000)
  {
    vel = 0;
    odometry.velocity = vel;
  }

//  if (abs(counter_10-counter)<10)
//  {
//    vel = 0;
//    odometry.velocity = vel;
//    counter_10=counter;
//  }
  
  p.publish(&odometry);
  nh.spinOnce();
 // delay(1);
  
//  Serial.print("Angle: ");
//  Serial.print(odometry.angle);
//  Serial.print(" | Distance: ");
//  Serial.print(odometry.distance);
//  Serial.print(" | Velocity: ");
//  Serial.println(odometry.velocity);
}

void A()
{
  if (digitalRead(3) == LOW)
  {
    counter++;
  }
  else
  {
    counter--;
  }
}

void B()
{
  if (digitalRead(2) == LOW)
  {
    counter--;
  }
  else
  {
    counter++;
  }
}

void twistCallback(const geometry_msgs::Twist& msg)
{
  speed_data = msg.linear.x;
  speed_action = (speed_data/top_speed)*255;
  steering_data = msg.angular.z;
  steering_action = (steering_data/((22/7.0)/2))*200;
  
  analogWrite(drive_motor_pwm, speed_action);


  
  if(steering_action<0){
    digitalWrite(en1, 0);
    digitalWrite(en2, 1);
    }
  else
  {
    digitalWrite(en2, 0);
    digitalWrite(en1, 1); 
  }
  analogWrite(steering_pwm, abs(steering_action));
  
  if (abs(deg)>90)
  {
     analogWrite(steering_pwm, 0);
  }

}
