

#include <ros.h>
//include our custom msg folder
#include <forklift_simulation/ard_odom.h>

float vel = 0;
float pulses = 1000.0;
volatile long x , t_interval, timer = 0 , counter = 0;
volatile float Rad_per_pulse = (2 * 3.14) / pulses;
float counter_100 = 0;
float counter_10 = 0;
double deg = 0;




//Create Ros Node Object
ros::NodeHandle nh;
forklift_simulation::ard_odom odometry;
ros::Publisher p("ard_odom_topic", &odometry);



void setup()
{
  //Same as Rosserial Baud Rate
  Serial.begin(57600);

  //Two Channels of Encoders
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);

  //Potentiometer Pin
  pinMode(A3, INPUT);

  //Attach Interrupt for Encoder Pins
  attachInterrupt(0, A, RISING);
  attachInterrupt(1, B, RISING);

  //Ros Node Initialization
  nh.initNode();
  nh.advertise(p);


}

void loop() {

  //Angle Reading From Potentiometer
  deg = 1.5 * 180 * (analogRead(A3) - 512) / 1024.0;
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
  delay(1);
  
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
