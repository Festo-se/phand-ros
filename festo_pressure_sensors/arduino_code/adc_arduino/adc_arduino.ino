/* FSR simple matrix testing sketch. 
Chloe Dickson, Festo Bionics - Oct. 2019 

Based on:  LadyAda simple FSR testing sketch
           ROS ADC example
           ROS Uint16Arraz message
           
Breadboard setup: Voltage divider with 10k resistor and parallel 4.7uF capa
Horizontal lines connected to A0-2, vertical lines to digital outputs 2-5
*/

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <festo_pressure_sensors/SensorValuesArduino.h>
#include <std_msgs/UInt16MultiArray.h>

static const int DATA_LENGTH = 9 ;

ros::NodeHandle nh;
festo_pressure_sensors::SensorValuesArduino adc_msg;
ros::Publisher p("adc", &adc_msg);
void comm_callback(const std_msgs::UInt16MultiArray& msg);
ros::Subscriber<std_msgs::UInt16MultiArray> sub("/settings" , comm_callback);

uint16_t adc_msg_data[DATA_LENGTH] = {0} ;
uint16_t setting_msg_data[3] = {0} ;
// the FSRs and 10K pulldowns are connected to a0-2
int fsrPin_0 = 0;     
int fsrPin_1 = 1;
int fsrPin_2 = 2;

int8_t col_pins[3] = { 3, 5 , 6} ; 

int delay1 = 5 ;
int delay2 = 5 ; 
int pwmValue = 155 ;

void comm_callback(const std_msgs::UInt16MultiArray& msg){
  
   delay1 = msg.data[0] ;
   delay2 = msg.data[1] ; 
   pwmValue = msg.data[2] ;
   pinMode(13, OUTPUT);
   digitalWrite(13, LOW);

}

void setup()
{ 
    pinMode(13, OUTPUT);
   digitalWrite(13, HIGH);
  adc_msg.x_dim = 3 ;
  adc_msg.y_dim = 3 ;
  
  adc_msg.data = adc_msg_data ;
  adc_msg.data_length = DATA_LENGTH;
  
  

 
  
  //Set-up digital outputs for matrix serial reading
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(sub);
}

//We average the analog reading to elminate some of the noise
int averageAnalog(int pin){
  int v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

long adc_timer;

void loop()
{
  
  for (int col=0; col<3; col++){
    pinMode(col, OUTPUT);    
    analogWrite(col_pins[col], pwmValue);
    delay(delay1);

    adc_msg.data[col]   = analogRead(fsrPin_0); 
    adc_msg.data[col+3] = analogRead(fsrPin_1); 
    adc_msg.data[col+6] = analogRead(fsrPin_2);
    //adc_msg.data[0] = averageAnalog(0);
    analogWrite(col_pins[col], 0);
    pinMode(2, INPUT);
    pinMode(3, INPUT);
    pinMode(4, INPUT);
    
    delay(delay2);
  }
  adc_msg.data[0] = delay1 ;
  adc_msg.data[1] = delay2 ;
  adc_msg.data[2] = pwmValue ;
      
  p.publish(&adc_msg);

  nh.spinOnce();
}

