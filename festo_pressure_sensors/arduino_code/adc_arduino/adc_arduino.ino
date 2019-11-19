/* FSR simple matrix testing sketch. 
Chloe Dickson, Festo Bionics - Oct. 2019 

Based on:  LadyAda simple FSR testing sketch
           ROS ADC example
           ROS Uint16Arraz message
           
Sensors initial range: 10kOhm-100Ohm (when pressed)
Textile matrix setup: rows/columns of conductive textile tape (conductive 
side against velostat), in matrix sandwich setup with velostat in the middle.
          
Breadboard setup: Voltage divider with 10k resistor and parallel 4.7uF capa
Horizontal lines connected to A0-2, vertical lines to digital outputs 3,5,6
Tested on an Arduino Uno
*/

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <festo_pressure_sensors/SensorValuesArduino.h>
#include <std_msgs/UInt16MultiArray.h>

static const uint8_t X_DIM = 3;
static const uint8_t Y_DIM = 3;
static const uint8_t DATA_LENGTH = X_DIM*Y_DIM ;

ros::NodeHandle nh;
festo_pressure_sensors::SensorValuesArduino adc_msg;
ros::Publisher p("adc", &adc_msg);
void comm_callback(const std_msgs::UInt16MultiArray& msg);
ros::Subscriber<std_msgs::UInt16MultiArray> sub("/settings" , comm_callback);

uint16_t adc_msg_data[DATA_LENGTH] = {0} ;
uint16_t setting_msg_data[3] = {0} ;

// the FSRs and 10K pulldowns are connected to a0-2
uint8_t fsrPins[X_DIM] = {0,1,2};
uint8_t col_pins[Y_DIM] = {3,5,6}; 

uint16_t delay1 = 5;
uint16_t delay2 = 5; 
uint16_t pwmValue = 155;

//Adjust sensing parameters with values entered in GUI
void comm_callback(const std_msgs::UInt16MultiArray& msg){
   delay1 = msg.data[0] ;
   delay2 = msg.data[1] ; 
   pwmValue = msg.data[2];
   pinMode(13, OUTPUT);
   digitalWrite(13, LOW);
}

void setup()
{ 
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  adc_msg.x_dim = X_DIM ;
  adc_msg.y_dim = Y_DIM ;
  
  adc_msg.data = adc_msg_data ;
  adc_msg.data_length = DATA_LENGTH;
  
  //Set-up digital outputs for matrix serial reading
  for (uint8_t i=0; i<Y_DIM; i++){
    pinMode(col_pins[i], OUTPUT);
  }
  
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(sub);
}


void loop()
{
  
  for (uint8_t col=0; col<Y_DIM; col++){
    
    //Pull line high with given analog value and delay
    pinMode(col_pins[col], OUTPUT);    
    analogWrite(col_pins[col], pwmValue); 
    delay(delay1);

    for (uint8_t row=0; row<X_DIM; row++){
      uint8_t i = row*Y_DIM + col;
      adc_msg.data[i] = analogRead(fsrPins[row]);
      delay(delay2);
    }
    
    analogWrite(col_pins[col], 0);

    //delay(delay2);
  }
      
  p.publish(&adc_msg);

  nh.spinOnce();
}
