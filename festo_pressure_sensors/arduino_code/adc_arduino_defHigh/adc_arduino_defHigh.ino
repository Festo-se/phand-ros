/* FSR simple matrix testing sketch. 
Chloe Dickson, Festo Bionics - Nov. 2019 

Based on:  LadyAda simple FSR testing sketch
           ROS ADC example
           ROS Uint16Arraz message
           
Sensors initial range: 10kOhm-100Ohm (when pressed)
Textile matrix setup: rows/columns of conductive textile tape (conductive 
side against velostat), in matrix sandwich setup with velostat in the middle.
          
Breadboard setup: see default High setup, 2.2kOhm resistors. Columns 
pulled high through resistors, rows high (to Vcc), one row pulled low 
at a time for readout. Sampling on columns.
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
uint16_t setting_msg_data[4] = {0} ;

// the FSRs and 2.2K pulldowns are connected to a0-2
uint8_t col_pins[X_DIM] = {0,1,2};
uint8_t row_pins[Y_DIM] = {3,5,6}; 

uint16_t delay1 = 5;
uint16_t delay2 = 5; 
uint16_t pwmValue = 155;
static const uint8_t MODE = 1;

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

//  adc_msg.mode = MODE;
  
  //Set-up digital outputs for matrix serial reading
  for (uint8_t i=0; i<Y_DIM; i++){
    pinMode(row_pins[i], OUTPUT);  
    analogWrite(row_pins[i], pwmValue); //pull line high 
  }
  
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(sub);
}


void loop()
{
  
  for (uint8_t row=0; row<Y_DIM; row++){
    
    //Pull line low with delay  
    analogWrite(row_pins[row], 0); 
    delay(delay1);

    for (uint8_t col=0; col<X_DIM; col++){
      uint8_t i = row*X_DIM + col;
      adc_msg.data[i] = 1023 - analogRead(col_pins[col]);
      delay(delay2);
    }

    //pull line high again
    analogWrite(row_pins[row], pwmValue);

  }
      
  p.publish(&adc_msg);

  nh.spinOnce();
}
