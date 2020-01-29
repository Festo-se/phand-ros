/* FSR simple matrix testing sketch. 
Chloe Dickson, Festo Bionics - Oct. 2019 

Based on:  LadyAda simple FSR testing sketch
           ROS ADC example
           ROS Uint16Array message
           
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

ros::NodeHandle nh;
festo_pressure_sensors::SensorValuesArduino adc_msg;
ros::Publisher p("adc", &adc_msg);
void comm_callback(const std_msgs::UInt16MultiArray& msg);
ros::Subscriber<std_msgs::UInt16MultiArray> sub("/settings" , comm_callback);

static const uint8_t X_DIM = 3; // number of columns per matrix
static const uint8_t Y_DIM = 3; // number of rows per matrix
static const uint8_t MATRIX_COUNT = 1; // number of matrices
static const uint8_t X_DIM_TOT = X_DIM*MATRIX_COUNT; // total number of columns
static const uint8_t DATA_LENGTH = X_DIM*Y_DIM; // data length of a single matrix

uint16_t adc_msg_data[DATA_LENGTH] = {0} ;
uint16_t setting_msg_data[3] = {0} ;

// the FSRs and resistors are connected to a0-2
uint8_t row_pins[Y_DIM] = {0,1,2}; // AIx Horizontal lines (A, B, C)
uint8_t col_pins[X_DIM_TOT] = {2,3,4}; // DOx Vertical lines (1, 2, 3)
//uint8_t col_pins[X_DIM_TOT] = {2,3,4,5,6,7}; // 2 matrices
//uint8_t col_pins[X_DIM_TOT] = {2,3,4,5,6,7,8,9,10}; // 3 matrices

// Settings
uint16_t delay1 = 5;
uint16_t delay2 = 5;
uint8_t row = 0, col = 0, m = 0, i=0; 

//Adjust sensing parameters with values entered in GUI
void comm_callback(const std_msgs::UInt16MultiArray& msg){
   delay1 = msg.data[0] ;
   delay2 = msg.data[1] ; 
//   pwmValue = msg.data[2];
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
  adc_msg.data_length = DATA_LENGTH*MATRIX_COUNT;
  
  //Set-up digital outputs for matrix serial reading
  for (col=0; col<X_DIM_TOT; col++){
    pinMode(col_pins[col], OUTPUT);
  }
  
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(sub);

  delay(2000);
  digitalWrite(13, LOW);
}


void loop()
{
  for (col=0; col<X_DIM_TOT; col++){

    m = uint8_t(col/X_DIM);
    
    //Pull line high with given analog value and delay  
    pinMode(col_pins[col], OUTPUT);
    digitalWrite(col_pins[col], HIGH);
    
    delay(delay1);

    // read all rows of activated column
    for (row=0; row<Y_DIM; row++){
      i = m*DATA_LENGTH + row*Y_DIM+(col%X_DIM);
      adc_msg.data[i] = analogRead(row_pins[row]);
//      meas[m][i] = analogRead(row_pins[row]);
      delay(delay2);
    }

//    for (uint8_t row=0; row<X_DIM; row++){
//      uint8_t i = row*Y_DIM + col;
//      adc_msg.data[i] = analogRead(row_pins[row]);
//      delay(delay2);
//    } 

    digitalWrite(col_pins[col], LOW);
    pinMode(col_pins[col], INPUT);
    
  }
      
  p.publish(&adc_msg);

  nh.spinOnce();
}
