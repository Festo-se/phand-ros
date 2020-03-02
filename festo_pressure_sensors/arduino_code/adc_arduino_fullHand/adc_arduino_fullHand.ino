/* FSR multimatrix readout code. 
Chloe Dickson, Festo Bionics - Oct. 2019 
           
ROWS: wired and programmed as powered lines
COLUMNS: wired and programmed as read-out lines
          
Hardware setup: 
Rows are connected to Arduino Digital outputs [2, 13].
Columns are connected to Analog Inputs [0, 5]. 
All "aligned" rows are connected together on the breadboard, so that rows 
(A, B, C) on every individual matrix are seen as one single row and only 
need for example 3 outputs for three 3x3 matrices. Rows are connected to the 
ground through 2.2kOhm resistors (voltage divider).
A LED is connected to Digital Output pin 13 (optional).

Message modified with extra matrix_count parameter (number of fingers)
*/


#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
//#define USE_USBCON
#include <ros.h>
#include <festo_pressure_sensors/SensorValuesArduino.h>
//#include <ros_lib/festo_pressure_sensors/SensorValuesArduino.h>
#include <std_msgs/UInt16MultiArray.h>

static const uint8_t X_DIM = 3; // number of columns per matrix
static const uint8_t Y_DIM = 3; // total number of rows
static const uint8_t MATRIX_COUNT = 3; // number of matrices
static const uint8_t DATA_LENGTH = X_DIM*Y_DIM*MATRIX_COUNT ; // full matrix size

ros::NodeHandle nh;
festo_pressure_sensors::SensorValuesArduino adc_msg;
ros::Publisher p("adc", &adc_msg);
void comm_callback(const std_msgs::UInt16MultiArray& msg);
ros::Subscriber<std_msgs::UInt16MultiArray> sub("/settings" , comm_callback);

uint16_t adc_msg_data[DATA_LENGTH] = {0} ;
uint16_t  adc_msg_ids[DATA_LENGTH] = {0} ;
uint16_t setting_msg_data[3] = {0} ;

// the FSRs and resistors are connected to a0-5, powered lines to d2-13
uint8_t powered_pins[X_DIM*MATRIX_COUNT] = {2,3,4,5,6,7}; // columns 
//uint8_t powered_pins[X_DIM*MATRIX_COUNT] = {2,3,4,5,6,7,8,9,10}; // columns 
uint8_t readout_pins[Y_DIM] = {0,1,2}; // rows

// setting_msg_data[0-2]
uint16_t delay1 = 1;
uint16_t delay2 = 1; 

uint8_t row = 0, col = 0, m = 0;

//Adjust sensing parameters with values entered in GUI
void comm_callback(const std_msgs::UInt16MultiArray& msg){
   delay1 = msg.data[0] ;
   delay2 = msg.data[1] ; 
}

void setup()
{   
  adc_msg.x_dim = X_DIM ;
  adc_msg.y_dim = Y_DIM ;
  
  adc_msg.data = adc_msg_data;
  adc_msg.data_length = DATA_LENGTH;
  adc_msg.matrix_count = MATRIX_COUNT;
  adc_msg.sens_id = adc_msg_ids;

  for (uint8_t i=0; i<DATA_LENGTH;i++){
    adc_msg.sens_id[i]=i;
  }

  
  //Set-up digital outputs for matrix serial reading
  for (uint8_t i=0; i<X_DIM; i++){
    pinMode(powered_pins[i], OUTPUT);
  }
  pinMode(13, OUTPUT);
  
  nh.initNode();
  nh.advertise(p);
  nh.subscribe(sub);

  for (uint8_t i = 0; i<3; i++){
    digitalWrite(13, LOW);
    delay(200);
    digitalWrite(13, HIGH);
    delay(200);
  } 
  digitalWrite(13, HIGH);
}


void loop()
{
  for (uint8_t i=0; i<X_DIM*MATRIX_COUNT; i++){ 
    pinMode(powered_pins[i], OUTPUT); 
    digitalWrite(powered_pins[i], HIGH); 
    delay(delay1);

    for (uint8_t j=0; j<Y_DIM; j++){
      uint8_t idx = j*X_DIM*MATRIX_COUNT + i;
      adc_msg.sens_id[idx] = idx;
      adc_msg.data[idx] = analogRead(readout_pins[j]);
      delay(delay2);
    } 

    digitalWrite(powered_pins[i], LOW); 
    pinMode(powered_pins[i], INPUT);
  }
      
  p.publish(&adc_msg);

  nh.spinOnce();
}
