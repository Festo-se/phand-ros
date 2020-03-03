/* FSR matrix testing sketch. 
Chloe Dickson, Festo Bionics - Nov. 2019 

Pulls up the columns successively (1, 2, 3) then reads the value on each 
row (A, B, C).

Settings: 
delay1: [0, 255] delay before measuring after pulling a line up (between rows)
delay2: [0, 255] delay after measuring (between each columns of the same row)
pwmValue: [0, 255] HIGH value for the read-out row. Maps to [0V, 5V] 
BACKLOG_SIZE: [1, 90] how many values are stored for a running average. 
  1 corresponds to real-time values, a bigger number smoothes readings
  but introduces delays
           
Sensors initial range: 10kOhm-100Ohm (when pressed)
Textile matrix setup: rows/columns of conductive textile tape (conductive 
side against velostat), in matrix sandwich setup with velostat in the middle.
          
Breadboard setup: Voltage divider with 5k resistor and parallel 4.7uF capa
Horizontal lines (A, B, C) connected to A0-2, vertical lines (1, 2, 3) to 
digital outputs 3,5,6. Tested on an Arduino Uno
*/

static const uint8_t X_DIM = 3;
static const uint8_t Y_DIM = 3;
static const uint8_t DATA_LENGTH = X_DIM*X_DIM;
static const uint8_t BASE_LENGTH = 90;


uint8_t col_pins[X_DIM] = {3,5,6}; // DOx Vertical lines (1, 2, 3)
uint8_t row_pins[Y_DIM] = {0,1,2}; // AIx Horizontal lines (A, B, C)

// Settings
uint8_t delay1 = 1;
uint8_t delay2 = 1; 
uint8_t delay3 = 1;
uint16_t pwmValue = 255; // Max value = 255

uint16_t meas[X_DIM*Y_DIM] = {0};

uint16_t count = 0;
uint8_t i = 0, j = 0;
uint8_t row = 0, col = 0;

int cycles = 0;

void setup()
{ 
  pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  
  digitalWrite(13, HIGH); // Turn LED on during setup
  
  //Set-up digital outputs for matrix serial reading
  for (col=0; col<Y_DIM; col++){
    pinMode(col_pins[col], OUTPUT);
  }
    
  Serial.begin(115200);
  delay(2000);

  // Print Serial Plotter labels for a 3x3 matrix
  Serial.println("A1,A2,A3,B1,B2,B3,C1,C2,C3,");
  
  digitalWrite(13, LOW);
  digitalWrite(9, HIGH);
}


void loop()
{
  while (count < 1500){
    for (col=0; col<Y_DIM; col++){
      
      //Pull line high with given analog value and delay
      analogWrite(col_pins[col], pwmValue);
      delay(delay1);
 
      for (row=0; row<X_DIM; row++){
        i = row*X_DIM+col;
        meas[i] = analogRead(row_pins[row]);
        Serial.print(meas[i]);
        Serial.print(",");
        delay(delay2);
      }
      
      analogWrite(col_pins[col], 0);
      delay(delay3);
    }
    
    Serial.println("");
//    delay(2);
    count++;
  }
  
  digitalWrite(9, LOW);
//  Serial.print("loop done: count = ");
//  Serial.println(count);
  while (1){
    delay(5000);
  }
  
}
