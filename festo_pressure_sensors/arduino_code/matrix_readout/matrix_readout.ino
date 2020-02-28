/* FSR matrix testing sketch. 
Chloe Dickson, Festo Bionics - Jan. 2020 

Program to read out up to 4 3x3 matrices on an Arduino Uno. Can also be wired
into for different matrix shapes (up to one 6x12 matrix). In the standard setup,
all columns are independant and all rows are connected together in hardware.
Pulls up each columns successively M1-(1, 2, 3), M2-(1,2,3), etc. then reads 
the value on each row (A, B, C) for the activated column. Stores the values 
then prints them out in Serial and can be displayed in the Serial Plotter. 

Settings: 
delay1: [0, 255] delay before measuring after pulling a line up (between rows)
delay2: [0, 255] delay after measuring (between each columns of the same row)
delay3: [0, 255] delay after pulling column low (between rows)

Limited mode:
Reads only a finite number of sensor values then stops the program. Useful 
for exporting and analysing small data subsets. A LED indicates when measuring.
Copy-paste the serial output into a .csv (or .txt) file, save then press reset.
LTD_MODE = {0, 1} when 1, the program is in limited mode
MAX_READS = [1, 65536] how many values are read before the program stops

Hardware setup: 
Column lines are connected to Arduino Digital outputs [2, 13].
Rows are connected to Analog Inputs [0, 5]. 
All "aligned" rows are connected together on the breadboard, so that rows 
(A, B, C) on every individual matrix are seen as one single row and only 
need for example 3 inputs for three 3x3 matrices. Rows are connected to the 
ground through 2.2kOhm resistors (voltage divider).
A LED is connected to Digital Output pin 13 (optional).
*/

static const uint8_t X_DIM = 3; // number of columns per matrix
static const uint8_t Y_DIM = 3; // number of rows per matrix
static const uint8_t MATRIX_COUNT = 2; // number of matrices
static const uint8_t X_DIM_TOT = X_DIM*MATRIX_COUNT; // total number of columns
static const uint8_t DATA_LENGTH = X_DIM*Y_DIM; // data length of a single matrix

// un-/comment or modify column/row lines to reflect the hardware setup
//uint8_t col_pins[X_DIM_TOT] = {2,3,4}; // DOx Vertical lines (1, 2, 3)
uint8_t col_pins[X_DIM_TOT] = {2,3,4,5,6,7}; // 2 matrices
//uint8_t col_pins[X_DIM_TOT] = {2,3,4,5,6,7,8,9,10}; // 3 matrices
uint8_t row_pins[Y_DIM] = {0,1,2}; // AIx Horizontal lines (A, B, C)

// Settings
uint8_t delay1 = 1;
uint8_t delay2 = 1; 
uint8_t delay3 = 0;

// Loop settings
static const bool LTD_MODE = 1;
static const uint16_t MAX_READS = 1000;
bool reading = 1;
uint16_t count = 0;

uint16_t meas[MATRIX_COUNT][DATA_LENGTH] = {0};
uint16_t max_vals[MATRIX_COUNT][DATA_LENGTH] = {0};
uint8_t i = 0;
uint8_t row = 0, col = 0, m = 0;

void setup()
{ 
  pinMode(13, OUTPUT); // Turn LED on during setup
  digitalWrite(13, HIGH);
  
  //Set-up digital outputs for matrix serial reading
  for (col=0; col<X_DIM_TOT; col++){
    pinMode(col_pins[col], OUTPUT);
  }

  Serial.begin(115200);
  digitalWrite(13, LOW);
  delay(2000);

  // Print Serial Plotter labels for 3x3 matrices, un-/comment or modify
  // label lines to reflect the hardware setup
  Serial.print("A1,A2,A3,B1,B2,B3,C1,C2,C3,");
//  Serial.print("2-A1, 2-A2, 2-A3, 2-B1, 2-B2, 2-B3, 2-C1, 2-C2, 2-C3, ");
//  Serial.print("3-A1, 3-A2, 3-A3, 3-B1, 3-B2, 3-B3, 3-C1, 3-C2, 3-C3, ");
  Serial.println("");

  digitalWrite(13, HIGH);
}


void loop()
{
  while (reading){
    for (col=0; col<X_DIM_TOT; col++){ 
      //Pull line high with given analog value and delay  
      pinMode(col_pins[col], OUTPUT);
      digitalWrite(col_pins[col], HIGH);
      delay(delay1);
      m = uint8_t(col/X_DIM);
  
      // read all rows of activated column
      for (row=0; row<Y_DIM; row++){
        i = row*Y_DIM+(col%X_DIM);    
        meas[m][i] = analogRead(row_pins[row]);
        delay(delay2);
      }
      
      digitalWrite(col_pins[col], LOW);
      delay(delay3);
      pinMode(col_pins[col], INPUT);
    } 
  
    // Print out values
    for (m=0; m<MATRIX_COUNT; m++){
      for (row=0; row<Y_DIM; row++){
        for (col=0; col<X_DIM; col++){
          i = row*Y_DIM+col;
          Serial.print(meas[m][i]);
          Serial.print(",");
        }
      }
    }
    
    Serial.println("");
    count++;
    
    if (count == MAX_READS && LTD_MODE){
      reading = 0;
      digitalWrite(13, LOW);
    }
  }
}
