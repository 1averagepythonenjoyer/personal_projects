#include "Wire.h" 
#include <LiquidCrystal.h>
#include "Keypad.h"

char key2;
char key = NO_KEY;
int count = 0;
int count2 = 0;
int asd = 1;
const byte ROWS = 4;
const byte COLUMNS = 4;
char keys[ROWS][COLUMNS] = {
{'1','2','3', 'A'},
{'4','5','6', 'B'},
{'7','8','9', 'C'},
{'*','0','#', 'D'}
};

const int trigpin = 2; 
const int echopin = 3;
float duration, distance;



char inputlist[4];
char pass[] = "1234";
const int rs = 12, en = 11, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
const int MPU_ADDR = 0x68; 
float accelerometer_x, accelerometer_y, accelerometer_z; 
int ledinput;

byte rowPins[ROWS] = {22, 23, 24 , 25}; 
byte colPins[COLUMNS] = {26, 27 , 28, 29};    
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLUMNS);
LiquidCrystal lcd = LiquidCrystal(rs, en, d4, d5, d6, d7);



#define JoyX 8
#define PIN_AUTOPLAY 1
#define PIN_READWRITE 10
#define PIN_CONTRAST 12

#define SPRITE_RUN1 1
#define SPRITE_RUN2 2
#define SPRITE_JUMP 3
#define SPRITE_JUMP_UPPER '.'         
#define SPRITE_JUMP_LOWER 4
#define SPRITE_TERRAIN_EMPTY ' '     
#define SPRITE_TERRAIN_SOLID 5
#define SPRITE_TERRAIN_SOLID_RIGHT 6
#define SPRITE_TERRAIN_SOLID_LEFT 7

#define BOY_HORIZONTAL_POSITION 1   

#define TERRAIN_WIDTH 16
#define TERRAIN_EMPTY 0
#define TERRAIN_LOWER_BLOCK 1
#define TERRAIN_UPPER_BLOCK 2

#define BOY_POSITION_OFF 0          
#define BOY_POSITION_RUN_LOWER_1 1  
#define BOY_POSITION_RUN_LOWER_2 2  

#define BOY_POSITION_JUMP_1 3       
#define BOY_POSITION_JUMP_2 4      
#define BOY_POSITION_JUMP_3 5       
#define BOY_POSITION_JUMP_4 6       
#define BOY_POSITION_JUMP_5 7       
#define BOY_POSITION_JUMP_6 8       
#define BOY_POSITION_JUMP_7 9     
#define BOY_POSITION_JUMP_8 10     

#define BOY_POSITION_RUN_UPPER_1 11 
#define BOY_POSITION_RUN_UPPER_2 12


static char terrainUpper[TERRAIN_WIDTH + 1];
static char terrainLower[TERRAIN_WIDTH + 1];
bool buttonPushed = false;

void initializeGraphics(){
  static byte graphics[] = {

    B01100,
    B01100,
    B00000,
    B01110,
    B11100,
    B01100,
    B11010,
    B10011,

    B01100,
    B01100,
    B00000,
    B01100,
    B01100,
    B01100,
    B01100,
    B01110,

    B01100,
    B01100,
    B00000,
    B11110,
    B01101,
    B11111,
    B10000,
    B00000,

    B11110,
    B01101,
    B11111,
    B10000,
    B00000,
    B00000,
    B00000,
    B00000,

    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,
    B11111,

    B00011,
    B00011,
    B00011,
    B00011,
    B00011,
    B00011,
    B00011,
    B00011,

    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
    B11000,
  };
  int i;

  for (i = 0; i < 7; ++i) {
    lcd.createChar(i + 1, &graphics[i * 8]);
  }
  for (i = 0; i < TERRAIN_WIDTH; ++i) {
    terrainUpper[i] = SPRITE_TERRAIN_EMPTY;
    terrainLower[i] = SPRITE_TERRAIN_EMPTY;
  }
}

void advanceTerrain(char* terrain, byte newTerrain){
  for (int i = 0; i < TERRAIN_WIDTH; ++i) {
    char current = terrain[i];
    char next = (i == TERRAIN_WIDTH-1) ? newTerrain : terrain[i+1];
    switch (current){
      case SPRITE_TERRAIN_EMPTY:
        terrain[i] = (next == SPRITE_TERRAIN_SOLID) ? SPRITE_TERRAIN_SOLID_RIGHT : SPRITE_TERRAIN_EMPTY;
        break;
      case SPRITE_TERRAIN_SOLID:
        terrain[i] = (next == SPRITE_TERRAIN_EMPTY) ? SPRITE_TERRAIN_SOLID_LEFT : SPRITE_TERRAIN_SOLID;
        break;
      case SPRITE_TERRAIN_SOLID_RIGHT:
        terrain[i] = SPRITE_TERRAIN_SOLID;
        break;
      case SPRITE_TERRAIN_SOLID_LEFT:
        terrain[i] = SPRITE_TERRAIN_EMPTY;
        break;
    }
  }
}

bool drawBoy(byte position, char* terrainUpper, char* terrainLower, unsigned int score) {
  bool collide = false;
  char upperSave = terrainUpper[BOY_HORIZONTAL_POSITION];
  char lowerSave = terrainLower[BOY_HORIZONTAL_POSITION];
  byte upper, lower;
  switch (position) {
    case BOY_POSITION_OFF:
      upper = lower = SPRITE_TERRAIN_EMPTY;
      break;
    case BOY_POSITION_RUN_LOWER_1:
      upper = SPRITE_TERRAIN_EMPTY;
      lower = SPRITE_RUN1;
      break;
    case BOY_POSITION_RUN_LOWER_2:
      upper = SPRITE_TERRAIN_EMPTY;
      lower = SPRITE_RUN2;
      break;
    case BOY_POSITION_JUMP_1:
    case BOY_POSITION_JUMP_8:
      upper = SPRITE_TERRAIN_EMPTY;
      lower = SPRITE_JUMP;
      break;
    case BOY_POSITION_JUMP_2:
    case BOY_POSITION_JUMP_7:
      upper = SPRITE_JUMP_UPPER;
      lower = SPRITE_JUMP_LOWER;
      break;
    case BOY_POSITION_JUMP_3:
    case BOY_POSITION_JUMP_4:
    case BOY_POSITION_JUMP_5:
    case BOY_POSITION_JUMP_6:
      upper = SPRITE_JUMP;
      lower = SPRITE_TERRAIN_EMPTY;
      break;
    case BOY_POSITION_RUN_UPPER_1:
      upper = SPRITE_RUN1;
      lower = SPRITE_TERRAIN_EMPTY;
      break;
    case BOY_POSITION_RUN_UPPER_2:
      upper = SPRITE_RUN2;
      lower = SPRITE_TERRAIN_EMPTY;
      break;
  }
  if (upper != ' ') {
    terrainUpper[BOY_HORIZONTAL_POSITION] = upper;
    collide = (upperSave == SPRITE_TERRAIN_EMPTY) ? false : true;
  }
  if (lower != ' ') {
    terrainLower[BOY_HORIZONTAL_POSITION] = lower;
    collide |= (lowerSave == SPRITE_TERRAIN_EMPTY) ? false : true;
  }
  
  byte digits = (score > 9999) ? 5 : (score > 999) ? 4 : (score > 99) ? 3 : (score > 9) ? 2 : 1;
  
  // Draw the scene
  terrainUpper[TERRAIN_WIDTH] = '\0';
  terrainLower[TERRAIN_WIDTH] = '\0';
  char temp = terrainUpper[16-digits];
  terrainUpper[16-digits] = '\0';
  lcd.setCursor(0,0);
  lcd.print(terrainUpper);
  terrainUpper[16-digits] = temp;  
  lcd.setCursor(0,1);
  lcd.print(terrainLower);
  
  lcd.setCursor(16 - digits,0);
  lcd.print(score);

  terrainUpper[BOY_HORIZONTAL_POSITION] = upperSave;
  terrainLower[BOY_HORIZONTAL_POSITION] = lowerSave;
  return collide;
}

void buttonPush() {
  buttonPushed = true;
  Serial.println(buttonPushed);
  
}

void setup() {
  Serial.begin(9600);

  while(asd == 1){
    while (count2 < 4){
    char key = keypad.getKey();
      if (key != NO_KEY){
        inputlist[count2] = key;
        lcd.print(key);
        count2 ++;    
    }
    }
    if (count2 == 4){
      delay(175);
      lcd.clear();
    }
    
    for (int j = 0; j < 4; j++){
      if (inputlist[j] != pass[j]){
        count ++;
      }
    }
    if (count == 0){
      asd = 0;
      
    }else{
      count2 = 0;
      count = 0;
    }
  } 
  

  pinMode(trigpin, OUTPUT);
  pinMode(echopin, INPUT);

  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); 
  Wire.write(0x6B); 
  Wire.write(0); 
  Wire.endTransmission(true);

  pinMode(A0, OUTPUT);
  lcd.begin(16, 2);
  lcd.print("Ax By Cz D *");

  pinMode(PIN_READWRITE, OUTPUT);
  digitalWrite(PIN_READWRITE, LOW);
  pinMode(PIN_CONTRAST, OUTPUT);
  digitalWrite(PIN_CONTRAST, LOW);
  pinMode(JoyX, INPUT);
  digitalWrite(JoyX, HIGH);
  pinMode(PIN_AUTOPLAY, OUTPUT);
  digitalWrite(PIN_AUTOPLAY, HIGH);
  
  // Digital pin 2 maps to interrupt 0
  attachInterrupt(0/*PIN_BUTTON*/, buttonPush, FALLING);
  
  initializeGraphics();

}




void displayacc(){
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); 
  Wire.endTransmission(false); 
  Wire.requestFrom(MPU_ADDR, 7*2, true); 
  accelerometer_x = 9.81*(Wire.read()<<8 | Wire.read())/16384 + 0.18; 
  accelerometer_y = (9.81*(Wire.read()<<8 | Wire.read())/16384) - 0.4; 
  accelerometer_z = (9.81*(Wire.read()<<8 | Wire.read())/16384) - 0.45; 
  lcd.clear();
  if (key2 == 'A'){
    lcd.print(accelerometer_x);
  }
  if (key2 == 'B'){
    lcd.print(accelerometer_y);
  }
  if (key2 == 'C'){
    lcd.print(accelerometer_z);
  }    
  lcd.print("m/s^2");
  lcd.display();
  delay(100);
}

void checkdistance(){
  digitalWrite(trigpin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigpin, HIGH);
  delayMicroseconds(10);  
	digitalWrite(trigpin, LOW);

  duration = pulseIn(echopin, HIGH);  
  distance = (duration*.0343)/2;
  lcd.clear();
  lcd.print(distance);
  lcd.print("cm");
  lcd.display();
  delay(100);
}

void loop() {
  
  key = keypad.getKey();

  if (key == 'A'||key == 'B'||key == 'C'){
    key2 = key;
    key == NO_KEY;
    do{
      displayacc();
      key = keypad.getKey();
    }while (key == NO_KEY);
  }

  if (key == 'D'){
    key == NO_KEY;
    do{
      checkdistance();

      key = keypad.getKey();
    }while (key == NO_KEY);
  }

  if (key == '*'){
    key == NO_KEY;
    do{
      static byte boyPos = BOY_POSITION_RUN_LOWER_1;
  static byte newTerrainType = TERRAIN_EMPTY;
  static byte newTerrainDuration = 1;
  static bool playing = false;
  static bool blink = false;
  static unsigned int distance = 0;
  
  if (!playing) {
    drawBoy((blink) ? BOY_POSITION_OFF : boyPos, terrainUpper, terrainLower, distance >> 3);
    if (blink) {
      lcd.setCursor(0,0);
      lcd.print("Press Start");
    }
    delay(250);
    blink = !blink;
    if (buttonPushed) {
      initializeGraphics();
      boyPos = BOY_POSITION_RUN_LOWER_1;
      playing = true;
      buttonPushed = false;
      distance = 0;
    }
    return;
  }

  // Shift the terrain to the left
  advanceTerrain(terrainLower, newTerrainType == TERRAIN_LOWER_BLOCK ? SPRITE_TERRAIN_SOLID : SPRITE_TERRAIN_EMPTY);
  advanceTerrain(terrainUpper, newTerrainType == TERRAIN_UPPER_BLOCK ? SPRITE_TERRAIN_SOLID : SPRITE_TERRAIN_EMPTY);
  
  // Make new terrain to enter on the right
  if (--newTerrainDuration == 0) {
    if (newTerrainType == TERRAIN_EMPTY) {
      newTerrainType = (random(3) == 0) ? TERRAIN_UPPER_BLOCK : TERRAIN_LOWER_BLOCK;
      newTerrainDuration = 2 + random(10);
    } else {
      newTerrainType = TERRAIN_EMPTY;
      newTerrainDuration = 10 + random(10);
    }
  }
    
  if (buttonPushed) {
    if (boyPos <= BOY_POSITION_RUN_LOWER_2) boyPos = BOY_POSITION_JUMP_1;
    buttonPushed = false;
  }  

  if (drawBoy(boyPos, terrainUpper, terrainLower, distance >> 3)) {
    playing = false; // The boy collided with something. Too bad.
  } else {
    if (boyPos == BOY_POSITION_RUN_LOWER_2 || boyPos == BOY_POSITION_JUMP_8) {
      boyPos = BOY_POSITION_RUN_LOWER_1;
    } else if ((boyPos >= BOY_POSITION_JUMP_3 && boyPos <= BOY_POSITION_JUMP_5) && terrainLower[BOY_HORIZONTAL_POSITION] != SPRITE_TERRAIN_EMPTY) {
      boyPos = BOY_POSITION_RUN_UPPER_1;
    } else if (boyPos >= BOY_POSITION_RUN_UPPER_1 && terrainLower[BOY_HORIZONTAL_POSITION] == SPRITE_TERRAIN_EMPTY) {
      boyPos = BOY_POSITION_JUMP_5;
    } else if (boyPos == BOY_POSITION_RUN_UPPER_2) {
      boyPos = BOY_POSITION_RUN_UPPER_1;
    } else {
      ++boyPos;
    }
    ++distance;
    
    digitalWrite(PIN_AUTOPLAY, terrainLower[BOY_HORIZONTAL_POSITION + 2] == SPRITE_TERRAIN_EMPTY ? HIGH : LOW);
  }

      key = keypad.getKey();
    }while (key == NO_KEY);
  }

  
}
