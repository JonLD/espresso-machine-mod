#define DEBUG true // activate print debugs

#include <Arduino.h>
#include <EEPROM.h>

#include <cppQueue.h> // Queue implementation
const bool OVERWRITE = true;
const uint8_t NB_RECS = 10;
cppQueue  weight_queue(4, 2, FIFO, true);

#include <HX711.h> // loadcell amplifier library
HX711 loadcell_1;
HX711 loadcell_2;

// oled libraries
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/Picopixel.h>

// OLED parameters
const uint8_t SCREEN_WIDTH  = 128;            // OLED display width, in pixels
const uint8_t SCREEN_HEIGHT = 64;             // OLED display height, in pixels
const int OLED_RESET  = -1;                   // Reset pin # (or -1 if sharing Arduino reset pin)
const int SCREEN_ADDRESS = 0x3C;              // See datasheet for screen address
#define FONT9 FreeSans9pt7b      // Font with 21 pixel height
#define FONT12 FreeSans12pt7b    // Font with 28 pixel height
#define FONT18 FreeSans18pt7b    // Font with 41 pixel height
#define  BOLD_FONT FreeSansBold9pt7b  // Bold font 21 pixel height
#define NEW_FONT Picopixel
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // SSD1306 OLED display object

// rotary encoder (RE) variables
const uint8_t ROTARY_BUTTON_PIN = 6;
const uint8_t RE_PIN_A = 5;
const uint8_t RE_PIN_B = 4;

const uint8_t PUMP_PIN = 3;

volatile bool target_weight_adjusted = false;
volatile float target_weight = 30;
bool target_adjustment_allowed = true;
float weight;
float previous_weight;
double extraction_time;
double previous_extraction_time;
float stored_target_weight;
float stored_previous_weight;
float stored_previous_extraction_time;
unsigned long extraction_end_time;

// ---- Tuning varialbes -------
const unsigned long DRIP_TIME = 3000;
const float offset_tuning = 500;


// ---- LOADCELL ----
const uint8_t LOADCELL_1_DOUT_PIN = 10;
const uint8_t LOADCELL_1_SCK_PIN = 11;
const uint8_t LOADCELL_2_DOUT_PIN = 12;
const uint8_t LOADCELL_2_SCK_PIN = 13;

const long LOADCELL_1_OFFSET = 1091; //adjustment parameters
const long LOADCELL_1_DIVIDER = 1091;
const long LOADCELL_2_OFFSET = 1091; //adjustment parameters
const long LOADCELL_2_DIVIDER = 1091;

bool first_loop = true;


// extraction states
enum state {
  state_pre_extraction = 1,
  state_extracting = 2,
  state_post_extraction = 3,
  state_tune_overshoot = 4
};
int current_state = state_pre_extraction;

// Function prototypes
void display_pre_extraction();
void display_extraction();
void display_post_extraction();
void adjust_target_weight();
void update_weight(int);
void pre_extraction();
void extracting();
void post_extraction();
void button_option_select();
void get_eeprom();
void update_eeprom();
void tare_scales();
float overshoot(float);
void tune_overshoot();

// ------------- SETUP ------------
void setup() {

  Serial.begin(9600);


  pinMode(RE_PIN_A, INPUT_PULLUP);
  pinMode(RE_PIN_B, INPUT_PULLUP);
  pinMode(3, OUTPUT);
  pinMode(ROTARY_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RE_PIN_A),  adjust_target_weight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_BUTTON_PIN), button_option_select, FALLING);

  get_eeprom();

  // Initialize loadcell library
  loadcell_1.begin(LOADCELL_1_DOUT_PIN, LOADCELL_1_SCK_PIN);
  loadcell_1.set_scale(LOADCELL_1_DIVIDER);
  loadcell_1.set_offset(LOADCELL_1_OFFSET);
  loadcell_2.begin(LOADCELL_2_DOUT_PIN, LOADCELL_2_SCK_PIN);
  loadcell_2.set_scale(LOADCELL_2_DIVIDER);
  loadcell_2.set_offset(LOADCELL_2_OFFSET);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display_pre_extraction();
}

// ------------- LOOP ------------
void loop() {

  switch (current_state) {
    case (state_pre_extraction):
      pre_extraction();
      break;
    case (state_extracting):
      extracting();
      break;
    case (state_post_extraction):
      post_extraction();
      break;
    case (state_tune_overshoot):
      tune_overshoot();
  }
}


void button_option_select() {
  static unsigned long previous_button_press = 0;
  if ((millis() - previous_button_press) > 300){
    switch (current_state) {
      case (state_pre_extraction):
        current_state = state_extracting;
        break;
      case (state_extracting):
        if (extraction_time < 8) {
          current_state = state_pre_extraction;
          break;
          }
        else {
          current_state = state_post_extraction;
          break;
        }
      case (state_post_extraction):
        current_state = state_pre_extraction;
        break;
    }
    previous_button_press = millis();
  }
}

void pre_extraction() {
  first_loop = true;
  display_pre_extraction();
}

void extracting() {
  extraction_time = 0;                 // Zero extraction time
  tare_scales();
  update_weight(1);
  display_extraction();                // Start displaying in extraction state
  digitalWrite(PUMP_PIN, HIGH);        // switch on pump
  unsigned long start_time = millis(); // Time extraction was started at

  // Check if in extracting state and limit max extraction length
  while (current_state == state_extracting && extraction_time < 50) { 
    // End extraction if correct weight but not for start of extraction
    // This prevents possible early ending when adjusting cup
    if (weight > target_weight - overshoot(offset_tuning) && extraction_time > 10) { 
      break;
    }
    extraction_time = (millis() - start_time) * 0.001;
    update_weight(1);
    display_extraction();
  }
  digitalWrite(3, LOW);
  extraction_end_time = millis();
  if (extraction_time > 8) {
      current_state = state_post_extraction;
  }
}

void post_extraction() {
  
  while (millis() - extraction_end_time < DRIP_TIME) {
    update_weight(1);
    display_extraction();
  }
  if (first_loop) {
    update_weight(2);
    display_post_extraction();
    previous_weight = weight;
    previous_extraction_time = extraction_time;
    update_eeprom();
    first_loop = false;
  }

}

void tune_overshoot() {
 // overshoot tuning to go here
}

// ---- State displays ----
void display_pre_extraction() {
  display.clearDisplay();
  display.setFont(NULL);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);     
  display.setCursor(0, 0);   
  display.println("Target Weight:");
  display.setTextSize(1);  
  display.setFont(&FONT18);
  display.setCursor(0, 35);      
  display.print(target_weight, 1);
  display.setCursor(68, 34);
  display.print("g");
  display.setFont(NULL);
  display.setTextSize(1); 
  display.setCursor(0, 46);
  display.print("Previous Weight:");
  display.setCursor(98, 46);   
  display.print(previous_weight, 1);
  display.print("g");
  display.setCursor(0, 56);
  display.print("Previous Time:");    
  display.setCursor(98, 56);   
  display.print(previous_extraction_time, 1);
  display.print("s");
  display.display();
}

void display_extraction() {
  display.clearDisplay();
  
  display.setFont(&FONT9);          // Target weight is shown at top of screen
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);     
  display.setCursor(0,12);
  display.print("Target: ");       
  display.print(target_weight, 1);
  display.print("g");

  display.setCursor(1, 34);         // Titles positioned above weight and time values
  display.print("Weight");
  display.setCursor(72, 34);
  display.print("Time");

  display.setFont(&FONT12);         // Display final weight and time of during extraction
  if (weight >= 0 && weight < 10) { // This logic is position the numbers correct independent of number of digits or sign
    display.setCursor(13,58);
    display.setCursor(13,58);
  }
  else if (weight > 10) {
  display.setCursor(0,58);
  }
  else if (weight < 0) {
    display.setCursor(5, 58);
  }
  display.print(weight, 1);
  display.print("g");     
  display.print("   ");

  if (extraction_time < 10) {
    display.setCursor(82, 58);
  }
  else {
    display.setCursor(68, 58);
  }
  display.print(extraction_time, 1);
  display.print("s");
  display.display();
}

void display_post_extraction() {
  display.clearDisplay();
  
  display.setFont(NULL);            // display previous Weight and time for comparison purposes
  display.setTextSize(1); 
  display.setCursor(0, 0);
  display.print("Previous Weight:");
  display.setCursor(98, 0);   
  display.print(previous_weight, 1);
  display.print("g");
  display.setCursor(0, 10);
  display.print("Previous Time:");    
  display.setCursor(98, 10);   
  display.print(previous_extraction_time, 1);
  display.print("s");


  display.setFont(&FONT9);          // Display final weight and time of extraction
  display.setCursor(1, 34);
  display.print("Weight");
  display.setCursor(72, 34);
  display.print("Time");
  display.setFont(&FONT12);
  if (weight >= 0 && weight < 10) { // This logic is position the numbers correct independent of number of digits or sign
    display.setCursor(13,58);
  }
  else if (weight > 10) {
  display.setCursor(0,58);
  }
  else if (weight < 0) {
    display.setCursor(5, 58);
  }
  display.print(weight, 1);
  display.print("g");     
  display.print("   ");

  if (extraction_time < 10) {
    display.setCursor(82, 58);
  }
  else {
    display.setCursor(68, 58);
  }
  display.print(extraction_time, 1);
  display.print("s");
  display.display();
}

float overshoot(float offset_tuning) {
  static unsigned long previous_sample_time;
  if (millis() - previous_sample_time > 500) {
    update_weight(1);
    weight_queue.push(&weight);
    if (weight_queue.isFull()) {
      float weight_gradient = (weight_queue.peekIdx(&weight, 0) + weight_queue.peekIdx(&weight, 1))
                              / (millis() - previous_sample_time);
      float mass_overshoot = weight_gradient * offset_tuning;
      return mass_overshoot;
    }
  }
}


void tare_scales() {
  // Tare both loadcells
  loadcell_1.tare();
  loadcell_2.tare();
}

void update_weight(int num_of_measurements) {
  weight = loadcell_1.get_units(num_of_measurements) + loadcell_2.get_units(num_of_measurements); // Average loadcells
}


// ------- Rotary Encoder -------
void adjust_target_weight() {
  /*RE_PIN_A and RE_PIN_B both defualt to HIGH
  On rotation both pass through LOW to HIGH*/
  static uint8_t pin_a_current_state = HIGH;
  static uint8_t pin_a_previous_state = pin_a_previous_state;

  if (current_state == state_pre_extraction) { // Am I allowed to adjust weight
    
    pin_a_current_state = digitalRead(RE_PIN_A);
    if ((pin_a_previous_state == LOW) && (pin_a_current_state == HIGH)) {

      // if counter clockwise RE_PIN_B returns to HIGH before RE_PIN_A and so will be HIGH currently
      if ((digitalRead(RE_PIN_B) == HIGH) && (target_weight >= 0.1)) {
        target_weight -= 0.1;
        }
      // if clockwise RE_PIN_B returns to HIGH after RE_PIN_A and thus will still be low currently
      else if (digitalRead(RE_PIN_B) == LOW) {
        target_weight += 0.1;
      }
      target_weight_adjusted = true;
    }
    pin_a_previous_state = pin_a_current_state;
  }
}

// --------- EEPROM ----------
// This is called when arduino is powered on
// It retreives saved values of target_weight previous_weight and previous_extraction_time
void get_eeprom() {

    // IF first time you are writing to the eeprom and if it is just put a 0 for now
  if (EEPROM.read(255) != 123) {
    EEPROM.write(255, 123);
    stored_target_weight = 300;
    stored_previous_weight = 0;
    stored_previous_extraction_time = 0;
  }

  else {
    EEPROM.get(0, stored_target_weight);
    int prev_weight_address = sizeof(stored_target_weight);
    EEPROM.get(prev_weight_address, stored_previous_weight);
    int prev_extraction_time_address = prev_weight_address + sizeof(stored_previous_weight);
    EEPROM.get(prev_extraction_time_address, stored_previous_extraction_time);
  }
  target_weight = stored_target_weight;
  previous_weight = stored_previous_weight;
  previous_extraction_time = stored_previous_extraction_time;
}

void update_eeprom() {

  int prev_weight_address = sizeof(stored_target_weight);
  int prev_extraction_time_address = prev_weight_address + sizeof(stored_previous_weight);

  if (stored_target_weight != target_weight) {
    stored_target_weight = target_weight;
    EEPROM.put(0, stored_target_weight);
    Serial.print(sizeof(stored_target_weight));
  }
  if (stored_previous_weight != previous_weight) {
    stored_previous_weight = previous_weight;
    EEPROM.put(prev_weight_address, stored_previous_weight);
  }
  if (stored_previous_extraction_time != previous_extraction_time) {
    stored_previous_extraction_time = previous_extraction_time;
    EEPROM.put(prev_extraction_time_address, stored_previous_extraction_time);
  }
}

