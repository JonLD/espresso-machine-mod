#define DEBUG true // activate print debugs

#include <Arduino.h>
#include <EEPROM.h>

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
#include <Fonts/FreeMono9pt7b.h>

// OLED parameters
const uint8_t SCREEN_WIDTH  = 128;            // OLED display width, in pixels
const uint8_t SCREEN_HEIGHT = 64;             // OLED display height, in pixels
const int OLED_RESET  = -1;                   // Reset pin # (or -1 if sharing Arduino reset pin)
const int SCREEN_ADDRESS = 0x3C;              // See datasheet for screen address
#define FONT FreeSans9pt7b         // default font
#define  BOLD_FONT FreeSansBold9pt7b  // bold font
#define NEW_FONT FreeMono9pt7b
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Declaration for an SSD1306 display

// rotary encoder (RE) variables
const uint8_t ROTARY_BUTTON_PIN = 6;
const uint8_t RE_PIN_A = 5;
const uint8_t RE_PIN_B = 4;

const uint8_t PUMP_PIN = 3;

volatile bool target_weight_adjusted = false;
volatile float target_weight = 24;
bool target_adjustment_allowed = true;
float weight;
float previous_weight;
double extraction_time;
unsigned long previous_extraction_time;
int stored_target_weight;
int stored_previous_weight;
unsigned long stored_previous_extraction_time;
unsigned long extraction_end_time;
const unsigned long DRIP_TIME = 3000;

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

float overshoot = 0;

// extraction states
enum state {
  state_pre_extraction = 1,
  state_extracting = 2,
  state_post_extraction = 3
};
int current_state = state_pre_extraction;

// Function prototypes
void display_pre_extraction();
void display_extraction();
void display_post_extraction();
void adjust_target_weight();
void update_weight();
void pre_extraction();
void extracting();
void post_extraction();
void button_option_select();
void get_stored_values();
void update_eeprom();

// ------------- SETUP ------------
void setup() {

  Serial.begin(9600);


  pinMode(RE_PIN_A, INPUT_PULLUP);
  pinMode(RE_PIN_B, INPUT_PULLUP);
  pinMode(3, OUTPUT);
  pinMode(ROTARY_BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RE_PIN_A),  adjust_target_weight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_BUTTON_PIN), button_option_select, FALLING);

  get_stored_values();

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
        if (extraction_time < 10000) {
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

// get the weight stored in EEPROM memory (preserved over Arduino shut off)
void get_stored_values() {
    // IF first time you are writing to the eeprom and if it is just put a 0 for now
  if (EEPROM.read(256) != 123) {
    EEPROM.write(256, 123);
    stored_target_weight = 0;
    stored_previous_weight = 0;
    stored_previous_extraction_time = 0;
  }

  else {
    EEPROM.get(0, stored_target_weight);
    EEPROM.get(1, stored_previous_weight);
    EEPROM.get(2, stored_previous_extraction_time);
  }
//  convert back into float
  target_weight = stored_target_weight*0.1;

}


void pre_extraction() {
  first_loop = true;
  display_pre_extraction(); 
}

void extracting() {
  stored_target_weight = target_weight;
  loadcell_1.tare();
  loadcell_2.tare();
  update_weight;
  extraction_time = 0;
  display_extraction();
  digitalWrite(PUMP_PIN, HIGH); // switch on pump
  unsigned long start_time = millis();
  while (current_state == state_extracting && extraction_time < 50) { // Check if in extracting state and limit max extraction length
    // End extraction if correct weight but not for start of extraction
    // This prevents possible early ending when adjusting cup
    if (weight == target_weight - overshoot && extraction_time > 8) { 
      break;
    }
    extraction_time = (millis() - start_time) * 0.001;
    update_weight();
    display_extraction();
  }
  digitalWrite(3, LOW);
  extraction_end_time = millis();
  if (extraction_time > 10) {
      current_state = state_post_extraction;
  }
}

void post_extraction() {
  
  while (millis() - extraction_end_time < DRIP_TIME) {
    update_weight();
    display_extraction();
  }
  if (first_loop) {
    update_weight();
    display_post_extraction();
    previous_weight = weight;
    previous_extraction_time = extraction_time;
    update_eeprom();
    first_loop = false;
  }

}

// ---- State displays ----
void display_pre_extraction() {
    display.clearDisplay();
    display.setFont(&FONT);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 24);
    display.print("Target: ");
    display.print(target_weight, 1);
    display.println("g");
    display.print(previous_weight, 1);
    display.print("   ");
    display.print(previous_extraction_time, 1);
    display.display();
}

void display_extraction() {
  display.clearDisplay();
  display.setFont(&FONT);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 24);
  display.print(extraction_time, 1);
  display.println("s");
  display.print("Weight: ");
  display.println(weight, 1);
  display.display();
    
}

void display_post_extraction() {
  display.clearDisplay();
  display.setFont(&BOLD_FONT);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 24);
  display.print(weight, 1);
  display.print("g");
  display.print("     ");
  display.print(extraction_time, 1);
  display.println("s");
  display.print(previous_weight, 1);
  display.print("     ");
  display.println(previous_extraction_time, 1);
  display.display();
}

void update_eeprom() {
  if (stored_target_weight != round(target_weight/0.1)) {
      EEPROM.put(0, stored_target_weight);
  } 
  EEPROM.put(1, stored_previous_weight);
  EEPROM.put(2, stored_previous_extraction_time);
  Serial.print("eeprom updated");
}

void update_weight() {
  weight = loadcell_1.get_units() + loadcell_2.get_units(); // Average loadcells
}

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