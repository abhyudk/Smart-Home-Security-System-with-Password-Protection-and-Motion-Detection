#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <Keypad.h>

// Pin Definitions
const int pirPin = 7;         // PIR sensor pin
const int buzzerPin = 3;      // Buzzer pin
const int ledPin = 5;         // LED pin
const int servoPin = 6;       // Servo pin

// Keypad configuration
const byte ROW_NUM    = 4;    // four rows
const byte COL_NUM    = 4;    // four columns
char keys[ROW_NUM][COL_NUM] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
byte pin_rows[ROW_NUM] = {10, 11, 12, 13};     // Pins for the rows
byte pin_column[COL_NUM] = {A0, A1, A2, A3};   // Pins for the columns

Keypad keypad = Keypad( makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COL_NUM );   //2d array to keypad object understand format

// LCD Setup (Address 0x27, 16x2 display)
LiquidCrystal_I2C lcd(0x27, 16, 2); // 16x2 LCD with I2C communication

// Variables
Servo doorServo; //wer making an object
const int correctPassword = 1234; // Correct password for entry
int enteredPassword = 0;
int passwordDigitCount = 0;
bool isPasswordCorrect = false;
bool doorIsOpen = false;       // Door status flag
bool motionHandled = false;    // Flag to ensure we only process motion once
unsigned long lastMotionTime = 0; // To track the last motion time (for debouncing)
unsigned long motionDebounceTime = 2000; // 2 seconds debounce time for motion sensor

// State Management
bool waitingForPassword = true; // Flag to check if we are waiting for password input
enum SystemState { WAITING_FOR_PASSWORD, DOOR_OPENED, EXIT_STATE, SYSTEM_READY }; //what all possible states , enumeration
SystemState currentState = WAITING_FOR_PASSWORD; // Start by waiting for password input

void setup() {
  // Initialize pins
  pinMode(pirPin, INPUT);
  pinMode(buzzerPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  // Initialize servo
  doorServo.attach(servoPin); //atttach to servo pin to use n control along with library
  doorServo.write(0); // Start with door closed , servo at 0 degrees
  digitalWrite(buzzerPin, LOW);
  digitalWrite(ledPin, LOW);

  // Initialize LCD
  lcd.init();
  lcd.backlight(); // Initialize the LCD with 16 columns, 2 rows
  lcd.clear();      // Clear the display
  lcd.setCursor(0, 0); // Start cursor at the top left corner
  lcd.print("System Ready");

  // Start Serial Monitor for debugging
  Serial.begin(9600);
  Serial.println("System Ready");
}

void loop() {
  // Step 1: Password Prompt and Input 
  if (waitingForPassword) {
    promptPassword(); // Always prompt for password only once if wer in waiting for pass state
  }

  // Step 2: If password is correct, immediately open and close the door (servo)
  if (isPasswordCorrect && currentState == WAITING_FOR_PASSWORD) {
    currentState = SYSTEM_READY;  // Transition to system ready state
    Serial.println("System is ready for entry");
    lcd.setCursor(0, 1);
    lcd.print("Access Granted");
    
  }

  // Step 3: Handle Keypad input for password
  char key = keypad.getKey(); //checks if key has been pressed
  if (key) {
    Serial.print("Key pressed: ");
    Serial.println(key);

    if (key >= '0' && key <= '9' && passwordDigitCount < 4) {
      enteredPassword = enteredPassword * 10 + (key - '0'); //for ascii value conversion
      passwordDigitCount++;
      lcd.setCursor(passwordDigitCount - 1, 1); // Move cursor to password digit position
      lcd.print("*"); // Mask password input for security
    }

    // If 4 digits have been entered, check if it's correct
    if (passwordDigitCount == 4) {
      if (enteredPassword == correctPassword) {
        Serial.println("\nAccess Granted");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Access Granted");
        isPasswordCorrect = true; // Grant access and open door
        enteredPassword = 0;
        passwordDigitCount = 0;
        waitingForPassword = false; // Stop waiting for password
        openAndCloseDoor(); // Open and close the door immediately after correct password
      } else {
        Serial.println("\nAccess Denied");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Access Denied");
        enteredPassword = 0;
        passwordDigitCount = 0;
        lcd.setCursor(0, 1);
        lcd.print("Enter Password:");
      }
    }
  }

  // Step 4: Handle PIR motion sensor and actions
  if (digitalRead(pirPin) == HIGH) { // PIR sensor detects motion
    unsigned long currentTime = millis(); //current time in ms since pgrm started

    // Only process the motion if it hasn't been processed recently (debounce)
    if (!motionHandled && currentTime - lastMotionTime > motionDebounceTime) {
      lastMotionTime = currentTime; // Update last motion time
      motionHandled = true; // Mark motion as handled

      Serial.println("Motion detected!");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Motion Detected");

      // Step 5: If the password was incorrect and motion is detected, trigger buzzer
      if (!isPasswordCorrect) {
        digitalWrite(buzzerPin, HIGH);  // Turn on the buzzer
        delay(1000);                    // Buzzer beeps for 1 second
        digitalWrite(buzzerPin, LOW);   // Turn off the buzzer
      }

      // Step 6: Handle entry and exit states based on the system state
      if (currentState == SYSTEM_READY) {
        // Entry detected but don't open the door if the system is in entry mode
        digitalWrite(ledPin, HIGH); // LED ON (Entry)
        currentState = DOOR_OPENED;
        Serial.println("You have entered. Light ON.");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("You Have Entered");
      } 
      else if (currentState == DOOR_OPENED) {
        // Exit detected: Turn off LED, close the door, and reset system
        digitalWrite(ledPin, LOW); // LED OFF (Exit)
        Serial.println("LED OFF (Exit)");
        // Open and close the door 
        openAndCloseDoor();
        currentState = WAITING_FOR_PASSWORD; // Reset to password input state
        Serial.println("Exit complete. Please enter password again.");
        isPasswordCorrect = false; // Reset password
        waitingForPassword = true; // Start waiting for password again
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Please Enter Password");
      }
    }
  } else {
    // Reset motionHandled flag if PIR sensor stops detecting motion
    motionHandled = false;
  }

  delay(100); // Small delay for stability , debouncing
}

// Function to prompt for password 
void promptPassword() {
  lcd.setCursor(0, 0);  // Set cursor to the first line
  lcd.print("Enter Password:"); // Prompt message
}

// Function to open and close the door (move servo to 90 degrees and back)
void openAndCloseDoor() {
  doorServo.write(90); // Open the door (servo to 90 degrees)
  delay(1000);          // Wait for door to open
  doorServo.write(0);   // Close the door (servo to 0 degrees)
  delay(1000);          // Wait for door to close
}