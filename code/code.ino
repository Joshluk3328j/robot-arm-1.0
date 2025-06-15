#include <Servo.h>  // Library to control servo motors
#include <EEPROM.h> // Library to save data to Arduino's memory even after power-off

// --- Pin Setup ---
// Define which Arduino pins are connected to the 5 servo motors
const int servoPins[5] = {3, 5, 6, 9, 10};
// Define pins for the 5 potentiometers (knobs) that control servo positions
const int potPins[5] = {A1, A2, A3, A4, A5};
// Define pins for two buttons and a buzzer for sound feedback
const int button1Pin = 2;  // Button to save motion or switch modes
const int button2Pin = 4;  // Button to pause/resume or delete motions
const int buzzerPin = 7;   // Buzzer for sound signals

// --- Custom Home Angles ---
// Define home angles for each servo (in degrees, 0 to 180)
const int homeAngles[5] = {90, 90, 90, 90, 90}; // Set your desired home angles here

// --- Motion Storage ---
// Set maximum number of motion frames we can save (each frame stores 5 servo angles)
const int MAX_MOTIONS = 100;
// Array to store up to 100 motion frames, each with 5 servo angles
byte motions[MAX_MOTIONS][5]; //motions[100][5]
// Keep track of how many motions are saved
int motionCount = 0;
// Track which motion is currently playing during playback
int motionIndex = 0;

// --- Timing and Transition Control ---
// Store when a servo movement (transition) starts
unsigned long transitionStartTime = 0;
// Duration for smooth servo movements (2000ms = 2 seconds)
unsigned long transitionDuration = 2000;
// Store when a delay (pause) starts
unsigned long delayStartTime = 0;
// 1-second pause between each motion during playback
const unsigned long motionDelay = 1000;
// 3-second pause after last motion before returning to home
const unsigned long endDelay = 3000;
// Flags to manage servo movement and delays
bool transitioning = false;          // True when servos are moving
bool returningHome = false;         // True when moving back to home position
bool transitioningToControl = false; // True when switching to manual control
bool inDelay = false;               // True when waiting during a pause/delay

// --- Mode Management ---
// Define two modes: CONTROL_MODE (manual control via knobs) and PLAY_MODE (automatic playback)
enum Mode { CONTROL_MODE, PLAY_MODE };
Mode currentMode = CONTROL_MODE; // Start in manual control mode
bool isPaused = false;          // True when playback is paused

// --- Servo and Angle Tracking ---
// Create objects to control the 5 servo motors
Servo servos[5];
// Store current angles for each servo (0 to 180 degrees)
int servoAngles[5];
// Store starting angles for smooth movement calculations
int startAngles[5] = {0};
// Store target angles for smooth movement calculations
int targetAngles[5] = {0};

// --- Button Debouncing ---
// Time to detect a double-click (600ms window)
const unsigned long doubleClickGap = 600;
// Time to ignore button bounces (50ms)
const unsigned long debounceDelay = 50;
// Track button presses and states for debouncing
unsigned long lastButton1Press = 0, lastButton1Change = 0;
unsigned long lastButton2Press = 0, lastButton2Change = 0;
bool button1SinglePending = false, button2SinglePending = false;
bool button1State = HIGH, button2State = HIGH; // we're using the inbuilt pullup resistor
bool lastButton1Reading = HIGH, lastButton2Reading = HIGH;

// --- Save Motions to Memory ---
// Function to save recorded motions to Arduino's EEPROM (permanent memory)
void saveMotionsToEEPROM() {
  EEPROM.update(0, motionCount); // Save number of motions into EEPROM index 0
  // Save each motion frame (5 servo angles) to EEPROM
  for (int i = 0; i < motionCount; i++) { // imagine motions = [[a00,a01,a02,a03,a04],[b00,b01,b02,b03,b04], ...]
    for (int j = 0; j < 5; j++) {
      EEPROM.update(1 + i * 5 + j, motions[i][j]); // the eeprom is a flat(1d) array so we're using this algorithm to save motions from "motions"(2d array) array into a flat(1d) array
    } // the algorithm is read(1 + current_row_index * total_number_of_columns + current_column_index). the 1 accounts for the first EEPROM index used to store motion counts
  }
  Serial.println("Motions saved to EEPROM.");
}

// --- Load Motions from Memory ---
// Function to load saved motions from EEPROM when Arduino starts
void loadMotionsFromEEPROM() {
  motionCount = constrain(EEPROM.read(0), 0, MAX_MOTIONS); // Read number of motions and ensure it doesn't exceed MAX_MOTIONS
  // Load each motion frame (5 servo angles) from EEPROM
  for (int i = 0; i < motionCount; i++) { // imagine motions = [[a00,a01,a02,a03,a04],[b00,b01,b02,b03,b04], ...]
    for (int j = 0; j < 5; j++) {
      motions[i][j] = EEPROM.read(1 + i * 5 + j);
    } 
  }
  Serial.print("Loaded "); Serial.print(motionCount); Serial.println(" motions from EEPROM.");
}

// --- Smooth Transition to Potentiometer Positions ---
// Function to move servos smoothly to positions set by the knobs
void smoothTransitionToPot() {
  // Set starting angles to current servo positions
  for (int i = 0; i < 5; i++) {
    startAngles[i] = servoAngles[i];
    startAngles[i] = constrain(startAngles[i], 0, 180); // Keep angles between 0 and 180
    int val = analogRead(potPins[i]); // Read knob position
    targetAngles[i] = map(val, 0, 1023, 0, 180); // Convert knob value to angle
  }

  // Move servos smoothly over 2 seconds (100 steps of 20ms)
  for (int step = 0; step <= 100; step++) {
    float t = step / 100.0; // Progress from 0 to 1
    float easedT = 0.5 * (1 - cos(t * PI)); // Smooth movement curve instead of a linear line between two points
    for (int i = 0; i < 5; i++) {
      int angle = startAngles[i] + (targetAngles[i] - startAngles[i]) * easedT; // Calculate intermediate angle
      angle = constrain(angle, 0, 180); // Ensure valid angle
      servos[i].write(angle); // Move servo to angle
    }
    delay(20); // Wait 20ms per step
  }

  // Update current angles to final positions
  for (int i = 0; i < 5; i++) {
    servoAngles[i] = targetAngles[i];
    startAngles[i] = targetAngles[i];
    targetAngles[i] = 0; // Reset targets
  }
}

// --- Setup: Run Once at Start ---
// Initialize the robotic arm and move it smoothly to home then to knob positions
void setup() {
  Serial.begin(9600); // Start communication with computer for debugging

  // Connect servos to their pins
  for (int i = 0; i < 5; i++) {
    servos[i].attach(servoPins[i]);
  }
  pinMode(button1Pin, INPUT_PULLUP); // Set button1 pin with internal pull-up resistor
  pinMode(button2Pin, INPUT_PULLUP); // Set button2 pin with internal pull-up resistor
  pinMode(buzzerPin, OUTPUT); // Set buzzer pin for output

  // Load saved motions from memory
  loadMotionsFromEEPROM();

  // Read initial knob positions to set servo starting angles
  for (int i = 0; i < 5; i++) {
    int val = analogRead(potPins[i]); // Read knob
    servoAngles[i] = map(val, 0, 1023, 0, 180); // Convert to angle
    startAngles[i] = servoAngles[i]; // Store starting angle
    servos[i].write(servoAngles[i]); // Move servo to initial position
  }

  // Step 1: Smoothly move servos from initial positions to home (custom angles) over 2 seconds
  for (int step = 0; step <= 100; step++) {
    float t = step / 100.0; // Progress from 0 to 1
    float easedT = 0.5 * (1 - cos(t * PI)); // Smooth movement curve
    for (int i = 0; i < 5; i++) {
      int angle = startAngles[i] + (homeAngles[i] - startAngles[i]) * easedT; // Move to custom home angle
      angle = constrain(angle, 0, 180); // Ensure valid angle
      servos[i].write(angle);
    }
    delay(20); // Wait 20ms per step
  }

  // Update angles to home position
  for (int i = 0; i < 5; i++) {
    servoAngles[i] = homeAngles[i];
    startAngles[i] = homeAngles[i]; // Prepare for next movement
  }

  // Step 2: Move servos smoothly to current knob positions
  for (int i = 0; i < 5; i++) {
    int val = analogRead(potPins[i]);
    targetAngles[i] = map(val, 0, 1023, 0, 180); // Set target to knob position
  }

  // Smoothly move from home to knob positions over 2 seconds
  for (int step = 0; step <= 100; step++) {
    float t = step / 100.0;
    float easedT = 0.5 * (1 - cos(t * PI));
    for (int i = 0; i < 5; i++) {
      int angle = startAngles[i] + (targetAngles[i] - startAngles[i]) * easedT;
      angle = constrain(angle, 0, 180); // Ensure valid angle
      servos[i].write(angle);
    }
    delay(20);
  }

  // Update angles to final knob positions
  for (int i = 0; i < 5; i++) {
    servoAngles[i] = targetAngles[i];
    startAngles[i] = targetAngles[i];
    targetAngles[i] = 0; // Reset targets
  }
}

// --- Manual Control Mode ---
// Function to control servos using knobs in CONTROL_MODE
void updateControl() {
  if (transitioningToControl) { // If switching from PLAY_MODE
    unsigned long now = millis(); // Get current time
    float t = (float)(now - transitionStartTime) / transitionDuration; // Progress from 0 to 1
    t = constrain(t, 0.0, 1.0); // Keep between 0 and 1
    float easedT = 0.5 * (1 - cos(t * PI)); // Smooth movement curve

    // Move servos smoothly to knob positions
    for (int i = 0; i < 5; i++) {
      int val = analogRead(potPins[i]);
      targetAngles[i] = map(val, 0, 1023, 0, 180);
      int interpAngle = startAngles[i] + (targetAngles[i] - startAngles[i]) * easedT;
      interpAngle = constrain(interpAngle, 0, 180);
      servos[i].write(interpAngle);
      servoAngles[i] = interpAngle; // Update current angle
    }
    if (t >= 1.0) { // Transition complete
      transitioningToControl = false;
      for (int i = 0; i < 5; i++) {
        startAngles[i] = servoAngles[i];
        targetAngles[i] = 0;
      }
    }
  } else { // Normal manual control
    for (int i = 0; i < 5; i++) {
      int val = analogRead(potPins[i]); // Read knob
      servoAngles[i] = map(val, 0, 1023, 0, 180); // Convert to angle
      servos[i].write(servoAngles[i]); // Move servo
    }
  }
}

// --- Playback Mode ---
// Function to play recorded motions automatically in PLAY_MODE
void playMotion() {
  if (motionCount == 0 || isPaused) return; // Stop if no motions or paused
  unsigned long now = millis(); // Get current time

  if (inDelay) { // If in a pause between motions
    if (returningHome && (now - delayStartTime >= endDelay)) { // After 3-second delay
      // Move to home position
      for (int i = 0; i < 5; i++) {
        startAngles[i] = servoAngles[i];
        targetAngles[i] = homeAngles[i]; // Use custom home angles
      }
      returningHome = false;
      inDelay = false;
      transitioning = true;
      transitionStartTime = now;
      motionIndex = 0; // Reset to start after returning home
      Serial.println("Returned to home after 3-second delay");
    } else if (!returningHome && (now - delayStartTime >= motionDelay)) { // After 1-second delay
      inDelay = false; // End delay
      if (motionIndex < motionCount) { // Play next motion
        // Move to next motion
        for (int i = 0; i < 5; i++) {
          startAngles[i] = servoAngles[i];
          targetAngles[i] = motions[motionIndex][i];
        }
        transitioning = true;
        transitionStartTime = now;
        Serial.print("Playing motion "); Serial.println(motionIndex + 1);
      }
    }
    return; // Wait during delay
  }

  if (!transitioning) { // Start a new motion or delay
    if (motionIndex >= motionCount) { // All motions played
      returningHome = true;
      inDelay = true; // Start 3-second delay before returning home
      delayStartTime = now;
      Serial.println("Starting 3-second delay before returning to home");
    } else {
      inDelay = true; // Start 1-second delay before next motion
      delayStartTime = now;
    }
    return;
  }

  // Smoothly move servos to target angles
  float t = (float)(now - transitionStartTime) / transitionDuration;
  t = constrain(t, 0.0, 1.0);
  float easedT = 0.5 * (1 - cos(t * PI));

  for (int i = 0; i < 5; i++) {
    int interpAngle = startAngles[i] + (targetAngles[i] - startAngles[i]) * easedT;
    interpAngle = constrain(interpAngle, 0, 180);
    servos[i].write(interpAngle);
    servoAngles[i] = interpAngle; // Update current angle
  }
  if (t >= 1.0) { // Transition complete
    transitioning = false;
    for (int i = 0; i < 5; i++) {
      servoAngles[i] = targetAngles[i]; // Sync angles
    }
    inDelay = true; // Start delay after motion
    delayStartTime = now;
    motionIndex++; // Increment after motion completes
    if (motionIndex < motionCount) {
      Serial.println("HEYYY, this is after each motion");
      tone(buzzerPin, 1000); // Short beep
      delay(100);
      noTone(buzzerPin);
    }
  }
}

// --- Button Debouncing ---
// Function to handle button presses reliably, ignoring electrical noise
void handleButton(int pin, bool &lastReading, bool &state, unsigned long &lastChangeTime) {
  bool reading = digitalRead(pin); // Read button state
  if (reading != lastReading) lastChangeTime = millis(); // Record time of change
  if ((millis() - lastChangeTime) > debounceDelay && reading != state) state = reading; // Update state after delay
  lastReading = reading;
}

// --- Button 1 Logic (Save Motion or Switch Mode) ---
// Handle single press (save motion) or double press (switch mode)
void handleButton1Logic() {
  if (button1State == LOW) { // Button 1 pressed
    unsigned long now = millis();
    tone(buzzerPin, 1000); // Short beep
    delay(100);
    noTone(buzzerPin);
    if (now - lastButton1Press < doubleClickGap) { // Double press
      button1SinglePending = false;
      tone(buzzerPin, 900); // Longer beep for mode switch
      delay(300);
      noTone(buzzerPin);
      if (currentMode == CONTROL_MODE) { // Switch to PLAY_MODE
        currentMode = PLAY_MODE;
        transitioning = false;
        returningHome = false; // Start from first motion
        motionIndex = 0;
        inDelay = true; // Start with delay before first motion
        delayStartTime = millis();
        Serial.print("Switched to PLAY MODE. Total motions: ");
        Serial.println(motionCount);
      } else { // Switch to CONTROL_MODE
        currentMode = CONTROL_MODE;
        transitioning = false;
        isPaused = false;
        transitioningToControl = true; // Smooth transition
        transitionStartTime = millis();
        for (int i = 0; i < 5; i++) {
          startAngles[i] = servoAngles[i];
          int val = analogRead(potPins[i]);
          targetAngles[i] = map(val, 0, 1023, 0, 180);
        }
        Serial.println("Switched to CONTROL MODE");
      }
    } else {
      if (currentMode == CONTROL_MODE) { // Single press only in CONTROL_MODE
        button1SinglePending = true;
      }
    }
    lastButton1Press = now;
    delay(50); // Small delay to avoid multiple triggers
  }

  if (button1SinglePending && (millis() - lastButton1Press > doubleClickGap)) { // Single press confirmed
    if (currentMode == CONTROL_MODE && motionCount < MAX_MOTIONS) {
      for (int i = 0; i < 5; i++) motions[motionCount][i] = servoAngles[i]; // Save current angles
      motionCount++;
      saveMotionsToEEPROM();
      Serial.print("Motion stored. Total motions: ");
      Serial.println(motionCount);
    }
    button1SinglePending = false;
  }
}

// --- Button 2 Logic (Delete, Pause, or Clear) ---
// Handle single press (pause/resume) or double press (delete/clear)
void handleButton2Logic() {
  if (button2State == LOW) { // Button 2 pressed
    unsigned long now = millis();
    tone(buzzerPin, 550); // Short beep
    delay(100);
    noTone(buzzerPin);
    if (now - lastButton2Press < doubleClickGap) { // Double press
      button2SinglePending = false;
      tone(buzzerPin, 900); // Longer beep
      delay(300);
      noTone(buzzerPin);
      if (currentMode == PLAY_MODE) { // Clear all motions
        motionCount = 0;
        motionIndex = 0;
        saveMotionsToEEPROM();
        Serial.println("All motions cleared.");
      } else if (motionCount > 0) { // Delete last motion
        motionCount--;
        saveMotionsToEEPROM();
        Serial.println("Last motion removed.");
      }
    } else {
      button2SinglePending = true;
    }
    lastButton2Press = now;
    delay(50);
  }

  if (button2SinglePending && (millis() - lastButton2Press > doubleClickGap)) { // Single press confirmed
    if (currentMode == PLAY_MODE) {
      isPaused = !isPaused; // Toggle pause
      Serial.println(isPaused ? "Paused playback." : "Resumed playback.");
      if (!isPaused) { // Resume playback
        if (motionIndex < motionCount) {
          for (int i = 0; i < 5; i++) {
            startAngles[i] = servoAngles[i];
            targetAngles[i] = motions[motionIndex][i];
          }
          transitioning = true;
          inDelay = false; // Clear delay
          transitionStartTime = millis();
        } else {
          // If paused at home, restart from first motion
          motionIndex = 0;
          inDelay = true;
          delayStartTime = millis();
        }
      } else { // Pause playback
        transitioning = false;
        inDelay = false;
      }
    }
    button2SinglePending = false;
  }
}

// --- Main Loop: Runs Continuously ---
// Check buttons and run appropriate mode
void loop() {
  // Store previous button states to detect new presses
  bool prevButton1State = button1State;
  bool prevButton2State = button2State;

  // Update button states
  handleButton(button1Pin, lastButton1Reading, button1State, lastButton1Change);
  handleButton(button2Pin, lastButton2Reading, button2State, lastButton2Change);

  // Check for new button presses
  bool button1JustPressed = (prevButton1State == HIGH && button1State == LOW);
  bool button2JustPressed = (prevButton2State == HIGH && button2State == LOW);

  // Handle button presses
  if (button1JustPressed) handleButton1Logic();
  if (button2JustPressed) handleButton2Logic();

  // Check for single press actions after double-click window
  if (button1SinglePending && (millis() - lastButton1Press > doubleClickGap)) {
    if (currentMode == CONTROL_MODE && motionCount < MAX_MOTIONS) {
      for (int i = 0; i < 5; i++) motions[motionCount][i] = servoAngles[i];
      motionCount++;
      saveMotionsToEEPROM();
      Serial.print("Motion stored. Total motions: ");
      Serial.println(motionCount);
    }
    button1SinglePending = false;
  }

  if (button2SinglePending && (millis() - lastButton2Press > doubleClickGap)) {
    if (currentMode == PLAY_MODE) {
      isPaused = !isPaused;
      Serial.println(isPaused ? "Paused playback." : "Resumed playback.");
      if (!isPaused) {
        if (motionIndex < motionCount) {
          for (int i = 0; i < 5; i++) {
            startAngles[i] = servoAngles[i];
            targetAngles[i] = motions[motionIndex][i];
          }
          transitioning = true;
          inDelay = false;
          transitionStartTime = millis();
        } else {
          // If paused at home, restart from first motion
          motionIndex = 0;
          inDelay = true;
          delayStartTime = millis();
        }
      } else {
        transitioning = false;
        inDelay = false;
      }
    }
    button2SinglePending = false;
  }

  // Run the appropriate mode
  if (currentMode == CONTROL_MODE) updateControl();
  else playMotion();
}
