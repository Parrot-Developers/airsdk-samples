#include <Arduino.h>

// Support for 'Lolin32 Lite' and 'ESP32-C6-Zero' boards
// Automatically detect the best LED pin for the selected board
#if defined(RGB_BUILTIN)
  #define LED_PIN RGB_BUILTIN
#elif defined(LED_BUILTIN)
  #define LED_PIN LED_BUILTIN
#else
  #define LED_PIN 22 // Default fallback for Lolin32 Lite
#endif

// Morse code timings (in ms)
#define DOT_DURATION 200
#define DASH_DURATION (DOT_DURATION * 3)
#define SYMBOL_SPACE DOT_DURATION
#define LETTER_SPACE (DOT_DURATION * 3)
#define WORD_SPACE (DOT_DURATION * 7)

// Morse code dictionary for A-Z and 0-9
const char* const morse_alphabet[] = {
  ".-",   // A
  "-...", // B
  "-.-.", // C
  "-..",  // D
  ".",    // E
  "..-.", // F
  "--.",  // G
  "....", // H
  "..",   // I
  ".---", // J
  "-.-",  // K
  ".-..", // L
  "--",   // M
  "-.",   // N
  "---",  // O
  ".--.", // P
  "--.-", // Q
  ".-.",  // R
  "...",  // S
  "-",    // T
  "..-",  // U
  "...-", // V
  ".--",  // W
  "-..-", // X
  "-.--", // Y
  "--.."  // Z
};

const char* const morse_numbers[] = {
  "-----", // 0
  ".----", // 1
  "..---", // 2
  "...--", // 3
  "....-", // 4
  ".....", // 5
  "-....", // 6
  "--...", // 7
  "---..", // 8
  "----."  // 9
};

void blink_symbol(bool is_dash) {
  digitalWrite(LED_PIN, HIGH);
  delay(is_dash ? DASH_DURATION : DOT_DURATION);
  digitalWrite(LED_PIN, LOW);
  delay(SYMBOL_SPACE);
}

void blink_string(const char* symbols) {
  for (int i = 0; symbols[i] != '\0'; i++) {
    blink_symbol(symbols[i] == '-');
  }
}

void blink_letter(char c) {
  c = toupper(c);

  if (c >= 'A' && c <= 'Z') {
    blink_string(morse_alphabet[c - 'A']);
    delay(LETTER_SPACE - SYMBOL_SPACE);
  }
  else if (c >= '0' && c <= '9') {
    blink_string(morse_numbers[c - '0']);
    delay(LETTER_SPACE - SYMBOL_SPACE);
  }
  else if (c == ' ') {
    delay(WORD_SPACE - LETTER_SPACE);
  }
}

void setup() {
  Serial.begin(115200);
#if !defined(RGB_BUILTIN)
  pinMode(LED_PIN, OUTPUT);
#endif
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      Serial.print("ACK: Blinking ");
      Serial.println(cmd);

      for (int i = 0; i < cmd.length(); i++) {
        blink_letter(cmd[i]);
      }
      Serial.println("DONE");
    }
  }
}
