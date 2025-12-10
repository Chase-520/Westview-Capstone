#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // Reset pin (-1 if not used)

// Using your pins: SDA=21, SCL=22
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);
  
  // Initialize OLED with I2C address 0x3C
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  Serial.println("OLED initialized!");
  
  // Clear the buffer
  display.clearDisplay();
  
  // Display text
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
  display.println(F("Hello World!"));
  display.println(F("OLED Test"));
  display.println(F("SDA: GPIO21"));
  display.println(F("SCL: GPIO22"));
  display.display();           // Actually display the content
  
  delay(2000); // Pause for 2 seconds
}

void loop() {
  // Clear and update display with time
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print(F("Time: "));
  display.println(millis() / 1000);
  display.display();
  
  delay(1000); // Update every second
}
