#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP5xx.h>

// BMP585 Debug version - checks sensor communication step by step

Adafruit_BMP5xx bmp;

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n=== BMP585 Debug Test ===\n");

  // Step 1: Initialize I2C
  Serial.println("Step 1: Initializing I2C...");
  Wire.begin(21, 22);
  Wire.setClock(400000);
  Serial.println("  ✓ I2C initialized (GPIO 21=SDA, GPIO 22=SCL, 400kHz)\n");

  // Step 2: Try to initialize sensor
  Serial.println("Step 2: Attempting BMP585 initialization...");

  bool initialized = false;

  // Try primary address (0x47 - jumper to VCC)
  Serial.println("  Trying address 0x47 (jumper to VCC)...");
  if (bmp.begin(0x47, &Wire))
  {
    Serial.println("    ✓ Success at 0x47!\n");
    initialized = true;
  }
  else
  {
    Serial.println("    ✗ Not at 0x47\n");

    // Try secondary address (0x46 - jumper to GND)
    Serial.println("  Trying address 0x46 (jumper to GND)...");
    if (bmp.begin(0x46, &Wire))
    {
      Serial.println("    ✓ Success at 0x46!\n");
      initialized = true;
    }
    else
    {
      Serial.println("    ✗ Not at 0x46\n");
    }
  }

  if (!initialized)
  {
    Serial.println("✗ FAILED to initialize BMP585!");
    Serial.println("  Troubleshooting:");
    Serial.println("  1. Check power supply to sensor (3.3V)");
    Serial.println("  2. Check I2C pull-up resistors (usually 10k on SDA/SCL)");
    Serial.println("  3. Verify correct address jumper setting");
    Serial.println("  4. Run I2C scanner sketch to find devices on bus");
    Serial.println("  5. Test with multimeter: SDA/SCL should have ~1.65V (3.3V/2)\n");
    return;
  }

  // Step 3: Configure sensor
  Serial.println("Step 3: Configuring sensor...");
  bmp.setTemperatureOversampling(BMP5XX_OVERSAMPLING_2X);
  bmp.setPressureOversampling(BMP5XX_OVERSAMPLING_16X);
  bmp.setIIRFilterCoeff(BMP5XX_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP5XX_ODR_50_HZ);
  Serial.println("  ✓ Configuration complete\n");

  // Step 4: Test first reading
  Serial.println("Step 4: Attempting first sensor read...");
  delay(100);

  if (bmp.performReading())
  {
    Serial.println("  ✓ Read successful!\n");

    Serial.println("Raw Sensor Values:");
    Serial.printf("  Temperature (raw): %.2f °C\n", bmp.temperature);
    Serial.printf("  Pressure (raw):    %f Pa\n", bmp.pressure);
    Serial.printf("  Pressure (hPa):    %.2f hPa\n", bmp.pressure / 100.0f);
    Serial.printf("  Altitude (raw):    %.2f m\n\n", bmp.readAltitude(1013.25f));

    // Check if values look reasonable
    if (bmp.temperature > 50 || bmp.temperature < -40)
    {
      Serial.println("⚠ WARNING: Temperature out of operating range (-40 to +85°C)");
      Serial.println("  This might indicate:");
      Serial.println("  - Wrong I2C address (reading from different device)");
      Serial.println("  - Data corruption on I2C bus");
      Serial.println("  - Sensor malfunction\n");
    }

    if (bmp.pressure < 30000 || bmp.pressure > 110000)
    {
      Serial.println("⚠ WARNING: Pressure out of operating range (300 to 1100 hPa)");
      Serial.println("  This might indicate:");
      Serial.println("  - Wrong I2C address (reading from different device)");
      Serial.println("  - Sensor not initialized properly");
      Serial.println("  - Data corruption on I2C bus\n");
    }
  }
  else
  {
    Serial.println("  ✗ Read failed!");
    Serial.println("  Sensor initialized but won't respond to read command\n");
  }

  Serial.println("=== Debug test complete ===\n");
  Serial.println("All subsequent reads will be printed below:\n");
}

void loop()
{
  delay(500);

  if (bmp.performReading())
  {
    Serial.printf("T: %.2f°C | P: %.2f hPa | Alt: %.2f m\n",
                  bmp.temperature,
                  bmp.pressure / 100.0f,
                  bmp.readAltitude(1013.25f));
  }
  else
  {
    Serial.println("Read failed");
  }
}
