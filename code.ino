#include <Wire.h>
#include <WiFi.h>
#include <math.h>

// I2C Address for BMI160

// Change to 0x68 if the SDO pin is tied to GND

bool start = false;
bool prevstate = false;

float angle_x = 0;
float angle_y = 0;
float angle_z = 0;
float D = 0;
float ax_g;
float ay_g;
float az_g;
float S = 0.0;
float u = 0.0;
float t = 0;
unsigned long int tl = 0;
unsigned long int tc = 0;
float a = 0.0;
float v = 0.0;

const int BMI160_ADDR = 0x69;

const char *ssid = "Modulas";

const char *pass = "12345678";

NetworkServer server(80);

int16_t ax, ay, az;
int16_t gx, gy, gz;  //do S=ut+(at^2)/2  , then get direction facing during the calc of u

String webread = "";

// BMI160 Registers
const uint8_t REG_CHIP_ID = 0x00;
const uint8_t REG_GYRO_DATA = 0x0C;   // Gyroscope X LSB
const uint8_t REG_ACCEL_DATA = 0x12;  // Accelerometer X LSB
const uint8_t REG_CMD = 0x7E;         // Command Register

// Power Mode Commands
const uint8_t CMD_ACC_NORMAL = 0x11;  // Bring Accel to normal mode
const uint8_t CMD_GYR_NORMAL = 0x15;  // Bring Gyro to normal mode

const uint8_t ACC_RANGE_REG = 0x41;
const uint8_t RANGE = 0x05;

void setup() {

  Serial.begin(115200);
  Wire.begin(21,47);

  activateBMI160_ACCEL_GYRO();

  WiFi.softAP(ssid, pass);
  server.begin();
}



void readGyro() {

  // Read 6 bytes of Gyroscope data starting at REG_GYRO_DATA

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_GYRO_DATA);
  Wire.endTransmission(false);
  Wire.requestFrom(BMI160_ADDR, 6);

  if (Wire.available() == 6) {

    gx = Wire.read() | (Wire.read() << 8);

    gy = Wire.read() | (Wire.read() << 8);

    gz = Wire.read() | (Wire.read() << 8);
  }
}



void readAccel() {

  // Read 6 bytes of Accelerometer data starting at REG_ACCEL_DATA

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_ACCEL_DATA);
  Wire.endTransmission(false);  // Restart condition for continuous read
  Wire.requestFrom(BMI160_ADDR, 6);

  if (Wire.available() == 6) {

    // Data is sent LSB first, then MSB

    ax = Wire.read() | (Wire.read() << 8);

    ay = Wire.read() | (Wire.read() << 8);

    az = Wire.read() | (Wire.read() << 8);
  }
}



void activateBMI160_ACCEL_GYRO() {

  // 1. Check the Chip ID (BMI160 should return 0xD1)

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_CHIP_ID);
  Wire.endTransmission();
  Wire.requestFrom(BMI160_ADDR, 1);

  if (Wire.available()) {

    uint8_t chipID = Wire.read();
    Serial.print("BMI160 Chip ID: 0x");
    Serial.println(chipID, HEX);

  } else {

    Serial.println("BMI160 not found! Check wiring and I2C address.");

    while (1)
      delay(10);
    ;  // Halt if not found
  }

  // 2. Power up the Accelerometer

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_CMD);
  Wire.write(CMD_ACC_NORMAL);
  Wire.endTransmission();
  delay(100);  // Allow time to start up

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(ACC_RANGE_REG);  // Target the ACC_RANGE register
  Wire.write(RANGE);          // Set the range to 4g
  Wire.endTransmission();
  delay(100);
  // 3. Power up the Gyroscope

  Wire.beginTransmission(BMI160_ADDR);
  Wire.write(REG_CMD);
  Wire.write(CMD_GYR_NORMAL);
  Wire.endTransmission();
  delay(100);  // Allow time to start up
}

void SIunit() {
  ax_g = ((float)ax / 8192.0) * 9.81;
  ay_g = ((float)ay / 8192.0) * 9.81;
  az_g = ((float)az / 8192.0) * 9.81;

  ax_g = ax_g - (9.81 * sin(angle_x * (PI / 180.0)));

  ay_g = ay_g - (9.81 * sin(angle_y * (PI / 180.0)));

  az_g = az_g - (9.81 * sin(angle_z * (PI / 180.0)));
}

void calc() {
  a = sqrt((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));
  float t2 = t * t;
  float at = (a * t2) / 2;
  v = u + (a * t);
  float Sp = S;
  S = (u * t) + at;
  u = v;
  if(S == 10 && S == Sp){
   S -= 10;
  }
  if(S == 9 && S == Sp){
   S -= 9;
  }
  float Dp = D;
  D += S;
  if (S > 0.1) {
    D = Dp;
    S = Sp;
  }
  Serial.println("v: ");
  Serial.println(v);
  Serial.println("a: ");
  Serial.println(a);
  Serial.println("m/s: ");
  Serial.println(ax_g);
  Serial.println(ay_g);
  Serial.println(az_g);
  Serial.println("t2: ");
  Serial.println(t2);
  Serial.println(t);
  Serial.println("D: ");
  Serial.print(D);
  Serial.println("S: ");
  Serial.print(S);
  Serial.println("////////////////////////////////////////////////////////////////////////////////////////");
}

void gyrocalib() {
  if (gx > 26 || gx < -26 || gy > 26 || gy < -26 || gz > 26 || gz < -26) {
    angle_x += ((float)gx / 16.4) - 13 * t;
    angle_y += ((float)gy / 16.4) - 13 * t;
    angle_z += ((float)gz / 16.4) - 13 * t;

    angle_x = fmod(angle_x, 360);
    angle_y = fmod(angle_y, 360);
    angle_z = fmod(angle_z, 360);

    if (angle_x < 0) {
      angle_x += 360;
    }
    if (angle_y < 0) {
      angle_y += 360;
    }
    if (angle_z < 0) {
      angle_z += 360;
    }
  }
  Serial.println("angles");
  Serial.println(angle_x);
  Serial.println(angle_y);
  Serial.println(angle_z);
}

void loop() {
  tc = millis();
  t = (float)(tc - tl) / 1000.0;
  tl = tc;

  NetworkClient client = server.accept();

  if (client) {

    while (client.connected()) {
      if (client.available()) {

        char c = client.read();

        if (c == '\n') {

          if (webread.length() != 0) {
            webread = "";
            break;
          }

        } else if (c != '\r') {
          webread += c;
        }
      }
    }
  }
  readAccel();
  readGyro();
  gyrocalib();
  SIunit();
  // Print the raw values to the Serial Monitor

  Serial.print("Accel (x,y,z): ");
  Serial.print(ax_g);
  Serial.print(", ");
  Serial.print(ay_g);
  Serial.print(", ");
  Serial.print(az_g);
  Serial.print(" | ");

  Serial.print("Gyro (x,y,z): ");
  Serial.print(gx);
  Serial.print(", ");
  Serial.print(gy);
  Serial.print(", ");
  Serial.println(gz);

  if (ax_g > 10 || ax_g < -10 || az_g > 10 || az_g < -10 || ay_g > 10 || ay_g < -10) {
    calc();
  } else {
    v = 0.0;
    u = 0.0;
  }

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: text/html");
  client.println("Refresh: 0.2");
  client.println();

  client.println("<!DOCTYPE html>");
  client.println("<html>");
  client.println("<head>");
  client.println("<meta charset=\"UTF-8\">");
  client.println("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\">");
  client.println("<title>Physics Block</title>");
  client.println("<style>");
  client.println("body { font-family: monospace; background: #0f172a; color: #38bdf8; text-align: center; margin-top: 40px; }");
  client.println(".card { background: #1e293b; padding: 20px; margin: 15px auto; width: 80%; max-width: 320px; border-radius: 12px; box-shadow: 0 4px 6px rgba(0,0,0,0.4); }");
  client.println("h1 { color: #f8fafc; font-size: 26px; }");
  client.println("h3 { color: #cbd5e1; border-bottom: 1px solid #334155; padding-bottom: 8px; }");
  client.println(".val { color: #4ade80; font-weight: bold; }");
  client.println("</style>");
  client.println("<script>");
  // Auto-updates the dashboard every 1 second via JSON fetch
  client.println("setInterval(() => {");
  client.println("  fetch('/data').then(res => res.json()).then(data => {");
  client.println("    document.getElementById('ax').innerText = data.content_accel.ax;");
  client.println("    document.getElementById('ay').innerText = data.content_accel.ay;");
  client.println("    document.getElementById('az').innerText = data.content_accel.az;");
  client.println("    document.getElementById('gx').innerText = data.content_gyro.gx;");
  client.println("    document.getElementById('gy').innerText = data.content_gyro.gy;");
  client.println("    document.getElementById('gz').innerText = data.content_gyro.gz;");
  client.println("    document.getElementById('v').innerText = data.kinematics.velocity;");
  client.println("    document.getElementById('a').innerText = data.kinematics.scalar_acceleration;");
  client.println("    document.getElementById('D').innerText = data.kinematics.distance;");
  client.println("    document.getElementById('S').innerText = data.kinematics.displacement;");
  client.println("  }).catch(err => console.log(err));");
  client.println("}, 1000);");
  client.println("</script>");
  client.println("</head>");
  client.println("<body>");

  client.println("<h1>Physics Block Dashboard</h1>");

  // Accelerometer Card
  client.println("<div class=\"card\">");
  client.println("<h3>Accelerometer</h3>");
  client.print("<p>AX: <span id=\"ax\" class=\"val\">");
  client.print(ax_g);
  client.println("</span></p>");
  client.print("<p>AY: <span id=\"ay\" class=\"val\">");
  client.print(ay_g);
  client.println("</span></p>");
  client.print("<p>AZ: <span id=\"az\" class=\"val\">");
  client.print(az_g);
  client.println("</span></p>");
  client.println("</div>");

  // Gyroscope Card
  client.println("<div class=\"card\">");
  client.println("<h3>Gyroscope</h3>");
  client.print("<p>GX: <span id=\"gx\" class=\"val\">");
  client.print(angle_x);
  client.println("</span></p>");
  client.print("<p>GY: <span id=\"gy\" class=\"val\">");
  client.print(angle_y);
  client.println("</span></p>");
  client.print("<p>GZ: <span id=\"gz\" class=\"val\">");
  client.print(angle_z);
  client.println("</span></p>");
  client.println("</div>");

  // Kinematics Card
  client.println("<div class=\"card\">");
  client.println("<h3>Kinematics</h3>");
  client.print("<p>Velocity (v): <span id=\"vel\" class=\"val\">");
  client.print(v);
  client.println("</span></p>");
  client.print("<p>Scalar Accel (a): <span id=\"acc\" class=\"val\">");
  client.print(a);
  client.println("</span></p>");
  client.print("<p>Distance (D): <span id=\"dis\" class=\"val\">");
  client.print(D);
  client.println("</span></p>");
  client.print("<p>Displacement (S): <span id=\"dpl\" class=\"val\">");
  client.print(S);
  client.println("</span></p>");
  client.println("</div>");

  client.println("</body>");
  client.println("</html>");
  client.println();

  delay(30);
}
