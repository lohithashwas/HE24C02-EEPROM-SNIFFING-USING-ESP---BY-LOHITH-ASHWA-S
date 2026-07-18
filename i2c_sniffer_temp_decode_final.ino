/*	DONE BY : LOHITH ASHWA S
	ON 18.07.2026, SATURDAY 

	LINKEDIN : https://www.linkedin.com/in/lohith-ashwa-s-480842277/
	EMAIL ID : lohithashwa51@gmail.com
	Portfolio : lohithashwa.me



  ESP32 I2C Sniffer + Temperature Decoder
  ----------------------------------------
  Same passive bus tap as before, but now every FRAME's raw 16-bit value
  (bytes[1] and bytes[2], little-endian) is converted to an estimated
  Fahrenheit temperature and printed next to the raw bytes.

  KNOWN LIMITATION (confirmed from real data 2024-xx-xx):
  This uses the simple formula  degC = raw/100 ; degF = degC*9/5+32
  This formula MATCHES the real display for normal-range readings
  (~96-98F), but UNDERSHOOTS high readings (it computed 106.9F for a
  reading that actually displayed 109.2F -- a 2.3F gap). This means the
  real device applies some extra compensation at higher temps that this
  code does NOT yet reproduce. Treat any computed value above ~100F as
  an underestimate until we figure out the correction.

  Also flags the 0x7Fxx error/out-of-range sentinel seen in your capture
  (shows as "ERR" instead of a temperature - this is what produced "HI"
  on the display).

  BUTTON TRIGGER (new)
  ---------------------
  Wire one ESP32 GPIO (PIN_BUTTON below) directly onto ONE of the two
  solder pads of the manual push button on the thermometer PCB. Leave the
  other pad alone (it stays wired to the thermometer's own circuit as-is).

  - Do NOT connect PIN_BUTTON to the thermometer's ground pad directly with
    a permanent wire - it must go to the button pad itself.
  - The pin sits in INPUT (high-impedance) mode by default, so it does not
    affect the button at all when idle - the physical button still works
    normally.
  - When you type 'S' (or 's') into the Serial Monitor and hit Enter, the
    firmware switches that pin to OUTPUT, drives it LOW for ~150ms (shorting
    the button contacts, same as a real press), then switches back to INPUT.
  - Requires ESP32 GND common with the thermometer's GND (you already have
    this from the I2C sniffing setup).

  WIFI CONTROL (new)
  -------------------
  The ESP32 connects to your WiFi and hosts a small web page. Open the
  printed IP address in any browser on the same network to see the last
  temperature and a "Take Reading" button (does the same thing as typing
  'S' in Serial Monitor, but from your phone/PC over WiFi).

  NOTE: WiFi credentials are hardcoded below for convenience since this
  runs on your own home network and controls your own hardware. Don't
  share this .ino file publicly with your password still in it.
*/

#include <WiFi.h>
#include <WebServer.h>

const char* WIFI_SSID     = "********";
const char* WIFI_PASSWORD = "********";

WebServer server(80);
String lastTemperature = "no reading yet";

#define PIN_BUTTON 4     // pick any free GPIO, wire to one button pad
#define PRESS_MS   150   // how long to "hold" the simulated press

#define PIN_SDA 21
#define PIN_SCL 22
#define FILTER_ADDR -1   // set to a specific address to filter, -1 = log all

volatile uint8_t  bitBuf     = 0;
volatile uint8_t  bitCount   = 0;
volatile bool     frameActive = false;

#define BUF_SIZE 512
struct Ev { uint8_t data; bool ack; bool isStart; bool isStop; };
volatile Ev evBuf[BUF_SIZE];
volatile uint16_t evHead = 0, evTail = 0;

void pushEvent(uint8_t data, bool ack, bool isStart, bool isStop) {
  uint16_t next = (evHead + 1) % BUF_SIZE;
  if (next == evTail) return;
  evBuf[evHead].data = data;
  evBuf[evHead].ack = ack;
  evBuf[evHead].isStart = isStart;
  evBuf[evHead].isStop = isStop;
  evHead = next;
}

void IRAM_ATTR onSCLRise() {
  if (!frameActive) return;
  int bit = digitalRead(PIN_SDA);
  bitBuf = (bitBuf << 1) | bit;
  bitCount++;
  if (bitCount == 9) {
    bool ack = (bitBuf & 0x01) == 0;
    uint8_t byteVal = (bitBuf >> 1) & 0xFF;
    pushEvent(byteVal, ack, false, false);
    bitCount = 0;
    bitBuf = 0;
  }
}

void IRAM_ATTR onSDAChange() {
  if (digitalRead(PIN_SCL) != HIGH) return;
  int sda = digitalRead(PIN_SDA);
  if (sda == LOW) {
    frameActive = true;
    bitCount = 0;
    bitBuf = 0;
    pushEvent(0, false, true, false);
  } else {
    frameActive = false;
    pushEvent(0, false, false, true);
  }
}

void triggerButtonPress();

void handleRoot() {
  String html = "<html><head><title>Thermometer Control</title>"
    "<meta name='viewport' content='width=device-width, initial-scale=1'>"
    "<style>body{font-family:sans-serif;text-align:center;margin-top:60px;}"
    "button{font-size:24px;padding:20px 40px;border-radius:10px;}"
    "#temp{font-size:48px;margin:30px;}</style>"
    "<script>"
    "function trigger(){fetch('/trigger').then(()=>setTimeout(refresh,2000));}"
    "function refresh(){fetch('/status').then(r=>r.text()).then(t=>{"
    "document.getElementById('temp').innerText=t;});}"
    "setInterval(refresh,2000);"
    "</script></head><body>"
    "<h2>Thermometer Control</h2>"
    "<div id='temp'>...</div>"
    "<button onclick='trigger()'>Take Reading</button>"
    "</body></html>";
  server.send(200, "text/html", html);
}

void handleTrigger() {
  triggerButtonPress();
  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  server.send(200, "text/plain", lastTemperature);
}

void setup() {
  Serial.begin(921600);
  delay(300);
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  pinMode(PIN_BUTTON, INPUT); // high-Z, doesn't affect the real button
  attachInterrupt(digitalPinToInterrupt(PIN_SCL), onSCLRise, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_SDA), onSDAChange, CHANGE);
  Serial.println();
  Serial.println("Ready. Take a reading...");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! Open this in your browser: http://");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/trigger", handleTrigger);
  server.on("/status", handleStatus);
  server.begin();
}

bool inFrame = false;
bool gotAddr = false;
uint8_t addr7 = 0;
bool isRead = false;
bool addrMatches = true;
uint8_t byteIndex = 0;
uint8_t frameBytes[8];
uint8_t frameByteCount = 0;

void printTempIfApplicable() {
  // A "write to register 0x26 with 2 data bytes" frame is exactly the
  // pattern that carries the raw temperature: byte[0]=0x26, byte[1..2]=raw LE
  if (frameByteCount >= 3 && frameBytes[0] == 0x26) {
    uint16_t raw = frameBytes[1] | (frameBytes[2] << 8);

    if ((raw & 0xFF00) == 0x7F00) {
      Serial.println("TEMPERATURE : HI (error)");
      lastTemperature = "HI (error)";
      return;
    }

    float degC = raw / 100.0;
    float degF = degC * 9.0 / 5.0 + 32.0;
    Serial.print("TEMPERATURE : ");
    Serial.print(degF, 1);
    Serial.println(" F");
    lastTemperature = String(degF, 1) + " F";
  }
}

void triggerButtonPress() {
  Serial.println("[button] simulated press");
  pinMode(PIN_BUTTON, OUTPUT);
  digitalWrite(PIN_BUTTON, LOW);
  delay(PRESS_MS);
  pinMode(PIN_BUTTON, INPUT); // release back to high-Z
}

void loop() {
  server.handleClient();

  if (Serial.available()) {
    char c = Serial.read();
    if (c == 'S' || c == 's') {
      triggerButtonPress();
    }
  }

  while (evTail != evHead) {
    Ev e;
    noInterrupts();
    e.data = evBuf[evTail].data;
    e.ack = evBuf[evTail].ack;
    e.isStart = evBuf[evTail].isStart;
    e.isStop = evBuf[evTail].isStop;
    evTail = (evTail + 1) % BUF_SIZE;
    interrupts();

    if (e.isStart) {
      inFrame = true;
      gotAddr = false;
      byteIndex = 0;
      frameByteCount = 0;
      continue;
    }
    if (e.isStop) {
      if (inFrame && addrMatches) {
        printTempIfApplicable();
      }
      inFrame = false;
      continue;
    }
    if (!inFrame) continue;

    if (!gotAddr) {
      addr7 = e.data >> 1;
      isRead = (e.data & 0x01);
      addrMatches = (FILTER_ADDR < 0) || (addr7 == FILTER_ADDR);
      gotAddr = true;
      continue;
    }

    if (addrMatches) {
      if (frameByteCount < 8) frameBytes[frameByteCount++] = e.data;
      byteIndex++;
    }
  }
}
