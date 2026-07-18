/*	DONE BY : LOHITH ASHWA S
	ON 18.07.2026, SATURDAY 

	LINKEDIN : https://www.linkedin.com/in/lohith-ashwa-s-480842277/
	EMAIL ID : lohithashwa51@gmail.com
	Portfolio : lohithashwa.me


  ESP32 Passive I2C Bus Sniffer
  ------------------------------
  Purpose: Listen (do NOT drive) the SDA/SCL lines running between the
  IR temperature sensor chip and the thermometer's main MCU, so you can
  capture raw sensor register reads and correlate them with the displayed
  temperature.

  THIS IS A LISTENER, NOT A MASTER OR SLAVE.
  - Do NOT call Wire.begin() anywhere in this sketch.
  - The two GPIO pins below are configured as plain INPUT (not INPUT_PULLUP
    unless the bus itself has no pull-ups already - most sensor boards do).
  - Tap SDA/SCL with jumper wires in parallel; do not cut the existing traces.

  WIRING
  ------
  - If the sensor bus runs at 3.3V (many small breakout boards, e.g. GY-906 /
    MLX90614 modules): connect directly.
  - If the bus runs at 5V: use a logic-level shifter (or a simple 2-resistor
    divider) on BOTH lines before they reach the ESP32 GPIOs. Do not feed 5V
    into ESP32 pins directly.
  - Common ESP32 GND to the thermometer board GND is required.

  PIN_SDA  -> sensor bus SDA (tap point, between sensor chip and main MCU)
  PIN_SCL  -> sensor bus SCL

  USAGE
  -----
  1. Flash this to the ESP32.
  2. Open Serial Monitor at 921600 baud (fast, so we don't drop bytes while
     buffering/printing).
  3. Take a temperature reading on the thermometer while watching the log.
  4. Each printed FRAME line is one I2C transaction: address + R/W bit,
     then each data byte with its ACK/NACK.
  5. Match the FRAME(s) that occur right as a reading is taken to the
     displayed value, same way you built the EEPROM table.

  NOTES
  -----
  - This works reliably at standard-mode I2C (100 kHz). Many cheap sensor
    boards run at 100 kHz. If frames look corrupted/truncated, the bus may
    be running fast-mode (400 kHz) - see FAST_MODE_HINT below.
  - If you don't know the sensor's address yet, leave FILTER_ADDR as -1 to
    log everything on the bus (there may be multiple devices).
*/

#define PIN_SDA 21
#define PIN_SCL 22

// Set to a specific 7-bit address (e.g. 0x5A for MLX90614) to only print
// frames for that device, or -1 to log all traffic.
#define FILTER_ADDR -1

// If true, print raw bit-level transitions for debugging when frames look
// garbled (useful for figuring out if the bus is running faster than this
// sketch can reliably sample).
#define DEBUG_RAW false

volatile uint8_t  bitBuf     = 0;
volatile uint8_t  bitCount   = 0;
volatile bool     frameActive = false;
volatile bool     firstByte   = true;

// Small ring buffer of decoded bytes + ack flags, drained in loop()
#define BUF_SIZE 512
struct Ev { uint8_t data; bool ack; bool isStart; bool isStop; };
volatile Ev evBuf[BUF_SIZE];
volatile uint16_t evHead = 0, evTail = 0;

void pushEvent(uint8_t data, bool ack, bool isStart, bool isStop) {
  uint16_t next = (evHead + 1) % BUF_SIZE;
  if (next == evTail) return; // buffer full, drop (shouldn't happen at 100kHz)
  evBuf[evHead].data = data;
  evBuf[evHead].ack = ack;
  evBuf[evHead].isStart = isStart;
  evBuf[evHead].isStop = isStop;
  evHead = next;
}

// Called on every SCL rising edge while a frame is active: sample SDA
void IRAM_ATTR onSCLRise() {
  if (!frameActive) return;
  int bit = digitalRead(PIN_SDA);
  bitBuf = (bitBuf << 1) | bit;
  bitCount++;
  if (bitCount == 9) {
    bool ack = (bitBuf & 0x01) == 0; // ACK is SDA low
    uint8_t byteVal = (bitBuf >> 1) & 0xFF;
    pushEvent(byteVal, ack, false, false);
    bitCount = 0;
    bitBuf = 0;
  }
}

// Called on every SDA change: only meaningful while SCL is high
// (START = SDA falls while SCL high, STOP = SDA rises while SCL high)
void IRAM_ATTR onSDAChange() {
  if (digitalRead(PIN_SCL) != HIGH) return; // mid-bit data change, ignore
  int sda = digitalRead(PIN_SDA);
  if (sda == LOW) {
    // START (or repeated START)
    frameActive = true;
    bitCount = 0;
    bitBuf = 0;
    pushEvent(0, false, true, false);
  } else {
    // STOP
    frameActive = false;
    pushEvent(0, false, false, true);
  }
}

void setup() {
  Serial.begin(921600);
  delay(300);
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_SCL), onSCLRise, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_SDA), onSDAChange, CHANGE);
  Serial.println();
  Serial.println("=== ESP32 Passive I2C Sniffer ready ===");
  Serial.println("Take a reading on the thermometer now...");
}

// Decode state kept in loop() (not ISR) so we can safely print/format
bool inFrame = false;
bool gotAddr = false;
uint8_t addr7 = 0;
bool isRead = false;
bool addrMatches = true;
uint8_t byteIndex = 0;

void printFrameHeader() {
  Serial.print("=== FRAME ");
  Serial.print(isRead ? "READ " : "WRITE ");
  Serial.print("addr=0x");
  if (addr7 < 16) Serial.print("0");
  Serial.print(addr7, HEX);
  Serial.println(" ===");
}

void loop() {
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
      continue;
    }
    if (e.isStop) {
      if (inFrame && addrMatches) {
        Serial.println("--- STOP ---");
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
      if (addrMatches) printFrameHeader();
      continue;
    }

    if (addrMatches) {
      Serial.print("  byte[");
      Serial.print(byteIndex++);
      Serial.print("] = 0x");
      if (e.data < 16) Serial.print("0");
      Serial.print(e.data, HEX);
      Serial.println(e.ack ? "  (ACK)" : "  (NACK)");
    }
  }
}
