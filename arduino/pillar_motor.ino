#include <Wire.h>

#define I2C_ADDR 0x18

// ===== Pins =====
const uint8_t ENA = 5;   // PWM
const uint8_t IN1 = 6;
const uint8_t IN2 = 7;
const uint8_t ENB = 10;  // PWM
const uint8_t IN3 = 8;
const uint8_t IN4 = 9;

// ===== RX buffers (ISR-safe) =====
volatile bool     rx_ready = false;
volatile uint8_t  rx_len   = 0;
volatile char     rx_buf[32];

char cmd_buf[32];  // processed in loop

// ===== Motor helpers =====
inline void pillarUp()    { analogWrite(ENA, 255); digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
inline void pillarDown()  { analogWrite(ENA, 255); digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
inline void pillarStop()  { analogWrite(ENA, 0);   digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
inline void shootStart()  { analogWrite(ENB, 255); digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
inline void shootStop()   { analogWrite(ENB, 0);   digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
inline void shootReverse(){ analogWrite(ENB, 0);   digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);  }

// ---- utils ----
static void sanitize(char* s, uint8_t& n) {
  while (n && (s[n-1]=='\r' || s[n-1]=='\n' || s[n-1]==' ' || s[n-1]=='\t')) n--;
  s[n] = '\0';
  for (uint8_t i=0; i<n; ++i) { char c=s[i]; if (c>='A' && c<='Z') s[i]=c-'A'+'a'; }
}
static bool equals(const char* a, const char* b) {
  while (*a && *b) { if (*a++ != *b++) return false; }
  return *a==0 && *b==0;
}

// ---- I2C ISRs ----
void onI2CReceive(int) {
  uint8_t i=0;
  while (Wire.available() && i < sizeof(rx_buf)-1) rx_buf[i++] = (char)Wire.read();
  while (Wire.available()) (void)Wire.read();  // flush extras
  rx_buf[i]='\0';
  rx_len=i;
  rx_ready=true;
}

void onI2CRequest() {
  // 간단 상태 1바이트 응답: 최근에 명령 수신했는지
  Wire.write(rx_ready ? 1 : 0);
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pillarStop(); shootStop();

  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
}

void loop() {
  if (rx_ready) {
    noInterrupts();
    uint8_t n = rx_len;
    if (n >= sizeof(cmd_buf)) n = sizeof(cmd_buf)-1;
    for (uint8_t i=0;i<n;++i) cmd_buf[i]=rx_buf[i];
    cmd_buf[n]='\0';
    rx_ready=false;
    interrupts();

    sanitize(cmd_buf, n);

    if      (equals(cmd_buf, "up"))           pillarUp();
    else if (equals(cmd_buf, "down"))         pillarDown();
    else if (equals(cmd_buf, "pillar_stop"))  pillarStop();
    else if (equals(cmd_buf, "shoot"))        shootStart();
    else if (equals(cmd_buf, "shoot_stop"))   shootStop();
    else if (equals(cmd_buf, "shoot_rev"))    shootReverse();
    // 그 외는 무시
  }
}
