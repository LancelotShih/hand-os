/*
 * MMC5983MA Serial Command API
 *
 * Self-contained I2C driver + Serial Monitor CLI covering every
 * datasheet function of the MEMSIC MMC5983MA (±8 G, 18-bit mag).
 *
 * Wiring (I2C):
 *   VDD/VDDIO -> 3.3V   (2.8–3.6 V; do NOT use 5 V)
 *   GND       -> GND
 *   SDA       -> SDA
 *   SCL       -> SCL
 *   SPI_CS    -> VDDIO  (must be high for I2C mode)
 *   CAP       -> 10 uF to GND (required for SET/RESET)
 *
 * Open Serial Monitor at 115200 baud, line ending: Newline or Both.
 * Type "help" for the command list.
 *
 * Datasheet: MMC5983MA Rev A (4/3/2019)
 */

#include <Wire.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Registers / constants (datasheet register map)
// ---------------------------------------------------------------------------
static const uint8_t MMC_I2C_ADDR     = 0x30;   // 7-bit: 0110000
static const uint8_t MMC_PROD_ID_VAL  = 0x30;

static const uint8_t REG_XOUT0        = 0x00;
static const uint8_t REG_XOUT1        = 0x01;
static const uint8_t REG_YOUT0        = 0x02;
static const uint8_t REG_YOUT1        = 0x03;
static const uint8_t REG_ZOUT0        = 0x04;
static const uint8_t REG_ZOUT1        = 0x05;
static const uint8_t REG_XYZOUT2      = 0x06;
static const uint8_t REG_TOUT         = 0x07;
static const uint8_t REG_STATUS       = 0x08;
static const uint8_t REG_CTRL0        = 0x09;
static const uint8_t REG_CTRL1        = 0x0A;
static const uint8_t REG_CTRL2        = 0x0B;
static const uint8_t REG_CTRL3        = 0x0C;
static const uint8_t REG_PRODUCT_ID   = 0x2F;

// Status bits
static const uint8_t ST_MEAS_M_DONE   = (1 << 0);
static const uint8_t ST_MEAS_T_DONE   = (1 << 1);
static const uint8_t ST_OTP_RD_DONE   = (1 << 4);

// CTRL0 bits
static const uint8_t C0_TM_M          = (1 << 0);
static const uint8_t C0_TM_T          = (1 << 1);
static const uint8_t C0_INT_MEAS_DONE = (1 << 2);
static const uint8_t C0_SET           = (1 << 3);
static const uint8_t C0_RESET         = (1 << 4);
static const uint8_t C0_AUTO_SR_EN    = (1 << 5);
static const uint8_t C0_OTP_READ      = (1 << 6);

// CTRL1 bits
static const uint8_t C1_BW0           = (1 << 0);
static const uint8_t C1_BW1           = (1 << 1);
static const uint8_t C1_X_INHIBIT     = (1 << 2);
static const uint8_t C1_YZ_INHIBIT    = (3 << 3);  // bits 3 and 4
static const uint8_t C1_SW_RST        = (1 << 7);

// CTRL2 bits
static const uint8_t C2_CM_FREQ_MASK  = 0x07;
static const uint8_t C2_CMM_EN        = (1 << 3);
static const uint8_t C2_PRD_SET_MASK  = (0x07 << 4);
static const uint8_t C2_EN_PRD_SET    = (1 << 7);

// CTRL3 bits
static const uint8_t C3_ST_ENP        = (1 << 1);  // pos -> neg coil current (saturation check)
static const uint8_t C3_ST_ENM        = (1 << 2);  // neg -> pos coil current (saturation check)
static const uint8_t C3_SPI_3W        = (1 << 6);

// 18-bit null-field midpoint and sensitivity (datasheet)
static const float MMC_NULL_18BIT     = 131072.0f;
static const float MMC_COUNTS_PER_G   = 16384.0f;  // 18-bit: 16384 counts/G

// Measurement timeout (ms) — must exceed longest BW setting (~8 ms) with margin
static const uint16_t MEAS_TIMEOUT_MS = 100;

// ---------------------------------------------------------------------------
// Shadow copies of write-only control registers
// ---------------------------------------------------------------------------
struct CtrlShadow {
  uint8_t ctrl0 = 0;
  uint8_t ctrl1 = 0;
  uint8_t ctrl2 = 0;
  uint8_t ctrl3 = 0;
} shadow;

bool streamEnabled = false;
uint32_t streamPeriodMs = 200;
uint32_t lastStreamMs = 0;

String cmdBuf;

// ---------------------------------------------------------------------------
// Low-level I2C
// ---------------------------------------------------------------------------
bool writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MMC_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MMC_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t n = Wire.requestFrom(MMC_I2C_ADDR, len);
  if (n != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool readReg(uint8_t reg, uint8_t &value) {
  return readRegs(reg, &value, 1);
}

bool writeCtrl0(uint8_t value) {
  shadow.ctrl0 = value;
  return writeReg(REG_CTRL0, value);
}

bool writeCtrl1(uint8_t value) {
  shadow.ctrl1 = value;
  return writeReg(REG_CTRL1, value);
}

bool writeCtrl2(uint8_t value) {
  shadow.ctrl2 = value;
  return writeReg(REG_CTRL2, value);
}

bool writeCtrl3(uint8_t value) {
  shadow.ctrl3 = value;
  return writeReg(REG_CTRL3, value);
}

bool setCtrl0Bits(uint8_t bits)  { return writeCtrl0(shadow.ctrl0 | bits); }
bool clrCtrl0Bits(uint8_t bits)  { return writeCtrl0(shadow.ctrl0 & ~bits); }
bool setCtrl1Bits(uint8_t bits)  { return writeCtrl1(shadow.ctrl1 | bits); }
bool clrCtrl1Bits(uint8_t bits)  { return writeCtrl1(shadow.ctrl1 & ~bits); }
bool setCtrl2Bits(uint8_t bits)  { return writeCtrl2(shadow.ctrl2 | bits); }
bool clrCtrl2Bits(uint8_t bits)  { return writeCtrl2(shadow.ctrl2 & ~bits); }
bool setCtrl3Bits(uint8_t bits)  { return writeCtrl3(shadow.ctrl3 | bits); }
bool clrCtrl3Bits(uint8_t bits)  { return writeCtrl3(shadow.ctrl3 & ~bits); }

// Self-clearing CTRL0 one-shots must not stick in the shadow
bool pulseCtrl0(uint8_t bits) {
  bool ok = writeReg(REG_CTRL0, shadow.ctrl0 | bits);
  // TM_M / TM_T / SET / RESET / OTP_READ self-clear; keep sticky bits only
  shadow.ctrl0 &= (C0_INT_MEAS_DONE | C0_AUTO_SR_EN);
  return ok;
}

// ---------------------------------------------------------------------------
// Device API — every datasheet capability
// ---------------------------------------------------------------------------
bool isConnected() {
  uint8_t id = 0;
  if (!readReg(REG_PRODUCT_ID, id)) return false;
  return id == MMC_PROD_ID_VAL;
}

uint8_t readProductId() {
  uint8_t id = 0;
  readReg(REG_PRODUCT_ID, id);
  return id;
}

uint8_t readStatus() {
  uint8_t s = 0;
  readReg(REG_STATUS, s);
  return s;
}

bool clearStatusBits(uint8_t bits) {
  // Writing 1 clears Meas_M_Done / Meas_T_Done interrupts
  return writeReg(REG_STATUS, bits);
}

bool softReset() {
  bool ok = writeReg(REG_CTRL1, shadow.ctrl1 | C1_SW_RST);
  shadow.ctrl0 = 0;
  shadow.ctrl1 = 0;
  shadow.ctrl2 = 0;
  shadow.ctrl3 = 0;
  delay(15);  // datasheet: ~10 ms power-on after SW_RST
  return ok;
}

bool performSet() {
  bool ok = pulseCtrl0(C0_SET);
  delay(1);  // SET pulse is ~500 ns; settle
  return ok;
}

bool performReset() {
  bool ok = pulseCtrl0(C0_RESET);
  delay(1);
  return ok;
}

bool enableAutoSetReset(bool en) {
  return en ? setCtrl0Bits(C0_AUTO_SR_EN) : clrCtrl0Bits(C0_AUTO_SR_EN);
}

bool enableMeasInterrupt(bool en) {
  return en ? setCtrl0Bits(C0_INT_MEAS_DONE) : clrCtrl0Bits(C0_INT_MEAS_DONE);
}

bool otpRead() {
  return pulseCtrl0(C0_OTP_READ);
}

// BW: 100, 200, 400, 800 Hz  (CTRL1 BW1:BW0)
bool setBandwidth(uint16_t bwHz) {
  uint8_t bits;
  switch (bwHz) {
    case 100: bits = 0; break;
    case 200: bits = C1_BW0; break;
    case 400: bits = C1_BW1; break;
    case 800: bits = C1_BW1 | C1_BW0; break;
    default:  return false;
  }
  uint8_t v = (shadow.ctrl1 & ~(C1_BW1 | C1_BW0)) | bits;
  return writeCtrl1(v);
}

uint16_t getBandwidth() {
  switch (shadow.ctrl1 & (C1_BW1 | C1_BW0)) {
    case 0:                 return 100;
    case C1_BW0:            return 200;
    case C1_BW1:            return 400;
    default:                return 800;
  }
}

uint16_t getMeasWaitMs() {
  // Measurement time from datasheet BW table (+ margin)
  switch (shadow.ctrl1 & (C1_BW1 | C1_BW0)) {
    case 0:                 return 10;   // 8 ms
    case C1_BW0:            return 6;    // 4 ms
    case C1_BW1:            return 4;    // 2 ms
    default:                return 2;    // 0.5 ms
  }
}

bool setXInhibit(bool inhibit) {
  return inhibit ? setCtrl1Bits(C1_X_INHIBIT) : clrCtrl1Bits(C1_X_INHIBIT);
}

bool setYZInhibit(bool inhibit) {
  return inhibit ? setCtrl1Bits(C1_YZ_INHIBIT) : clrCtrl1Bits(C1_YZ_INHIBIT);
}

// Continuous mode frequency codes (CM_Freq[2:0])
// 0=off, 1=1Hz, 2=10, 3=20, 4=50, 5=100, 6=200(BW=01), 7=1000(BW=11)
bool setContinuousFreq(uint16_t hz) {
  uint8_t code;
  switch (hz) {
    case 0:    code = 0; break;
    case 1:    code = 1; break;
    case 10:   code = 2; break;
    case 20:   code = 3; break;
    case 50:   code = 4; break;
    case 100:  code = 5; break;
    case 200:  code = 6; break;
    case 1000: code = 7; break;
    default:   return false;
  }
  uint8_t v = (shadow.ctrl2 & ~C2_CM_FREQ_MASK) | code;
  return writeCtrl2(v);
}

uint16_t getContinuousFreq() {
  static const uint16_t table[] = {0, 1, 10, 20, 50, 100, 200, 1000};
  return table[shadow.ctrl2 & C2_CM_FREQ_MASK];
}

bool enableContinuousMode(bool en) {
  if (en && (shadow.ctrl2 & C2_CM_FREQ_MASK) == 0) return false;
  return en ? setCtrl2Bits(C2_CMM_EN) : clrCtrl2Bits(C2_CMM_EN);
}

// Periodic SET sample interval codes
// 0=1, 1=25, 2=75, 3=100, 4=250, 5=500, 6=1000, 7=2000
bool setPeriodicSetSamples(uint16_t n) {
  uint8_t code;
  switch (n) {
    case 1:    code = 0; break;
    case 25:   code = 1; break;
    case 75:   code = 2; break;
    case 100:  code = 3; break;
    case 250:  code = 4; break;
    case 500:  code = 5; break;
    case 1000: code = 6; break;
    case 2000: code = 7; break;
    default:   return false;
  }
  uint8_t v = (shadow.ctrl2 & ~C2_PRD_SET_MASK) | (code << 4);
  return writeCtrl2(v);
}

uint16_t getPeriodicSetSamples() {
  static const uint16_t table[] = {1, 25, 75, 100, 250, 500, 1000, 2000};
  return table[(shadow.ctrl2 >> 4) & 0x07];
}

bool enablePeriodicSet(bool en) {
  // Needs Auto_SR_en and Cmm_en as well (datasheet)
  return en ? setCtrl2Bits(C2_EN_PRD_SET) : clrCtrl2Bits(C2_EN_PRD_SET);
}

bool applySatCurrentPosToNeg(bool en) {
  return en ? setCtrl3Bits(C3_ST_ENP) : clrCtrl3Bits(C3_ST_ENP);
}

bool applySatCurrentNegToPos(bool en) {
  return en ? setCtrl3Bits(C3_ST_ENM) : clrCtrl3Bits(C3_ST_ENM);
}

bool enable3WireSPI(bool en) {
  return en ? setCtrl3Bits(C3_SPI_3W) : clrCtrl3Bits(C3_SPI_3W);
}

// ---------------------------------------------------------------------------
// Measurement helpers
// ---------------------------------------------------------------------------
bool waitMeasDone(uint8_t doneBit) {
  uint32_t start = millis();
  while (millis() - start < MEAS_TIMEOUT_MS) {
    uint8_t s = readStatus();
    if (s & doneBit) return true;
    delay(1);
  }
  return false;
}

bool readRawXYZ(uint32_t &x, uint32_t &y, uint32_t &z) {
  uint8_t buf[7];
  if (!readRegs(REG_XOUT0, buf, 7)) return false;

  // 18-bit: [17:10]=out0, [9:2]=out1, [1:0] packed in XYZout2
  x = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | ((buf[6] >> 6) & 0x03);
  y = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | ((buf[6] >> 4) & 0x03);
  z = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | ((buf[6] >> 2) & 0x03);
  return true;
}

float countsToGauss(uint32_t counts) {
  return ((float)counts - MMC_NULL_18BIT) / MMC_COUNTS_PER_G;
}

bool takeMagMeasurement(uint32_t &x, uint32_t &y, uint32_t &z) {
  clearStatusBits(ST_MEAS_M_DONE);
  if (!pulseCtrl0(C0_TM_M)) return false;
  delay(getMeasWaitMs());
  if (!waitMeasDone(ST_MEAS_M_DONE)) return false;
  return readRawXYZ(x, y, z);
}

bool takeTempMeasurement(float &tempC) {
  clearStatusBits(ST_MEAS_T_DONE);
  if (!pulseCtrl0(C0_TM_T)) return false;
  delay(getMeasWaitMs());
  if (!waitMeasDone(ST_MEAS_T_DONE)) return false;
  uint8_t t = 0;
  if (!readReg(REG_TOUT, t)) return false;
  // Datasheet: 0x00 = -75 °C, ~0.8 °C/LSB
  tempC = -75.0f + (float)t * 0.8f;
  return true;
}

void printXYZGauss(uint32_t x, uint32_t y, uint32_t z) {
  float gx = countsToGauss(x);
  float gy = countsToGauss(y);
  float gz = countsToGauss(z);
  float mag = sqrtf(gx * gx + gy * gy + gz * gz);

  Serial.print(F("X: "));
  Serial.print(gx, 4);
  Serial.print(F(" G   Y: "));
  Serial.print(gy, 4);
  Serial.print(F(" G   Z: "));
  Serial.print(gz, 4);
  Serial.print(F(" G   |B|: "));
  Serial.print(mag, 4);
  Serial.println(F(" G"));
}

void printXYZRaw(uint32_t x, uint32_t y, uint32_t z) {
  Serial.print(F("X_raw: "));
  Serial.print(x);
  Serial.print(F("   Y_raw: "));
  Serial.print(y);
  Serial.print(F("   Z_raw: "));
  Serial.println(z);
}

// ---------------------------------------------------------------------------
// Datasheet example sequences
// ---------------------------------------------------------------------------

// EXAMPLE MEASUREMENT (datasheet pp. data-transfer section)
bool exampleMeasurement() {
  Serial.println(F("--- Datasheet EXAMPLE MEASUREMENT ---"));
  Serial.println(F("1) Write CTRL0 TM_M=1 to start acquisition"));
  clearStatusBits(ST_MEAS_M_DONE);
  if (!pulseCtrl0(C0_TM_M)) {
    Serial.println(F("FAIL: TM_M write"));
    return false;
  }

  Serial.println(F("2) Poll STATUS until Meas_M_Done=1"));
  if (!waitMeasDone(ST_MEAS_M_DONE)) {
    Serial.println(F("FAIL: Meas_M_Done timeout"));
    return false;
  }
  Serial.print(F("   STATUS=0x"));
  Serial.println(readStatus(), HEX);

  Serial.println(F("3) Burst-read Xout0..XYZout2 (18-bit)"));
  uint32_t x, y, z;
  if (!readRawXYZ(x, y, z)) {
    Serial.println(F("FAIL: data read"));
    return false;
  }
  printXYZRaw(x, y, z);
  printXYZGauss(x, y, z);
  Serial.println(F("PASS: example measurement complete"));
  return true;
}

// EXAMPLE OF SET (CTRL0 bit Set = 0x08)
bool exampleSet() {
  Serial.println(F("--- Datasheet EXAMPLE OF SET ---"));
  Serial.println(F("Write CTRL0 = 0x08 (SET bit)"));
  bool ok = performSet();
  Serial.println(ok ? F("PASS: SET done (~500 ns coil pulse)") : F("FAIL: SET"));
  return ok;
}

// EXAMPLE OF RESET (CTRL0 bit Reset = 0x10)
bool exampleReset() {
  Serial.println(F("--- Datasheet EXAMPLE OF RESET ---"));
  Serial.println(F("Write CTRL0 = 0x10 (RESET bit)"));
  bool ok = performReset();
  Serial.println(ok ? F("PASS: RESET done (~500 ns coil pulse)") : F("FAIL: RESET"));
  return ok;
}

// Run all three datasheet protocol examples back-to-back
bool exampleAll() {
  bool ok = true;
  ok &= exampleSet();
  ok &= exampleMeasurement();
  ok &= exampleReset();
  ok &= exampleMeasurement();
  Serial.println(ok ? F("=== ALL EXAMPLE TESTS PASSED ===")
                    : F("=== EXAMPLE TESTS HAD FAILURES ==="));
  return ok;
}

// Offset-cancel protocol from datasheet:
// SET -> meas1 (+H+Offset) -> RESET -> meas2 (-H+Offset) -> H=(m1-m2)/2
bool measureOffsetCancelled() {
  Serial.println(F("--- SET/RESET offset-cancelled measurement ---"));

  if (!performSet()) {
    Serial.println(F("FAIL: SET"));
    return false;
  }
  uint32_t x1, y1, z1;
  if (!takeMagMeasurement(x1, y1, z1)) {
    Serial.println(F("FAIL: measurement after SET"));
    return false;
  }

  if (!performReset()) {
    Serial.println(F("FAIL: RESET"));
    return false;
  }
  uint32_t x2, y2, z2;
  if (!takeMagMeasurement(x2, y2, z2)) {
    Serial.println(F("FAIL: measurement after RESET"));
    return false;
  }

  // H = (Output1 - Output2) / 2   in counts, then to Gauss
  // Offset = (Output1 + Output2) / 2
  float hx = ((float)x1 - (float)x2) / 2.0f / MMC_COUNTS_PER_G;
  float hy = ((float)y1 - (float)y2) / 2.0f / MMC_COUNTS_PER_G;
  float hz = ((float)z1 - (float)z2) / 2.0f / MMC_COUNTS_PER_G;
  float ox = (((float)x1 + (float)x2) / 2.0f - MMC_NULL_18BIT) / MMC_COUNTS_PER_G;
  float oy = (((float)y1 + (float)y2) / 2.0f - MMC_NULL_18BIT) / MMC_COUNTS_PER_G;
  float oz = (((float)z1 + (float)z2) / 2.0f - MMC_NULL_18BIT) / MMC_COUNTS_PER_G;

  Serial.println(F("After SET:"));
  printXYZGauss(x1, y1, z1);
  Serial.println(F("After RESET:"));
  printXYZGauss(x2, y2, z2);

  Serial.print(F("H (offset-free)  X: "));
  Serial.print(hx, 4);
  Serial.print(F(" G   Y: "));
  Serial.print(hy, 4);
  Serial.print(F(" G   Z: "));
  Serial.print(hz, 4);
  Serial.println(F(" G"));

  Serial.print(F("Bridge offset    X: "));
  Serial.print(ox, 4);
  Serial.print(F(" G   Y: "));
  Serial.print(oy, 4);
  Serial.print(F(" G   Z: "));
  Serial.print(oz, 4);
  Serial.println(F(" G"));
  return true;
}

// Saturation check using St_enp / St_enm (CTRL3)
// Healthy sensor: applying coil current shifts the reading noticeably.
// Saturated sensor: little/no change → run SET/RESET to recover.
bool checkSaturation() {
  Serial.println(F("--- Saturation check (St_enp / St_enm) ---"));

  // Clear any leftover self-test currents
  applySatCurrentPosToNeg(false);
  applySatCurrentNegToPos(false);
  delay(2);

  uint32_t xb, yb, zb;
  if (!takeMagMeasurement(xb, yb, zb)) {
    Serial.println(F("FAIL: baseline measurement"));
    return false;
  }
  Serial.print(F("Baseline:   "));
  printXYZGauss(xb, yb, zb);

  applySatCurrentPosToNeg(true);
  delay(2);
  uint32_t xp, yp, zp;
  if (!takeMagMeasurement(xp, yp, zp)) {
    applySatCurrentPosToNeg(false);
    Serial.println(F("FAIL: St_enp measurement"));
    return false;
  }
  applySatCurrentPosToNeg(false);
  Serial.print(F("St_enp ON:  "));
  printXYZGauss(xp, yp, zp);

  applySatCurrentNegToPos(true);
  delay(2);
  uint32_t xm, ym, zm;
  if (!takeMagMeasurement(xm, ym, zm)) {
    applySatCurrentNegToPos(false);
    Serial.println(F("FAIL: St_enm measurement"));
    return false;
  }
  applySatCurrentNegToPos(false);
  Serial.print(F("St_enm ON:  "));
  printXYZGauss(xm, ym, zm);

  float dbx = fabsf(countsToGauss(xp) - countsToGauss(xb));
  float dby = fabsf(countsToGauss(yp) - countsToGauss(yb));
  float dbz = fabsf(countsToGauss(zp) - countsToGauss(zb));
  float dmx = fabsf(countsToGauss(xm) - countsToGauss(xb));
  float dmy = fabsf(countsToGauss(ym) - countsToGauss(yb));
  float dmz = fabsf(countsToGauss(zm) - countsToGauss(zb));

  float maxDelta = dbx;
  if (dby > maxDelta) maxDelta = dby;
  if (dbz > maxDelta) maxDelta = dbz;
  if (dmx > maxDelta) maxDelta = dmx;
  if (dmy > maxDelta) maxDelta = dmy;
  if (dmz > maxDelta) maxDelta = dmz;

  Serial.print(F("Max |delta| from baseline: "));
  Serial.print(maxDelta, 4);
  Serial.println(F(" G"));

  // Threshold: coil self-test field should move reading by tens of mG+.
  // If max delta is tiny, sensor is likely saturated / stuck.
  const float SAT_THRESHOLD_G = 0.02f;  // 20 mG
  if (maxDelta < SAT_THRESHOLD_G) {
    Serial.println(F("RESULT: LIKELY SATURATED (little response to coil current)"));
    Serial.println(F("Tip: run 'set' then 'magreset', or 'softreset', then retest."));
    return false;
  }

  Serial.println(F("RESULT: NOT SATURATED (sensor responds to self-test field)"));
  return true;
}

// ---------------------------------------------------------------------------
// Serial CLI
// ---------------------------------------------------------------------------
void printHelp() {
  Serial.println();
  Serial.println(F("MMC5983MA commands (case-insensitive):"));
  Serial.println(F("  help                 - this list"));
  Serial.println(F("  id                   - read Product ID (expect 0x30)"));
  Serial.println(F("  status               - read STATUS register"));
  Serial.println(F("  clearstatus          - clear Meas_M/T_Done interrupt flags"));
  Serial.println(F("  regs                 - dump control shadow + status"));
  Serial.println();
  Serial.println(F("  softreset            - SW_RST (full chip reset, ~10 ms)"));
  Serial.println(F("  set                  - SET coil pulse (datasheet example)"));
  Serial.println(F("  magreset | reset     - RESET coil pulse (datasheet example)"));
  Serial.println(F("  otpread              - re-read OTP into shadow regs"));
  Serial.println();
  Serial.println(F("  xyz | read | measure - single mag reading in Gauss"));
  Serial.println(F("  raw                  - single mag reading as raw 18-bit counts"));
  Serial.println(F("  temp                 - temperature reading (°C)"));
  Serial.println(F("  offset               - SET/RESET offset-cancelled H measurement"));
  Serial.println();
  Serial.println(F("  sat | saturate       - saturation self-test (St_enp/St_enm)"));
  Serial.println(F("  satpos on|off        - apply/remove pos->neg coil current"));
  Serial.println(F("  satneg on|off        - apply/remove neg->pos coil current"));
  Serial.println();
  Serial.println(F("  example              - run datasheet EXAMPLE MEASUREMENT"));
  Serial.println(F("  exampleset           - run datasheet EXAMPLE OF SET"));
  Serial.println(F("  examplereset         - run datasheet EXAMPLE OF RESET"));
  Serial.println(F("  exampleall           - run SET + MEASURE + RESET + MEASURE"));
  Serial.println();
  Serial.println(F("  autosr on|off        - automatic SET/RESET"));
  Serial.println(F("  int on|off           - measurement-done interrupt enable"));
  Serial.println(F("  bw 100|200|400|800   - decimation filter bandwidth (Hz)"));
  Serial.println(F("  cmmfreq <Hz>         - continuous mode rate: 0,1,10,20,50,100,200,1000"));
  Serial.println(F("  cmm on|off           - continuous measurement mode"));
  Serial.println(F("  prdset <N>           - periodic SET every N samples: 1,25,75,100,250,500,1000,2000"));
  Serial.println(F("  prd on|off           - enable periodic SET (needs autosr+cmm)"));
  Serial.println(F("  xinhibit on|off      - inhibit X channel"));
  Serial.println(F("  yzinhibit on|off     - inhibit Y and Z channels"));
  Serial.println(F("  spi3w on|off         - 3-wire SPI mode bit (I2C boards: leave off)"));
  Serial.println();
  Serial.println(F("  stream on [ms]       - auto-print xyz every ms (default 200)"));
  Serial.println(F("  stream off           - stop streaming"));
  Serial.println();
}

String lower(const String &s) {
  String o = s;
  o.toLowerCase();
  o.trim();
  return o;
}

bool parseOnOff(const String &arg, bool &out) {
  String a = lower(arg);
  if (a == "on" || a == "1" || a == "true")  { out = true;  return true; }
  if (a == "off" || a == "0" || a == "false") { out = false; return true; }
  return false;
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Split into cmd + optional arg(s)
  int sp = line.indexOf(' ');
  String cmd = lower(sp < 0 ? line : line.substring(0, sp));
  String rest = sp < 0 ? "" : line.substring(sp + 1);
  rest.trim();
  int sp2 = rest.indexOf(' ');
  String arg1 = sp2 < 0 ? rest : rest.substring(0, sp2);
  String arg2 = sp2 < 0 ? "" : rest.substring(sp2 + 1);
  arg1.trim();
  arg2.trim();

  if (cmd == "help" || cmd == "?") {
    printHelp();
    return;
  }

  if (cmd == "id" || cmd == "productid") {
    uint8_t id = readProductId();
    Serial.print(F("Product ID: 0x"));
    if (id < 16) Serial.print('0');
    Serial.print(id, HEX);
    Serial.println(id == MMC_PROD_ID_VAL ? F("  (OK)") : F("  (UNEXPECTED)"));
    return;
  }

  if (cmd == "status") {
    uint8_t s = readStatus();
    Serial.print(F("STATUS=0x"));
    if (s < 16) Serial.print('0');
    Serial.println(s, HEX);
    Serial.print(F("  Meas_M_Done="));
    Serial.println((s & ST_MEAS_M_DONE) ? 1 : 0);
    Serial.print(F("  Meas_T_Done="));
    Serial.println((s & ST_MEAS_T_DONE) ? 1 : 0);
    Serial.print(F("  OTP_Rd_Done="));
    Serial.println((s & ST_OTP_RD_DONE) ? 1 : 0);
    return;
  }

  if (cmd == "clearstatus") {
    Serial.println(clearStatusBits(ST_MEAS_M_DONE | ST_MEAS_T_DONE)
                       ? F("status flags cleared")
                       : F("FAIL"));
    return;
  }

  if (cmd == "regs") {
    Serial.print(F("CTRL0 shadow=0x")); Serial.println(shadow.ctrl0, HEX);
    Serial.print(F("CTRL1 shadow=0x")); Serial.println(shadow.ctrl1, HEX);
    Serial.print(F("CTRL2 shadow=0x")); Serial.println(shadow.ctrl2, HEX);
    Serial.print(F("CTRL3 shadow=0x")); Serial.println(shadow.ctrl3, HEX);
    Serial.print(F("BW=")); Serial.print(getBandwidth()); Serial.println(F(" Hz"));
    Serial.print(F("CMM freq=")); Serial.print(getContinuousFreq()); Serial.println(F(" Hz"));
    Serial.print(F("Periodic SET every ")); Serial.print(getPeriodicSetSamples()); Serial.println(F(" samples"));
    uint8_t s = readStatus();
    Serial.print(F("STATUS=0x")); Serial.println(s, HEX);
    return;
  }

  if (cmd == "softreset") {
    Serial.println(softReset() ? F("soft reset done") : F("FAIL"));
    return;
  }

  if (cmd == "set") {
    Serial.println(performSet() ? F("SET done") : F("FAIL"));
    return;
  }

  // "reset" = magnetic RESET coil (what user asked for); softreset is separate
  if (cmd == "reset" || cmd == "magreset") {
    Serial.println(performReset() ? F("RESET (coil) done") : F("FAIL"));
    return;
  }

  if (cmd == "otpread") {
    Serial.println(otpRead() ? F("OTP read triggered") : F("FAIL"));
    delay(5);
    Serial.print(F("OTP_Rd_Done="));
    Serial.println((readStatus() & ST_OTP_RD_DONE) ? 1 : 0);
    return;
  }

  if (cmd == "xyz" || cmd == "read" || cmd == "measure") {
    uint32_t x, y, z;
    if (!takeMagMeasurement(x, y, z)) {
      Serial.println(F("FAIL: measurement"));
      return;
    }
    printXYZGauss(x, y, z);
    return;
  }

  if (cmd == "raw") {
    uint32_t x, y, z;
    if (!takeMagMeasurement(x, y, z)) {
      Serial.println(F("FAIL: measurement"));
      return;
    }
    printXYZRaw(x, y, z);
    printXYZGauss(x, y, z);
    return;
  }

  if (cmd == "temp" || cmd == "temperature") {
    float t;
    if (!takeTempMeasurement(t)) {
      Serial.println(F("FAIL: temperature"));
      return;
    }
    Serial.print(F("Temperature: "));
    Serial.print(t, 1);
    Serial.println(F(" C"));
    return;
  }

  if (cmd == "offset") {
    measureOffsetCancelled();
    return;
  }

  if (cmd == "sat" || cmd == "saturate" || cmd == "saturation") {
    checkSaturation();
    return;
  }

  if (cmd == "satpos") {
    bool en;
    if (!parseOnOff(arg1, en)) {
      Serial.println(F("usage: satpos on|off"));
      return;
    }
    Serial.println(applySatCurrentPosToNeg(en)
                       ? (en ? F("St_enp ON") : F("St_enp OFF"))
                       : F("FAIL"));
    return;
  }

  if (cmd == "satneg") {
    bool en;
    if (!parseOnOff(arg1, en)) {
      Serial.println(F("usage: satneg on|off"));
      return;
    }
    Serial.println(applySatCurrentNegToPos(en)
                       ? (en ? F("St_enm ON") : F("St_enm OFF"))
                       : F("FAIL"));
    return;
  }

  if (cmd == "example" || cmd == "examplemeas") {
    exampleMeasurement();
    return;
  }
  if (cmd == "exampleset") {
    exampleSet();
    return;
  }
  if (cmd == "examplereset") {
    exampleReset();
    return;
  }
  if (cmd == "exampleall" || cmd == "test") {
    exampleAll();
    return;
  }

  if (cmd == "autosr") {
    bool en;
    if (!parseOnOff(arg1, en)) {
      Serial.println(F("usage: autosr on|off"));
      return;
    }
    Serial.println(enableAutoSetReset(en) ? F("OK") : F("FAIL"));
    return;
  }

  if (cmd == "int" || cmd == "interrupt") {
    bool en;
    if (!parseOnOff(arg1, en)) {
      Serial.println(F("usage: int on|off"));
      return;
    }
    Serial.println(enableMeasInterrupt(en) ? F("OK") : F("FAIL"));
    return;
  }

  if (cmd == "bw" || cmd == "bandwidth") {
    if (arg1.length() == 0) {
      Serial.print(F("BW="));
      Serial.print(getBandwidth());
      Serial.println(F(" Hz"));
      return;
    }
    uint16_t bw = (uint16_t)arg1.toInt();
    Serial.println(setBandwidth(bw) ? F("OK") : F("FAIL (use 100|200|400|800)"));
    return;
  }

  if (cmd == "cmmfreq") {
    if (arg1.length() == 0) {
      Serial.print(F("CMM freq="));
      Serial.print(getContinuousFreq());
      Serial.println(F(" Hz"));
      return;
    }
    uint16_t hz = (uint16_t)arg1.toInt();
    Serial.println(setContinuousFreq(hz)
                       ? F("OK")
                       : F("FAIL (use 0|1|10|20|50|100|200|1000)"));
    return;
  }

  if (cmd == "cmm" || cmd == "continuous") {
    bool en;
    if (!parseOnOff(arg1, en)) {
      Serial.println(F("usage: cmm on|off   (set cmmfreq first if enabling)"));
      return;
    }
    if (!enableContinuousMode(en)) {
      Serial.println(F("FAIL (set cmmfreq to non-zero before enabling)"));
      return;
    }
    Serial.println(F("OK"));
    return;
  }

  if (cmd == "prdset") {
    if (arg1.length() == 0) {
      Serial.print(F("Periodic SET every "));
      Serial.print(getPeriodicSetSamples());
      Serial.println(F(" samples"));
      return;
    }
    uint16_t n = (uint16_t)arg1.toInt();
    Serial.println(setPeriodicSetSamples(n)
                       ? F("OK")
                       : F("FAIL (use 1|25|75|100|250|500|1000|2000)"));
    return;
  }

  if (cmd == "prd" || cmd == "periodic") {
    bool en;
    if (!parseOnOff(arg1, en)) {
      Serial.println(F("usage: prd on|off"));
      return;
    }
    Serial.println(enablePeriodicSet(en) ? F("OK") : F("FAIL"));
    if (en) {
      Serial.println(F("Note: datasheet requires autosr on + cmm on for periodic SET"));
    }
    return;
  }

  if (cmd == "xinhibit") {
    bool en;
    if (!parseOnOff(arg1, en)) {
      Serial.println(F("usage: xinhibit on|off"));
      return;
    }
    Serial.println(setXInhibit(en) ? F("OK") : F("FAIL"));
    return;
  }

  if (cmd == "yzinhibit") {
    bool en;
    if (!parseOnOff(arg1, en)) {
      Serial.println(F("usage: yzinhibit on|off"));
      return;
    }
    Serial.println(setYZInhibit(en) ? F("OK") : F("FAIL"));
    return;
  }

  if (cmd == "spi3w") {
    bool en;
    if (!parseOnOff(arg1, en)) {
      Serial.println(F("usage: spi3w on|off"));
      return;
    }
    Serial.println(enable3WireSPI(en) ? F("OK") : F("FAIL"));
    return;
  }

  if (cmd == "stream") {
    String a = lower(arg1);
    if (a == "off" || a == "0") {
      streamEnabled = false;
      Serial.println(F("stream off"));
      return;
    }
    if (a == "on" || a == "1" || a.length() == 0) {
      streamEnabled = true;
      if (arg2.length() > 0) streamPeriodMs = (uint32_t)arg2.toInt();
      else if (a != "on" && a != "1" && arg1.length() > 0)
        streamPeriodMs = (uint32_t)arg1.toInt();
      if (streamPeriodMs < 10) streamPeriodMs = 10;
      Serial.print(F("stream on, period "));
      Serial.print(streamPeriodMs);
      Serial.println(F(" ms"));
      return;
    }
    // "stream 100" shorthand
    if (arg1.toInt() > 0) {
      streamEnabled = true;
      streamPeriodMs = (uint32_t)arg1.toInt();
      Serial.print(F("stream on, period "));
      Serial.print(streamPeriodMs);
      Serial.println(F(" ms"));
      return;
    }
    Serial.println(F("usage: stream on [ms] | stream off"));
    return;
  }

  Serial.print(F("Unknown command: "));
  Serial.println(cmd);
  Serial.println(F("Type 'help' for the list."));
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { /* wait for USB serial on some boards */ }

  Wire.begin();
  Wire.setClock(400000);  // I2C FAST mode (datasheet max 400 kHz)

  Serial.println();
  Serial.println(F("MMC5983MA Serial API"));
  Serial.println(F("===================="));

  delay(20);  // allow sensor power-up

  if (!isConnected()) {
    Serial.println(F("ERROR: MMC5983MA not found at 0x30"));
    Serial.println(F("Check wiring, 3.3 V supply, and SPI_CS tied high for I2C."));
  } else {
    Serial.println(F("MMC5983MA detected (Product ID 0x30)"));
  }

  softReset();
  setBandwidth(100);
  performSet();  // condition sensor (datasheet recommends SET before measuring)

  printHelp();
  Serial.println(F("Ready."));
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdBuf.length() > 0) {
        handleCommand(cmdBuf);
        cmdBuf = "";
      }
    } else if (c >= 32 && c < 127) {
      if (cmdBuf.length() < 64) cmdBuf += c;
    }
  }

  if (streamEnabled && (millis() - lastStreamMs >= streamPeriodMs)) {
    lastStreamMs = millis();
    uint32_t x, y, z;
    if (takeMagMeasurement(x, y, z)) {
      printXYZGauss(x, y, z);
    } else {
      Serial.println(F("stream: measurement failed"));
    }
  }
}