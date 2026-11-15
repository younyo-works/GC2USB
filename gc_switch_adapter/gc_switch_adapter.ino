/*
 * ============================================================
 *  GC コントローラー -> Switch 2 / PC 変換アダプター  v1.2
 * ============================================================
 *  RP2040-Zero でゲームキューブコントローラーを読み取り、
 *  有線 USB 接続の HORI ポッケンコントローラー互換機として
 *  Nintendo Switch 2 で使うためのファームウェア。
 *  起動方法で PC 用のデバッグ/アナログモードにも切り替わる。
 *
 *  ■ 配線 (GC コネクタ正面、中央のベロを挟んで)
 *    上段 左->右: 1=5V(未接続) / 2=DATA / 3=GND
 *    下段 左->右: 4=GND / 5=未使用 / 6=3.3V
 *    外側シェル:   シールド (GND)
 *    DATA は GP29 へ。DATA-3.3V 間に 1kΩ のプルアップを
 *    入れる (純正パッド内蔵のプルアップと併用)。
 *
 *  ■ 使い方
 *    通常の USB 接続        -> Switch モード (LED: 青)
 *    GC の Start を押しながら
 *      USB 接続              -> PC モード (LED: 緑)
 *    LED は起動中とパッド未検出の間はゆっくり明滅し、
 *    パッドを検出するとモード色で点灯する。
 *    PC でも通常は Switch モードのまま使える (Windows
 *    からは POKKEN CONTROLLER として認識される)。
 *    PC モードは PC 専用の特殊モード: 名乗りは同じ
 *    POKKEN CONTROLLER のままで、L/R のアナログ値が追加
 *    の軸 (Rx / Ry) として出る。レポート形式が Switch 用
 *    と異なるため、このモードで Switch に接続しても
 *    使えない。シリアルは両モードとも無効。
 *
 *  ■ マッピング (Switch モード)
 *    A/B/X/Y・両スティック・十字キー: そのまま
 *    Start -> + (コンボ判定のため約100ms遅れて発火する。
 *               100ms未満の短押しでは + は出ない)
 *    肩ボタンはプロファイル切替式 (起動時は「汎用」):
 *      汎用:     L -> L  / R -> R  / Z -> ZR
 *      スマブラ: L -> ZL / R -> ZR / Z -> R
 *      切替操作: L+R+Z を約1秒長押し
 *    同時押しコンボ (発火中は元の入力を抑制):
 *      Start + Z            -> -
 *      Start + 十字キー下   -> キャプチャー
 *      L + R + Start 約1秒 -> Home
 *    L/R はデジタルのクリックに加え、アナログ値 190 以上
 *    でも押下扱いになる (クリックまで押し切らなくても
 *    反応する補助)。
 *
 *  ■ スティック調整
 *    起動直後に約0.4秒間、触らずに置いておくと中心と
 *    トリガーの静止値を自動でキャリブレーションする。
 *    可動域のゲインは GAIN_MAIN_PCT / GAIN_C_PCT (%) で
 *    調整できる (半径方向ゲイン。八角形ゲートの形状差を
 *    吸収するため軸ごとではなくベクトル長に掛ける)。
 *
 *  ■ 設計メモ (実装上の要点)
 *    - 両モードとも Serial.end() で CDC を完全に落とす。
 *      CDC が混ざると Switch に認識されない。
 *    - USB は detach した状態でモード判定と構成を行い、
 *      最終構成だけで attach する。途中のデフォルト構成
 *      (CDC のみ) をホストに見せないのが重要。
 *    - GC の読み取りはプローブではなくポーリング直接が
 *      安定する。起動直後はパッドの起動を 500ms 待って
 *      から話しかける。
 *    - レポートは常時送信する。パッド未検出の間も
 *      ニュートラルを送り続けることで、ホスト側の登録が
 *      安定する。
 *    - PC モードのレポートは Switch 用に Rx / Ry を追加
 *      した 10 バイト。名乗り (VID/PID と文字列) は Switch
 *      モードと同一にし、CDC も持たない (Switch モードと
 *      の差はディスクリプタだけ)。汎用 HID パッドとして
 *      出すと Windows の GameInput が仮想デバイス
 *      (Synthetic Gamepad) を作りに来るため、この形に
 *      してある。
 *    - Switch のレポート形式は LUFA 系ファイトスティック
 *      と同じ 8 バイト (16ボタン / ハット / 8bit軸 x4 /
 *      ベンダ)。スティックは 0..255 のフルレンジを出す
 *      (符号付き -127..127 を単純に +128 すると最小が 1
 *      になって 0 が出ないため、半径クランプを 128 に
 *      して変換している)。
 *
 *  ■ 動作確認
 *    Nintendo Switch 2 (ドック経由)、Windows PC
 *    (joy.cpl / ブラウザのゲームパッドテスター)。
 *    ボードは Waveshare RP2040-Zero、USB スタックは
 *    TinyUSB (Earle Philhower RP2040 コア + Arduino IDE)。
 * ============================================================
 */

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>  // TinyUSB スタック使用時に必要
#include <Adafruit_NeoPixel.h>
#include <math.h>

#include "hardware/gpio.h"  // GPIO を SDK で直接制御する

#define GC_DATA_PIN 29
#define GC_Y_INVERT 1

// スティックのゲイン (%)。半径方向にかける。
// 八角形ゲートの「辺の中央 (真横・真縦) は頂点 (斜め) より
// 中心に近い」という形状差を吸収するため、軸ごとではなく
// ベクトルの長さに対してゲインをかけ、外周は円周上に
// クランプする。数字を変えれば効き具合を調整できる。
#define GAIN_MAIN_PCT 160  // メインスティック
#define GAIN_C_PCT    170  // Cスティック

// ---- Switch 向け HID (HORI ポッケン互換) ----
static const uint8_t desc_hid_report[] = {
  0x05, 0x01,  // Usage Page (Generic Desktop)
  0x09, 0x05,  // Usage (Game Pad)
  0xA1, 0x01,  // Collection (Application)
  // Buttons (16 bits)
  0x15, 0x00,  //   Logical Minimum (0)
  0x25, 0x01,  //   Logical Maximum (1)
  0x35, 0x00,  //   Physical Minimum (0)
  0x45, 0x01,  //   Physical Maximum (1)
  0x75, 0x01,  //   Report Size (1)
  0x95, 0x10,  //   Report Count (16)
  0x05, 0x09,  //   Usage Page (Button)
  0x19, 0x01,  //   Usage Minimum (1)
  0x29, 0x10,  //   Usage Maximum (16)
  0x81, 0x02,  //   Input (Data,Var,Abs)
  // Hat switch (4 bits) + padding (4 bits)
  0x05, 0x01,  //   Usage Page (Generic Desktop)
  0x25, 0x07,  //   Logical Maximum (7)
  0x46, 0x3B, 0x01,  // Physical Maximum (315)
  0x75, 0x04,  //   Report Size (4)
  0x95, 0x01,  //   Report Count (1)
  0x65, 0x14,  //   Unit (Eng Rot: Degree)
  0x09, 0x39,  //   Usage (Hat switch)
  0x81, 0x42,  //   Input (Data,Var,Abs,Null)
  0x65, 0x00,  //   Unit (None)
  0x95, 0x01,  //   Report Count (1)
  0x81, 0x01,  //   Input (Const) -- padding
  // Axes LX, LY, RX, RY (4 x 8 bits, 0-255)
  0x26, 0xFF, 0x00,  // Logical Maximum (255)
  0x46, 0xFF, 0x00,  // Physical Maximum (255)
  0x09, 0x30,  //   Usage (X)
  0x09, 0x31,  //   Usage (Y)
  0x09, 0x32,  //   Usage (Z)
  0x09, 0x35,  //   Usage (Rz)
  0x75, 0x08,  //   Report Size (8)
  0x95, 0x04,  //   Report Count (4)
  0x81, 0x02,  //   Input (Data,Var,Abs)
  // Vendor input (1 byte)
  0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined)
  0x09, 0x20,  //   Usage (0x20)
  0x95, 0x01,  //   Report Count (1)
  0x81, 0x02,  //   Input (Data,Var,Abs)
  // Vendor output (8 bytes) -- Switch が要求するため宣言だけ置く
  0x0A, 0x21, 0x26,  // Usage (0x2621)
  0x95, 0x08,  //   Report Count (8)
  0x91, 0x02,  //   Output (Data,Var,Abs)
  0xC0         // End Collection
};

// HID オブジェクトは両モード共用で1個だけ使う。
// begin() の前に、モード側のレポートディスクリプタを設定する。
Adafruit_USBD_HID usb_hid;

// 基板上の RGB LED (WS2812, GP16): モード表示用
#define LED_PIN 16
static Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);
static uint32_t modeLedColor = 0;  // パッド検出後に点けるモード色

// LED をゆっくり明滅させる (起動中・待機中の表示)。
// 周期 1.4 秒の三角波で、完全には消灯しない。
static void ledPulse(uint32_t color) {
  uint32_t t = millis() % 1400;
  float ph = (t < 700) ? (t / 700.0f) : ((1400 - t) / 700.0f);
  float level = 0.08f + 0.92f * ph;
  uint8_t r = (uint8_t)(((color >> 16) & 0xFF) * level);
  uint8_t g = (uint8_t)(((color >> 8) & 0xFF) * level);
  uint8_t b = (uint8_t)((color & 0xFF) * level);
  pixels.setPixelColor(0, pixels.Color(r, g, b));
  pixels.show();
}

// ---- PC モード向け HID (ポッ拳互換 + トリガー軸) ----
// Switch 用ディスクリプタに Rx / Ry (L/R アナログ) を足した
// もの。レポート先頭 8 バイトの並びは Switch 用と同一に保つ。
static const uint8_t desc_hid_report_pc[] = {
  0x05, 0x01,  // Usage Page (Generic Desktop)
  0x09, 0x05,  // Usage (Game Pad)
  0xA1, 0x01,  // Collection (Application)
  // Buttons (16 bits)
  0x15, 0x00,  //   Logical Minimum (0)
  0x25, 0x01,  //   Logical Maximum (1)
  0x35, 0x00,  //   Physical Minimum (0)
  0x45, 0x01,  //   Physical Maximum (1)
  0x75, 0x01,  //   Report Size (1)
  0x95, 0x10,  //   Report Count (16)
  0x05, 0x09,  //   Usage Page (Button)
  0x19, 0x01,  //   Usage Minimum (1)
  0x29, 0x10,  //   Usage Maximum (16)
  0x81, 0x02,  //   Input (Data,Var,Abs)
  // Hat switch (4 bits) + padding (4 bits)
  0x05, 0x01,  //   Usage Page (Generic Desktop)
  0x25, 0x07,  //   Logical Maximum (7)
  0x46, 0x3B, 0x01,  // Physical Maximum (315)
  0x75, 0x04,  //   Report Size (4)
  0x95, 0x01,  //   Report Count (1)
  0x65, 0x14,  //   Unit (Eng Rot: Degree)
  0x09, 0x39,  //   Usage (Hat switch)
  0x81, 0x42,  //   Input (Data,Var,Abs,Null)
  0x65, 0x00,  //   Unit (None)
  0x95, 0x01,  //   Report Count (1)
  0x81, 0x01,  //   Input (Const) -- padding
  // Axes LX, LY, RX, RY + L/R triggers (6 x 8 bits, 0-255)
  0x26, 0xFF, 0x00,  // Logical Maximum (255)
  0x46, 0xFF, 0x00,  // Physical Maximum (255)
  0x09, 0x30,  //   Usage (X)
  0x09, 0x31,  //   Usage (Y)
  0x09, 0x32,  //   Usage (Z)
  0x09, 0x35,  //   Usage (Rz)
  0x09, 0x33,  //   Usage (Rx) -- L trigger
  0x09, 0x34,  //   Usage (Ry) -- R trigger
  0x75, 0x08,  //   Report Size (8)
  0x95, 0x06,  //   Report Count (6)
  0x81, 0x02,  //   Input (Data,Var,Abs)
  // Vendor input (1 byte)
  0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined)
  0x09, 0x20,  //   Usage (0x20)
  0x95, 0x01,  //   Report Count (1)
  0x81, 0x02,  //   Input (Data,Var,Abs)
  // Vendor output (8 bytes) -- Switch が要求するため宣言だけ置く
  0x0A, 0x21, 0x26,  // Usage (0x2621)
  0x95, 0x08,  //   Report Count (8)
  0x91, 0x02,  //   Output (Data,Var,Abs)
  0xC0         // End Collection
};

// PC レポート (10 バイト、ID なし)
struct __attribute__((packed)) PcReport {
  uint16_t buttons;
  uint8_t  hat;
  uint8_t  lx, ly, rx, ry;
  uint8_t  tl, tr;  // Rx / Ry の位置に出す L/R アナログ
  uint8_t  vendor;
};
static PcReport pcRep;
static bool pcMode = false;
static uint8_t trigRestL = 0, trigRestR = 0;  // トリガー静止値

// Switch レポート (8 バイト、ID なし)
struct __attribute__((packed)) SwitchReport {
  uint16_t buttons;
  uint8_t  hat;
  uint8_t  lx, ly, rx, ry;
  uint8_t  vendor;
};
static SwitchReport swRep;

// Switch ボタンのビット位置
#define SW_Y       0x0001
#define SW_B       0x0002
#define SW_A       0x0004
#define SW_X       0x0008
#define SW_L       0x0010
#define SW_R       0x0020
#define SW_ZL      0x0040
#define SW_ZR      0x0080
#define SW_MINUS   0x0100
#define SW_PLUS    0x0200
#define SW_L3      0x0400
#define SW_R3      0x0800
#define SW_HOME    0x1000
#define SW_CAPTURE 0x2000

// ---- プロトタイプ ----
static inline void lineLow();
static inline void lineRelease();
static void joybusSendByte(uint8_t b);
static void joybusSendStop();
static bool joybusReadByte(uint8_t &out, uint32_t firstTimeoutUs, bool firstByte);
static bool joybusPoll(uint8_t dataOut[8], bool rumble);

// ------------------------------------------------------------
// ライン制御
// ------------------------------------------------------------
static inline void lineLow()     { gpio_set_dir(GC_DATA_PIN, GPIO_OUT); }
static inline void lineRelease() { gpio_set_dir(GC_DATA_PIN, GPIO_IN); }
static inline bool lineLevel()   { return gpio_get(GC_DATA_PIN); }

static void joybusSendByte(uint8_t b) {
  for (int i = 7; i >= 0; i--) {
    bool bit = (b >> i) & 1;
    lineLow();
    delayMicroseconds(bit ? 1 : 3);
    lineRelease();
    delayMicroseconds(bit ? 3 : 1);
  }
}

static void joybusSendStop() {
  lineLow();
  delayMicroseconds(2);
  lineRelease();
  delayMicroseconds(2);
}

static bool readBits(uint8_t &out, int startBit, uint32_t firstTimeoutUs) {
  for (int i = startBit; i >= 0; i--) {
    uint32_t start = micros();
    uint32_t timeout = (i == startBit) ? firstTimeoutUs : 60;
    while (lineLevel()) {
      if (micros() - start > timeout) return false;
    }
    delayMicroseconds(2);
    if (lineLevel()) out |= (1 << i);
    if (i > 0) {
      start = micros();
      while (!lineLevel()) {
        if (micros() - start > 60) return false;
      }
    }
  }
  return true;
}

static bool joybusReadByte(uint8_t &out, uint32_t firstTimeoutUs, bool firstByte) {
  out = 0;
  if (!firstByte) {
    uint32_t start = micros();
    while (!lineLevel()) {
      if (micros() - start > 60) return false;
    }
    return readBits(out, 7, firstTimeoutUs);
  }
  // フレーム先頭バイト: 先頭 bit は常に 0。受信開始時点でラインが
  // 既に LOW なら、その bit を 0 で確定させて bit6 から読む。
  if (!lineLevel()) {
    uint32_t start = micros();
    while (!lineLevel()) {
      if (micros() - start > 60) return false;
    }
    return readBits(out, 6, 60);
  }
  return readBits(out, 7, firstTimeoutUs);
}

// 先頭 bit を取りこぼすと全体が 1bit 左にずれて読める。
// 落ちる bit は byte0 の最上位 (= 常に 0) なので右シフトで復元できる。
static void realignRight1(uint8_t *d, int n) {
  uint8_t carry = 0;
  for (int k = 0; k < n; k++) {
    uint8_t nextCarry = d[k] & 1;
    d[k] = (carry << 7) | (d[k] >> 1);
    carry = nextCarry;
  }
}

static int stickDeviation(const uint8_t d[8]) {
  int s = 0;
  for (int i = 2; i <= 5; i++) s += abs((int)d[i] - 128);
  return s;
}


// POLL (0x40, モード3) のレイアウト:
//   byte0: 0 0 0 Start Y X B A
//   byte1: 1 L R Z D-Up D-Down D-Right D-Left
//   byte2: メインスティック X / byte3: Y
//   byte4: Cスティック X / byte5: Y
//   byte6: L アナログ / byte7: R アナログ
static bool joybusPoll(uint8_t dataOut[8], bool rumble) {
  noInterrupts();
  joybusSendByte(0x40);
  joybusSendByte(0x03);
  joybusSendByte(rumble ? 0x01 : 0x00);
  joybusSendStop();
  bool ok = true;
  for (int i = 0; i < 8; i++) {
    if (!joybusReadByte(dataOut[i], i == 0 ? 300 : 60, i == 0)) { ok = false; break; }
  }
  interrupts();
  if (ok && (dataOut[1] & 0x80) == 0) {
    realignRight1(dataOut, 8);
    if ((dataOut[1] & 0x80) == 0) return false;
  } else if (ok && dataOut[2] < 24 && dataOut[4] < 24) {
    uint8_t tmp[8];
    memcpy(tmp, dataOut, 8);
    realignRight1(tmp, 8);
    if ((tmp[1] & 0x80) && stickDeviation(tmp) < stickDeviation(dataOut)) {
      memcpy(dataOut, tmp, 8);
    }
  }
  dataOut[0] &= 0x1F;  // byte0 上位 3bit は常に 0 の領域
  return ok;
}


// ------------------------------------------------------------
// キャリブレーション: 起動直後の数フレームで中心を測る
// ------------------------------------------------------------
static const uint8_t CALIB_FRAMES = 24;
static bool     calibDone  = false;
static uint8_t  calibCount = 0;
static uint16_t sumX = 0, sumY = 0, sumCX = 0, sumCY = 0;
static uint16_t sumTL = 0, sumTR = 0;
static int      centerLX = 128, centerLY = 128;
static int      centerRX = 128, centerRY = 128;

static void calibrateFeed(const uint8_t d[8]) {
  sumX += d[2]; sumY += d[3]; sumCX += d[4]; sumCY += d[5];
  sumTL += d[6]; sumTR += d[7];
  calibCount++;
  if (calibCount >= CALIB_FRAMES) {
    centerLX = sumX / CALIB_FRAMES;
    centerLY = sumY / CALIB_FRAMES;
    centerRX = sumCX / CALIB_FRAMES;
    centerRY = sumCY / CALIB_FRAMES;
    trigRestL = sumTL / CALIB_FRAMES;
    trigRestR = sumTR / CALIB_FRAMES;
    calibDone = true;
  }
}

// GC 生値を中心基準・デッドゾーン付きで 0..255 (中心 128) に変換
// スティックのベクトルに半径方向のゲインをかける。
// 方向は保持したまま長さだけを伸ばし、最大で円周上
// (半径 127) にクランプする。
static void stickRadial(int *px, int *py, int pct, int radius) {
  float x = (float)*px, y = (float)*py;
  float r = sqrtf(x * x + y * y);
  if (r < 1.0f) return;
  float scaled = r * pct / 100.0f;
  if (scaled > (float)radius) scaled = (float)radius;
  float k = scaled / r;
  *px = (int)lroundf(x * k);
  *py = (int)lroundf(y * k);
}

// 符号付き (-128..127) を 0..255 にクランプして変換する
static uint8_t toU8(int s) {
  s += 128;
  if (s < 0) s = 0;
  if (s > 255) s = 255;
  return (uint8_t)s;
}


// Switch のハット値: 上=0 から時計回りに 0..7、ニュートラル=8
static uint8_t toSwitchHat(const uint8_t d[8], bool suppressDown) {
  bool up    = d[1] & 0x08;
  bool down  = (d[1] & 0x04) && !suppressDown;
  bool right = d[1] & 0x02;
  bool left  = d[1] & 0x01;
  if (up && right)    return 1;
  if (down && right)  return 3;
  if (down && left)   return 5;
  if (up && left)     return 7;
  if (up)    return 0;
  if (right) return 2;
  if (down)  return 4;
  if (left)  return 6;
  return 8;
}

// ------------------------------------------------------------
// マッピング v1 + 同時押しコンボ
// ------------------------------------------------------------
static int profile = 0;  // 0 = 汎用, 1 = スマブラ寄り

static uint16_t buildButtons(const uint8_t d[8], bool *suppressHatDown) {
  static uint32_t startSince = 0;  // Start が押され続けてからの起点
  static uint32_t homeSince  = 0;
  static uint32_t profSince  = 0;
  static bool profLatched = false;

  uint32_t now = millis();
  bool start = d[0] & 0x10;
  // アナログ補助: 既定値 (静止 約35/28、最奥 約230+) に対して
  // 190 以上でも押下扱いにし、クリック手前で発火させる
  bool l = (d[1] & 0x40) || d[6] >= 190;
  bool r = (d[1] & 0x20) || d[7] >= 190;
  bool z = d[1] & 0x10;
  bool dd = d[1] & 0x04;

  uint16_t b = 0;
  if (d[0] & 0x08) b |= SW_Y;
  if (d[0] & 0x02) b |= SW_B;
  if (d[0] & 0x01) b |= SW_A;
  if (d[0] & 0x04) b |= SW_X;
  *suppressHatDown = false;

  bool homeCombo = l && r && start;
  bool profCombo = l && r && z && !start;

  // プロファイル切替: L+R+Z 約1秒長押し (仮の操作)
  if (profCombo) {
    if (profSince == 0) profSince = now;
    if (!profLatched && now - profSince >= 1000) {
      profile = 1 - profile;
      profLatched = true;
    }
  } else {
    profSince = 0;
    profLatched = false;
  }

  // Home: L+R+Start 約1秒長押し
  bool homeActive = false;
  if (homeCombo) {
    if (homeSince == 0) homeSince = now;
    if (now - homeSince >= 1000) homeActive = true;
  } else {
    homeSince = 0;
  }

  bool minusCombo   = start && z && !homeCombo;   // Start + Z -> -
  bool captureCombo = start && dd && !homeCombo;  // Start + DD -> Capture
  *suppressHatDown = captureCombo;

  // 肩ボタン (コンボ成立中は抑制)
  if (!homeCombo && !profCombo) {
    if (profile == 0) {
      if (l) b |= SW_L;
      if (r) b |= SW_R;
      if (z && !minusCombo) b |= SW_ZR;
    } else {
      if (l) b |= SW_ZL;
      if (r) b |= SW_ZR;
      if (z && !minusCombo) b |= SW_R;
    }
  }

  if (minusCombo)   b |= SW_MINUS;
  if (captureCombo) b |= SW_CAPTURE;
  if (homeActive)   b |= SW_HOME;

  // Start 単独 -> + は約100ms遅れて発火 (コンボ判定待ち)
  if (start) {
    if (startSince == 0) startSince = now;
    if (!minusCombo && !captureCombo && !homeCombo && now - startSince >= 100) {
      b |= SW_PLUS;
    }
  } else {
    startSince = 0;
  }

  return b;
}

// ------------------------------------------------------------
// PC モード用変換
// ------------------------------------------------------------
static int8_t axisFromRawPC(uint8_t raw, int center, bool invert) {
  int v = (int)raw - center;
  if (v >= -6 && v <= 6) {
    v = 0;
  } else if (v > 0) {
    int range = 255 - center;
    v = v * 127 / (range > 0 ? range : 127);
  } else {
    int range = center;
    v = v * 127 / (range > 0 ? range : 127);
  }
  if (invert) v = -v;
  if (v > 127) v = 127;
  if (v < -127) v = -127;
  return (int8_t)v;
}

// L/R トリガーを 0..255 に変換する (静止値でキャリブレーション)
static uint8_t triggerToU8(uint8_t raw, uint8_t rest) {
  if (raw <= rest) return 0;
  int v = (raw - rest) * 255 / (255 - rest);
  if (v > 255) v = 255;
  return (uint8_t)v;
}

// 起動時モード判定: USB 構成前に GC を読み、Start が押されて
// いたら PC モード。読めなければ Switch モード (既定)。
static bool detectPcMode() {
  // パッドの起動待ち (話しかける前に 500ms 待つ)。
  // この間は LED を白く明滅させて起動中らしく見せる。
  for (int i = 0; i < 50; i++) {
    ledPulse(0xFFFFFF);
    delay(10);
  }
  uint8_t d[8];
  uint32_t t0 = millis();
  while (millis() - t0 < 500) {
    if (joybusPoll(d, false)) {
      return (d[0] & 0x10) != 0;  // Start ビット
    }
    ledPulse(0xFFFFFF);
    delay(10);
  }
  return false;
}

// ------------------------------------------------------------
// setup / loop
// ------------------------------------------------------------
void setup() {

  // 判定と構成が終わるまでホストに見せない
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }
  TinyUSBDevice.detach();

  pixels.begin();
  pixels.setBrightness(30);
  pixels.show();

  // GC DATA ピンの初期化: 出力ラッチを先に LOW へ固定し、
  // 以後は方向の切り替えだけでバスを駆動する (オープンドレイン)
  gpio_init(GC_DATA_PIN);
  gpio_set_pulls(GC_DATA_PIN, false, false);
  gpio_put(GC_DATA_PIN, false);
  lineRelease();

  // モード判定: 起動時に GC を読む (Start 押下中なら PC モード)
  pcMode = detectPcMode();

  if (pcMode) {
    // PC モード: 名乗りは Switch モードと同じポッ拳互換で、
    // ディスクリプタにトリガー軸を追加したものを使う。
    // シリアルは使わない (両モードとも CDC を落とす)。
    Serial.end();
    USBDevice.setID(0x0F0D, 0x0092);
    USBDevice.setManufacturerDescriptor("HORI CO., LTD.");
    USBDevice.setProductDescriptor("POKKEN CONTROLLER");
    usb_hid.setReportDescriptor(desc_hid_report_pc,
                                sizeof(desc_hid_report_pc));
    modeLedColor = pixels.Color(0, 255, 0);  // 緑: PC
  } else {
    // Switch モード: シリアルを完全に切る
    // (CDC が混ざると Switch に認識されない)
    Serial.end();
    USBDevice.setID(0x0F0D, 0x0092);
    USBDevice.setManufacturerDescriptor("HORI CO., LTD.");
    USBDevice.setProductDescriptor("POKKEN CONTROLLER");
    usb_hid.setReportDescriptor(desc_hid_report,
                                sizeof(desc_hid_report));
    modeLedColor = pixels.Color(0, 0, 255);  // 青: Switch
  }
  usb_hid.setPollInterval(2);
  usb_hid.begin();

  // 最終構成で列挙させる
  TinyUSBDevice.attach();

  // 列挙完了を待つ (最大2秒。完了しなくてもループに進み、
  // ニュートラル送信を始める)
  uint32_t t0 = millis();
  while (!USBDevice.mounted() && millis() - t0 < 2000) {
    ledPulse(modeLedColor);  // 列挙待ちの間も明滅させる
    delay(10);
  }
  delay(500);
}

void loop() {
  static bool gcPresent = false;

  // GC の状態を取得する。取れない間もニュートラルのレポートを
  // 送り続ける (実証済みスケッチは常時送信する作り)。
  // プローブは使わず、直接ポーリングする。
  // ポーリングが通ればパッド有り、500ms 以上途絶えたら無し。
  static uint32_t lastPollOkMs = 0;
  uint8_t d[8];
  bool have = false;
  if (joybusPoll(d, false)) {
    have = true;
    gcPresent = true;
    lastPollOkMs = millis();
  } else if (millis() - lastPollOkMs > 500) {
    gcPresent = false;
  }

  if (have) {
    if (!calibDone) calibrateFeed(d);
  }

  // LED: パッド検出中はモード色で点灯、未検出の間は
  // モード色でゆっくり明滅する (待機中の合図)
  static bool lastPresent = false;
  if (gcPresent) {
    if (!lastPresent) {
      lastPresent = true;
      pixels.setPixelColor(0, modeLedColor);
      pixels.show();
    }
  } else {
    lastPresent = false;
    ledPulse(modeLedColor);
  }

  if (pcMode) {
    // ボタン/ハット/スティックは Switch モードと同じ変換を使い、
    // L/R アナログだけを追加の軸として載せる
    if (have && calibDone) {
      bool suppressDown = false;
      pcRep.buttons = buildButtons(d, &suppressDown);
      pcRep.hat     = toSwitchHat(d, suppressDown);
      int sx = axisFromRawPC(d[2], centerLX, false);
      int sy = axisFromRawPC(d[3], centerLY, GC_Y_INVERT != 0);
      stickRadial(&sx, &sy, GAIN_MAIN_PCT, 128);
      int srx = axisFromRawPC(d[4], centerRX, false);
      int sry = axisFromRawPC(d[5], centerRY, GC_Y_INVERT != 0);
      stickRadial(&srx, &sry, GAIN_C_PCT, 128);
      pcRep.lx = toU8(sx);
      pcRep.ly = toU8(sy);
      pcRep.rx = toU8(srx);
      pcRep.ry = toU8(sry);
      pcRep.tl = triggerToU8(d[6], trigRestL);
      pcRep.tr = triggerToU8(d[7], trigRestR);
      pcRep.vendor = 0;
    } else {
      pcRep.buttons = 0;
      pcRep.hat     = 8;
      pcRep.lx = 128; pcRep.ly = 128; pcRep.rx = 128; pcRep.ry = 128;
      pcRep.tl = 0; pcRep.tr = 0;
      pcRep.vendor = 0;
    }
    if (usb_hid.ready()) usb_hid.sendReport(0, &pcRep, sizeof(pcRep));
  } else {
    if (have && calibDone) {
      bool suppressDown = false;
      swRep.buttons = buildButtons(d, &suppressDown);
      swRep.hat     = toSwitchHat(d, suppressDown);
      int sx = axisFromRawPC(d[2], centerLX, false);
      int sy = axisFromRawPC(d[3], centerLY, GC_Y_INVERT != 0);
      stickRadial(&sx, &sy, GAIN_MAIN_PCT, 128);
      int srx = axisFromRawPC(d[4], centerRX, false);
      int sry = axisFromRawPC(d[5], centerRY, GC_Y_INVERT != 0);
      stickRadial(&srx, &sry, GAIN_C_PCT, 128);
      swRep.lx = toU8(sx);
      swRep.ly = toU8(sy);
      swRep.rx = toU8(srx);
      swRep.ry = toU8(sry);
      swRep.vendor = 0;
    } else {
      swRep.buttons = 0;
      swRep.hat     = 8;
      swRep.lx = 128; swRep.ly = 128; swRep.rx = 128; swRep.ry = 128;
      swRep.vendor = 0;
    }
    if (usb_hid.ready()) usb_hid.sendReport(0, &swRep, sizeof(swRep));
  }
  delay(2);
}
