// Raspberry Pi Pico 2 W で Bluetooth ゲームコントローラを直接扱うサンプル。
//
// arduino-pico (earlephilhower) の BluetoothHIDMaster を使う。Pico 2 W の
// CYW43439 はクラシック Bluetooth (BR/EDR) と BLE の両方に対応しているので、
// Mac などのホストを経由せず Pico 自身がコントローラを受けられる。
//   - BR/EDR: 8BitDo SN30 Pro / DualSense / DualShock4 / Switch Pro など
//   - BLE (HOGP): Xbox Series コントローラなど
//
// Arduino IDE の設定 (どちらも必須):
//   ボード: Raspberry Pi Pico 2 W (Raspberry Pi Pico/RP2040/RP2350 by Earle F. Philhower, III)
//   ツール > IP/Bluetooth Stack: "IPv4 + Bluetooth" (または "IPv4/IPv6 + Bluetooth")
// Bluetooth 入りのスタックを選ばないとビルド時にエラーになる。
//
// スティック入力は DRV8833 x2 (モーター 4 個) の PWM に割り当てている。
// 配線は GP2 から順に、GP2/GP3 = #1 Motor A、GP4/GP5 = #1 Motor B、
// GP6/GP7 = #2 Motor A、GP8/GP9 = #2 Motor B。
//
// セットアップ手順は README.md の「Pico 2 W + BluetoothHIDMaster」を参照。

#include <BluetoothHIDMaster.h>
#include <BluetoothLock.h>   // btstack を直接叩くときのロック (core 同梱)

// ---------------------------------------------------------------------------
// 設定
// ---------------------------------------------------------------------------

// true: BLE (HOGP) で接続する / false: クラシック Bluetooth (BR/EDR) で接続する。
// 8BitDo・DualSense・Switch Pro は BR/EDR なので false のまま。
// Xbox Series コントローラのような BLE のみのモデルは true にする。
static const bool USE_BLE = false;

// BLE モードのときに Pico 側が名乗る名前。
static const char *BLE_MASTER_NAME = "PICO2W";

// 接続先の MAC アドレスを決め打ちする (BR/EDR のみ)。
// ペアリング済みのコントローラは電源を入れても inquiry には応答せず page scan
// しかしないので、スキャンでは見つからない。アドレスが分かっているなら直接
// hid_host_connect() を投げた方が確実に繋がる。
// 全ゼロにするとこの経路を使わず、スキャン結果からの接続だけになる。
static uint8_t TARGET_ADDR[6] = {0, 0, 0, 0, 0, 0};
// static uint8_t TARGET_ADDR[6] = {0xe4, 0x17, 0xd8, 0xc5, 0x7b, 0x06};

// スキャン結果の絞り込み。
//   0 = ジョイスティック / ゲームパッドだけ (既定)
//   1 = Peripheral 全般 — キーボードやマウスも含む
//   2 = 見つかったもの全部 — スマートフォンなども含むので接続先を間違えやすい
//
// ライブラリの hid.scan(mask, ...) は使わない。あのフィルタは
// (mask & cod) == mask、つまりマスクのビットがすべて立っている機器しか拾わず、
// joystick_cod (0x2508) に含まれる 0x2000 = Limited Discoverable Mode の
// サービスビットのせいで、General Discoverable で広告するコントローラ
// (cod 0x000508 など) まで落としてしまう。かといってマスクを外すと今度は
// スマートフォンにまで繋ぎに行く。Class of Device の Major/Minor を自前で
// 見れば、Limited Discoverable かどうかに関係なく種別だけで判定できる。
static const int DEVICE_FILTER = 0;

// 空文字列でなければ、機器名にこの文字列を含むものだけを対象にする
// (部分一致・大文字小文字は無視)。例: "8BitDo"
// 同じ種別の機器が複数ある場合の絞り込み用。
static const char *TARGET_NAME = "";

// スティックの生値のレンジ。多くのコントローラは 8bit unsigned (0..255, 中央 128)。
// -127..127 のような符号付きで報告する機種は PRINT_RAW_AXES で確認して書き換える。
static const int AXIS_RAW_MIN = 0;
static const int AXIS_RAW_MAX = 255;

// 正規化後 (-1.0..1.0) のデッドゾーン。中央付近のふらつきを 0 に落とす。
static const float AXIS_DEADZONE = 0.12f;

// スティックを倒しっぱなしのときのログ抑制。この間隔より短い連続出力はしない。
static const unsigned long REPORT_INTERVAL_MS = 100;

// 前回出力からこの値以上動いたときだけスティックの状態を出す。
static const float AXIS_REPORT_DELTA = 0.05f;

// true にすると HID の生値をそのまま出力する。AXIS_RAW_MIN/MAX の確認用。
static const bool PRINT_RAW_AXES = false;

// true: blue_test.ino と同じ形式で操作ログを出す。
//   [button] #3 DOWN         ボタン番号は HID の Button usage と同じ 1 始まり
//   [buttons] 0x00000004     押されているボタンのビットマップ
//   [hat] RIGHT (0x02)
//   [axes] LX= 128 LY= 128 RX= 128 RY= 128    生値のまま
static const bool PRINT_INPUT_LOG = true;

// [axes] を出すしきい値 (生値)。スティックはニュートラルでも値が細かく揺れる
// ので、この幅未満の変化はログに出さない。0 にすると生の値がそのまま流れる。
static const int AXIS_LOG_THRESHOLD = 8;

// true: 正規化後のログ (dpad ... / stick L(+0.00, +0.00) R(...)) も出す。
// PRINT_INPUT_LOG と同時に有効にすると 1 回の操作で 2 種類の行が出る。
static const bool PRINT_NORMALIZED_LOG = false;

// コントローラ未接続のときに状況を知らせる間隔。
static const unsigned long WAITING_LOG_INTERVAL_MS = 5000;

// ペアリング済みのコントローラは電源を入れると自分から繋ぎに来る。
// スキャンを始める前にこの時間だけ着信を待つ。
static const unsigned long INCOMING_WAIT_MS = 3000;

// BR/EDR のスキャン時間 (秒)。
static const int SCAN_TIME_SEC = 5;

// 接続要求を出してから HID ディスクリプタが届くまでの待ち時間。
// ライブラリの connectJoystick() はここが無限待ちなので自前で接続している。
//
// btstack は hid_host_connect() の直後に SDP クエリで HID ディスクリプタを
// 取りに行く。相手の応答が遅いと SDP の L2CAP タイムアウト (20 秒以上) まで
// 何のイベントも上がってこないので、短く打ち切ると失敗理由が分からないうえ、
// SDP クライアントが使用中のまま残って次回以降の接続まで巻き添えになる。
// 途中で切らずに btstack の状態遷移を最後まで走らせること。
static const unsigned long CONNECT_TIMEOUT_MS = 30000;

// BR/EDR で L2CAP チャネル (SDP を含む) に要求するセキュリティレベル。
//   0 = 認証なし / 2 = 暗号化を要求 (btstack の既定)
// btstack は SDP クエリの前にこのレベルを満たそうとするため、ペアリングが
// 成立しないと SDP のイベントが一切上がらないまま固まる。
// 「stalled in the SDP query」が出る場合は 0 にして認証を要求しないようにする。
static const int BREDR_SECURITY_LEVEL = 0;

// true: hid_host が 0x11 (SDP に HID レコードがない) で失敗した相手に、
// SDP を使わず HID の既定 PSM (Control 0x11 / Interrupt 0x13) へ直接つなぐ。
// DualShock 4 互換品には SDP にレコードを一切出さないものがある。
// ディスクリプタが取れないので、入力は DS4 のレポート形式として自前で解析する。
static const bool DIRECT_L2CAP_FALLBACK = true;

// 直接接続のときに要求するセキュリティレベル。
//   2 = 暗号化 (ペアリングする) / 0 = 認証なし
// DS4 は未ペアリングのホストからの HID チャネルを受け付けないことが多いので 2。
// "[direct] control channel failed" が出る場合は 0 も試す。
static const int DIRECT_SECURITY_LEVEL = 2;

// true にすると起動時にペアリング情報を消し、必ずペアリングからやり直す。
// false なら一度ペアリングしたコントローラは電源を入れるだけで再接続する。
// シリアルモニタから 'f' を送っても消去できる。
static const bool FORGET_KEYS_ON_BOOT = false;

// --- DRV8833 x2 (モーター 4 個) の配線。GP2 から順に割り当てる ---
// DRV8833 #1 Motor A: AIN1 -> GP2, AIN2 -> GP3
// DRV8833 #1 Motor B: BIN1 -> GP4, BIN2 -> GP5
static const uint8_t DRV1_MOTOR_A_IN1_PIN = 2;
static const uint8_t DRV1_MOTOR_A_IN2_PIN = 3;
static const uint8_t DRV1_MOTOR_B_IN1_PIN = 4;
static const uint8_t DRV1_MOTOR_B_IN2_PIN = 5;

// DRV8833 #2 Motor A: AIN1 -> GP6, AIN2 -> GP7
// DRV8833 #2 Motor B: BIN1 -> GP8, BIN2 -> GP9
static const uint8_t DRV2_MOTOR_A_IN1_PIN = 6;
static const uint8_t DRV2_MOTOR_A_IN2_PIN = 7;
static const uint8_t DRV2_MOTOR_B_IN1_PIN = 8;
static const uint8_t DRV2_MOTOR_B_IN2_PIN = 9;

// PWM 設定。20kHz は可聴域外なのでモーターが鳴きにくい。
static const uint32_t PWM_FREQUENCY_HZ = 20000;
static const uint32_t PWM_RANGE = 1023;

// 起動トルクが足りないモーター向けの下限デューティ (0.0..1.0)。
// 0.0 ならそのまま。0.25 などにすると弱い入力でも最低このデューティで回る。
static const float MOTOR_MIN_DUTY = 0.0f;

// スティックは上方向が負なので、true にすると「上に倒す = 正転」になる。
static const bool INVERT_STICK_Y = true;

// true でモーターの回転方向/デューティの変化をシリアルに出力する。
static const bool PRINT_MOTOR_STATE = true;

// オンボード LED の点滅周期 (ms)。コントローラ接続中 (未接続で探している間) は
// この周期で点滅させ、接続が確立したら点灯しっぱなしにする。
static const unsigned long LED_BLINK_INTERVAL_MS = 250;

// ---------------------------------------------------------------------------
// 状態
// ---------------------------------------------------------------------------

BluetoothHIDMaster hid;

// BluetoothHIDMaster のコールバックは BTstack の割り込み文脈から呼ばれる。
// ここでは値を退避するだけにして、実際の処理は loop() 側で行う。
struct GamepadReport {
    int x, y, z, rz;
    uint8_t hat;
    uint32_t buttons;
};

static GamepadReport sharedReport;
static volatile bool reportPending = false;

// 変化検出用の前回値。
static uint32_t lastButtons = 0;
static uint8_t lastHat = 0x0f;
static float lastLX = 0.0f, lastLY = 0.0f, lastRX = 0.0f, lastRY = 0.0f;
static unsigned long lastAxisReportMs = 0;

// PRINT_INPUT_LOG 用の前回値。生値のまま比較するので上の正規化後の値とは別に持つ。
static GamepadReport lastLoggedRaw = { 0, 0, 0, 0, 0x0f, 0 };
static bool firstInputLog = true;
static bool wasConnected = false;

// 自前で開始した HID 接続の cid。ディスクリプタが来ないまま張りっぱなしに
// なったリンクを閉じるために保持する。
static uint16_t hidCid = 0;
static unsigned long lastWaitingLogMs = 0;

// BR/EDR の ACL 接続ハンドル。HID のセットアップに失敗したリンクは
// hid_host_disconnect() では解放されないことがあるので、最後は ACL ごと切る。
// HCI イベントハンドラ (割り込み文脈) から更新するので volatile。
static volatile hci_con_handle_t aclHandle = HCI_CON_HANDLE_INVALID;
static volatile bool aclUpNotice = false;
static volatile bool aclDownNotice = false;
static btstack_packet_callback_registration_t hciEventRegistration;

// BluetoothHIDMaster 内蔵の HID ホストハンドラは
//   - HID ディスクリプタ用バッファが 300 バイト固定 (足りないと接続が完了しない)
//   - 接続失敗の status を外から見られない
// ので、BR/EDR ではスタックの起動だけライブラリに任せ、HID ホストは自前で扱う。
// (BLE を使う場合はライブラリ側の実装をそのまま使う)
static uint8_t hidDescriptorStorage[1024];
static volatile bool hidConnected = false;
static volatile bool hidDescriptorReady = false;
static char connectedAddrText[18] = "";

// HID_SUBEVENT_CONNECTION_OPENED で失敗が返ったときの status。0 なら失敗なし。
// 失敗が分かった時点で待機を打ち切るのに使う。
static volatile uint8_t hidConnectFailStatus = 0;

// 直接 L2CAP 接続 (DIRECT_L2CAP_FALLBACK) の状態。
static volatile uint16_t directControlCid = 0;
static volatile uint16_t directInterruptCid = 0;
static volatile bool directConnected = false;
static volatile uint8_t directFailStatus = 0;
static bd_addr_t directAddr;
static volatile bool unknownReportLogged = false;

// hid_host が 0x11 で失敗したアドレス。次回からは最初から直接接続する。
static bool noSdpAddrValid = false;
static uint8_t noSdpAddr[6];

// SDP ダンプ (診断用) の状態。
static volatile bool sdpDumpRunning = false;
static uint8_t sdpAttrValue[48];

// ---------------------------------------------------------------------------
// ユーティリティ
// ---------------------------------------------------------------------------

// 生値を -1.0..1.0 に正規化し、デッドゾーンを適用する。
// デッドゾーンの外は 0.0 から連続的に増えるようリスケールする。
static float normalizeAxis(int raw) {
    const float center = (AXIS_RAW_MIN + AXIS_RAW_MAX) / 2.0f;
    const float half = (AXIS_RAW_MAX - AXIS_RAW_MIN) / 2.0f;
    float value = (raw - center) / half;
    if (value > 1.0f) {
        value = 1.0f;
    } else if (value < -1.0f) {
        value = -1.0f;
    }
    const float magnitude = fabsf(value);
    if (magnitude < AXIS_DEADZONE) {
        return 0.0f;
    }
    const float scaled = (magnitude - AXIS_DEADZONE) / (1.0f - AXIS_DEADZONE);
    return (value < 0.0f) ? -scaled : scaled;
}

// HID の hat 値 (0..7 が方向、8 以上は中立) を dx/dy に変換する。上が -1。
static void hatToDirection(uint8_t hat, int *dx, int *dy) {
    static const int8_t table[8][2] = {
        { 0, -1}, { 1, -1}, { 1,  0}, { 1,  1},
        { 0,  1}, {-1,  1}, {-1,  0}, {-1, -1},
    };
    if (hat > 7) {
        *dx = 0;
        *dy = 0;
        return;
    }
    *dx = table[hat][0];
    *dy = table[hat][1];
}

static const char *hatName(uint8_t hat) {
    static const char *names[8] = { "UP", "UP-RIGHT", "RIGHT", "DOWN-RIGHT",
                                    "DOWN", "DOWN-LEFT", "LEFT", "UP-LEFT" };
    return (hat > 7) ? "CENTER" : names[hat];
}

// ---------------------------------------------------------------------------
// オンボード LED
// ---------------------------------------------------------------------------

// 接続を探している間 (isControllerConnected() が false の間) は点滅、
// 接続が確立したら点灯しっぱなしにする。接続待ちのブロッキングループの中でも
// 呼べるように、内部状態を持たず millis() だけで点滅を決める。
static void updateConnectionLed() {
    if (isControllerConnected()) {
        digitalWrite(LED_BUILTIN, HIGH);
        return;
    }
    const bool on = (millis() / LED_BLINK_INTERVAL_MS) % 2 == 0;
    digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
}

// ---------------------------------------------------------------------------
// DRV8833 モーター制御
// ---------------------------------------------------------------------------

static float clampf(float value, float lo, float hi) {
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

// DRV8833 の 1 チャンネル (IN1/IN2 の 2 ピンで 1 モーター) を扱う。
// 片側に PWM、もう片側を LOW にする fast decay 方式で正転/逆転する。
class Drv8833Motor {
public:
    void begin(uint8_t in1Pin, uint8_t in2Pin, const char *name) {
        _in1 = in1Pin;
        _in2 = in2Pin;
        _name = name;
        pinMode(_in1, OUTPUT);
        pinMode(_in2, OUTPUT);
        stop();
    }

    void stop() {
        drive(0, 0, "stop", 0.0f);
    }

    // speed: -1.0 (逆転) .. 0.0 (停止) .. +1.0 (正転)
    void setSpeed(float speed) {
        const float s = clampf(speed, -1.0f, 1.0f);
        if (s == 0.0f) {
            stop();
            return;
        }

        float magnitude = fabsf(s);
        if (magnitude < MOTOR_MIN_DUTY) {
            magnitude = MOTOR_MIN_DUTY;
        }
        int duty = (int)lroundf(magnitude * PWM_RANGE);
        if (duty > (int)PWM_RANGE) {
            duty = (int)PWM_RANGE;
        }

        if (s > 0.0f) {
            drive(duty, 0, "forward", s);
        } else {
            drive(0, duty, "reverse", s);
        }
    }

private:
    void drive(int duty1, int duty2, const char *direction, float speed) {
        analogWrite(_in1, duty1);
        analogWrite(_in2, duty2);
        logState(direction, duty1 + duty2, speed);
    }

    // 回転方向が変わったとき、またはデューティが 5% 以上変わったときだけ出力する。
    void logState(const char *direction, int duty, float speed) {
        if (!PRINT_MOTOR_STATE) {
            return;
        }
        const int threshold = (int)(PWM_RANGE / 20);
        if (_lastDirection && strcmp(direction, _lastDirection) == 0 &&
                abs(duty - _lastDuty) < threshold) {
            return;
        }
        Serial.printf("%s %-7s duty=%d/%d speed=%+.2f\n",
                      _name, direction, duty, (int)PWM_RANGE, speed);
        _lastDirection = direction;
        _lastDuty = duty;
    }

    uint8_t _in1 = 0;
    uint8_t _in2 = 0;
    const char *_name = "motor";
    const char *_lastDirection = nullptr;
    int _lastDuty = -1;
};

static Drv8833Motor drv1MotorA;
static Drv8833Motor drv1MotorB;
static Drv8833Motor drv2MotorA;
static Drv8833Motor drv2MotorB;

static void setupMotors() {
    // analogWriteFreq()/analogWriteRange() は PWM 出力全体に効く。
    analogWriteFreq(PWM_FREQUENCY_HZ);
    analogWriteRange(PWM_RANGE);
    drv1MotorA.begin(DRV1_MOTOR_A_IN1_PIN, DRV1_MOTOR_A_IN2_PIN, "drv1-A");
    drv1MotorB.begin(DRV1_MOTOR_B_IN1_PIN, DRV1_MOTOR_B_IN2_PIN, "drv1-B");
    drv2MotorA.begin(DRV2_MOTOR_A_IN1_PIN, DRV2_MOTOR_A_IN2_PIN, "drv2-A");
    drv2MotorB.begin(DRV2_MOTOR_B_IN1_PIN, DRV2_MOTOR_B_IN2_PIN, "drv2-B");
}

// 左スティック -> DRV8833 #1、右スティック -> DRV8833 #2。
//   左 X -> #1 Motor A / 左 Y -> #1 Motor B
//   右 X -> #2 Motor A / 右 Y -> #2 Motor B
static void applyOutputs(float leftX, float leftY, float rightX, float rightY) {
    const float ly = INVERT_STICK_Y ? -leftY : leftY;
    const float ry = INVERT_STICK_Y ? -rightY : rightY;
    drv1MotorA.setSpeed(leftX);
    drv1MotorB.setSpeed(ly);
    drv2MotorA.setSpeed(rightX);
    drv2MotorB.setSpeed(ry);
}

// 切断時や停止時にすべてのモーターを止める。
static void stopOutputs() {
    drv1MotorA.stop();
    drv1MotorB.stop();
    drv2MotorA.stop();
    drv2MotorB.stop();
}

// ---------------------------------------------------------------------------
// HID コールバック (割り込み文脈)
// ---------------------------------------------------------------------------

// BLE (HOGP) 用。BR/EDR ではライブラリの HID ホストを使わないので呼ばれない。
// 報告されない軸・ハットは 0 として渡ってくる。ボタンは最大 32 個のビットマスク。
static void onJoystickReport(void *cbData, int x, int y, int z, int rz,
                             uint8_t hat, uint32_t buttons) {
    (void)cbData;
    noInterrupts();
    sharedReport.x = x;
    sharedReport.y = y;
    sharedReport.z = z;
    sharedReport.rz = rz;
    sharedReport.hat = hat;
    sharedReport.buttons = buttons;
    reportPending = true;
    interrupts();
}

// ---------------------------------------------------------------------------
// 接続
// ---------------------------------------------------------------------------

static void resetTrackingState() {
    lastButtons = 0;
    lastHat = 0x0f;
    lastLX = lastLY = lastRX = lastRY = 0.0f;
    lastAxisReportMs = 0;
    lastLoggedRaw = { 0, 0, 0, 0, 0x0f, 0 };
    firstInputLog = true;
    noInterrupts();
    reportPending = false;
    interrupts();
}

// HID 入力レポートを解析して sharedReport に積む。
// usage page 0x01 (Generic Desktop) が軸とハット、0x09 (Button) がボタン。
static void handleInputReport(btstack_hid_parser_t *parser) {
    int x = 0, y = 0, z = 0, rz = 0;
    uint8_t hat = 0x0f;   // 0x0f = 中立。ハットを持たない機種は 0 (=上) にしない
    uint32_t buttons = 0;
    bool updated = false;

    while (btstack_hid_parser_has_more(parser)) {
        uint16_t usagePage;
        uint16_t usage;
        int32_t value;
        btstack_hid_parser_get_field(parser, &usagePage, &usage, &value);

        if (usagePage == 0x01) {
            updated = true;
            switch (usage) {
            case 0x30: x = value; break;            // X
            case 0x31: y = value; break;            // Y
            case 0x32: z = value; break;            // Z  (右スティック X)
            case 0x33: z = value; break;            // Rx (機種によってはこちら)
            case 0x34: rz = value; break;           // Ry (機種によってはこちら)
            case 0x35: rz = value; break;           // Rz (右スティック Y)
            case 0x39: hat = (uint8_t)(value & 0xff); break;  // Hat switch
            default: break;
            }
        } else if (usagePage == 0x09) {
            updated = true;
            if (value && usage > 0) {
                buttons |= 1UL << (usage - 1);
            }
        }
    }

    if (!updated) {
        return;
    }

    sharedReport.x = x;
    sharedReport.y = y;
    sharedReport.z = z;
    sharedReport.rz = rz;
    sharedReport.hat = hat;
    sharedReport.buttons = buttons;
    reportPending = true;
}

// btstack の HID ホストイベントを直接受ける。状態と失敗理由がすべて見える。
static void hidHostEventHandler(uint8_t packetType, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packetType != HCI_EVENT_PACKET || hci_event_packet_get_type(packet) != HCI_EVENT_HID_META) {
        return;
    }

    uint8_t status;
    bd_addr_t addr;

    switch (hci_event_hid_meta_get_subevent_code(packet)) {
    case HID_SUBEVENT_INCOMING_CONNECTION:
        hidCid = hid_subevent_incoming_connection_get_hid_cid(packet);
        hid_subevent_incoming_connection_get_address(packet, addr);
        Serial.printf("[hid] incoming connection from %s: accepting\n", bd_addr_to_str(addr));
        hid_host_accept_connection(hidCid, HID_PROTOCOL_MODE_REPORT);
        break;

    case HID_SUBEVENT_CONNECTION_OPENED:
        status = hid_subevent_connection_opened_get_status(packet);
        if (status != ERROR_CODE_SUCCESS) {
            Serial.printf("[hid] connection failed: status 0x%02x\n", status);
            if (status == ERROR_CODE_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE) {
                // SDP には応答したが HID サービスの PSM が取れなかった
                Serial.println("  (0x11: SDP answered but no usable HID record / PSM)");
            }
            hidConnectFailStatus = status;
            hidConnected = false;
            hidDescriptorReady = false;
            hidCid = 0;
            break;
        }
        hidCid = hid_subevent_connection_opened_get_hid_cid(packet);
        hid_subevent_connection_opened_get_bd_addr(packet, addr);
        strncpy(connectedAddrText, bd_addr_to_str(addr), sizeof(connectedAddrText) - 1);
        connectedAddrText[sizeof(connectedAddrText) - 1] = 0;
        hidConnected = true;
        hidDescriptorReady = false;
        Serial.printf("[hid] connection opened (cid=0x%04x, %s)\n", hidCid, connectedAddrText);
        break;

    case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
        status = hid_subevent_descriptor_available_get_status(packet);
        if (status == ERROR_CODE_SUCCESS) {
            hidDescriptorReady = true;
            Serial.printf("[hid] descriptor available (%u bytes)\n",
                          (unsigned)hid_descriptor_storage_get_descriptor_len(hidCid));
        } else {
            // 0x07 = メモリ不足 (hidDescriptorStorage を増やす)
            // 0x11 = 相手が HID ディスクリプタを持っていない
            Serial.printf("[hid] descriptor NOT available: status 0x%02x\n", status);
        }
        break;

    case HID_SUBEVENT_REPORT: {
        if (!hidDescriptorReady) {
            break;
        }
        uint16_t reportLen = hid_subevent_report_get_report_len(packet);
        const uint8_t *report = hid_subevent_report_get_report(packet);
        if ((reportLen < 1) || (*report != 0xa1)) {   // 0xa1 = DATA/Input
            break;
        }
        report++;
        reportLen--;
        btstack_hid_parser_t parser;
        btstack_hid_parser_init(&parser,
                                hid_descriptor_storage_get_descriptor_data(hidCid),
                                hid_descriptor_storage_get_descriptor_len(hidCid),
                                HID_REPORT_TYPE_INPUT, report, reportLen);
        handleInputReport(&parser);
        break;
    }

    case HID_SUBEVENT_CONNECTION_CLOSED:
        Serial.println("[hid] connection closed");
        hidConnected = false;
        hidDescriptorReady = false;
        hidCid = 0;
        break;

    default:
        break;
    }
}

// ACL リンクの上げ下げを監視して接続ハンドルを覚えておく。
// hid_host のイベントはライブラリが握っているので、こちらは HCI 層を見る。
static void hciEventHandler(uint8_t packetType, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packetType != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_CONNECTION_COMPLETE:
        if (hci_event_connection_complete_get_status(packet) == ERROR_CODE_SUCCESS) {
            aclHandle = hci_event_connection_complete_get_connection_handle(packet);
            aclUpNotice = true;
        }
        break;
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        if (hci_event_disconnection_complete_get_connection_handle(packet) == aclHandle) {
            aclHandle = HCI_CON_HANDLE_INVALID;
            aclDownNotice = true;
        }
        break;
    default:
        break;
    }
}

// 割り込み文脈で溜めたリンク状態のログを吐く。
static void pumpLinkLog() {
    if (aclUpNotice) {
        aclUpNotice = false;
        Serial.println("  [link] ACL connected (waiting for the HID descriptor)");
    }
    if (aclDownNotice) {
        aclDownNotice = false;
        Serial.println("  [link] ACL disconnected");
    }
}

// BR/EDR は自前の HID ホスト状態、BLE はライブラリ側の状態を見る。
static bool isControllerConnected() {
    if (USE_BLE) {
        return hid.connected();
    }
    return (hidConnected && hidDescriptorReady) || directConnected;
}

// 接続完了まで待つ。必ず timeoutMs 以内に戻る。
// btstack から接続失敗が通知されたら、その時点で打ち切る。
static bool waitUntilConnected(unsigned long timeoutMs) {
    const unsigned long startedAt = millis();
    while (millis() - startedAt < timeoutMs) {
        pumpLinkLog();
        updateConnectionLed();
        if (isControllerConnected()) {
            return true;
        }
        if (hidConnectFailStatus || directFailStatus) {
            break;
        }
        delay(20);
    }
    pumpLinkLog();
    return false;
}

// ディスクリプタが来ないまま開いたリンクを閉じる。
// hid.disconnect() は connected() が false だと何もしないうえ、
// btstack の hid_host_disconnect() もセットアップ途中の接続は解放しきれない。
// 解放されないと次回以降 hid_host_connect() が 0x0c
// (ERROR_CODE_COMMAND_DISALLOWED) を返し続けるので、最後は ACL ごと落とす。
static void forceDisconnect() {
    if (hidCid) {
        BluetoothLock lock;
        hid_host_disconnect(hidCid);
    }
    hidCid = 0;

    if (aclHandle != HCI_CON_HANDLE_INVALID) {
        BluetoothLock lock;
        gap_disconnect(aclHandle);
    }

    // 切断完了 (HCI_EVENT_DISCONNECTION_COMPLETE) を待つ。
    const unsigned long startedAt = millis();
    while (aclHandle != HCI_CON_HANDLE_INVALID && millis() - startedAt < 2000) {
        updateConnectionLed();
        delay(20);
    }
    pumpLinkLog();
}

// --- 直接 L2CAP 接続 (DS4 互換品など SDP を持たない機器向け) -------------

// DS4 の入力レポートを GamepadReport に詰める。
//   0x01: 短い形式。[LX][LY][RX][RY][btn0][btn1][btn2][L2][R2]...
//   0x11: 完全な形式。先頭 2 バイトのヘッダの後ろが 0x01 と同じ並び。
// btn0 の下位 4 ビットがハット (8 = 中立)、上位 4 ビットが □ × ○ △。
// btn1 は L1 R1 L2 R2 SHARE OPTIONS L3 R3、btn2 の bit0 が PS、bit1 がタッチパッド。
// ボタン番号は USB 接続時の DS4 の HID Button usage と同じ並び
// (#1 □, #2 ×, #3 ○, #4 △, #5 L1, ... #13 PS, #14 タッチパッド) になる。
static void handleDs4Report(const uint8_t *packet, uint16_t size) {
    // packet[0] = 0xa1 (DATA | Input)、packet[1] = レポート ID
    if (size < 2 || packet[0] != 0xa1) {
        return;
    }
    const uint8_t reportId = packet[1];
    const uint8_t *p = nullptr;
    uint16_t len = 0;
    if (reportId == 0x01) {
        p = packet + 2;
        len = size - 2;
    } else if (reportId == 0x11 && size >= 4) {
        p = packet + 4;
        len = size - 4;
    }
    if (!p || len < 7) {
        if (!unknownReportLogged) {
            unknownReportLogged = true;
            Serial.printf("[direct] unknown report id 0x%02x (%u bytes):", reportId, size);
            for (uint16_t i = 0; i < size && i < 24; i++) {
                Serial.printf(" %02x", packet[i]);
            }
            Serial.println();
        }
        return;
    }
    sharedReport.x = p[0];
    sharedReport.y = p[1];
    sharedReport.z = p[2];
    sharedReport.rz = p[3];
    sharedReport.hat = p[4] & 0x0f;
    sharedReport.buttons = ((uint32_t)(p[4] >> 4)) |
                           ((uint32_t)p[5] << 4) |
                           ((uint32_t)(p[6] & 0x03) << 12);
    reportPending = true;
}

static void directFail(uint8_t status) {
    directFailStatus = status ? status : ERROR_CODE_UNSPECIFIED_ERROR;
    if (directControlCid) {
        l2cap_disconnect(directControlCid);
    }
    if (directInterruptCid) {
        l2cap_disconnect(directInterruptCid);
    }
    directControlCid = 0;
    directInterruptCid = 0;
    directConnected = false;
}

// 直接接続した L2CAP チャネルのイベントとデータ (割り込み文脈)。
static void directL2capHandler(uint8_t packetType, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packetType == L2CAP_DATA_PACKET) {
        if (directConnected && channel == directInterruptCid) {
            handleDs4Report(packet, size);
        }
        return;
    }
    if (packetType != HCI_EVENT_PACKET) {
        return;
    }

    switch (hci_event_packet_get_type(packet)) {
    case L2CAP_EVENT_CHANNEL_OPENED: {
        const uint8_t status = l2cap_event_channel_opened_get_status(packet);
        const uint16_t cid = l2cap_event_channel_opened_get_local_cid(packet);
        const bool isControl = (cid == directControlCid);
        if (!isControl && cid != directInterruptCid) {
            break;
        }
        if (status != ERROR_CODE_SUCCESS) {
            Serial.printf("[direct] %s channel failed: status 0x%02x\n",
                          isControl ? "control" : "interrupt", status);
            if (isControl) {
                directControlCid = 0;
            } else {
                directInterruptCid = 0;
            }
            directFail(status);
            break;
        }
        if (isControl) {
            Serial.println("[direct] control channel (PSM 0x11) opened");
            uint16_t interruptCid = 0;
            const uint8_t st = l2cap_create_channel(&directL2capHandler, directAddr,
                                                    BLUETOOTH_PSM_HID_INTERRUPT, 0xffff, &interruptCid);
            if (st != ERROR_CODE_SUCCESS) {
                Serial.printf("[direct] interrupt channel request rejected (0x%02x)\n", st);
                directFail(st);
                break;
            }
            directInterruptCid = interruptCid;
        } else {
            Serial.println("[direct] interrupt channel (PSM 0x13) opened");
            strncpy(connectedAddrText, bd_addr_to_str(directAddr), sizeof(connectedAddrText) - 1);
            connectedAddrText[sizeof(connectedAddrText) - 1] = 0;
            unknownReportLogged = false;
            directConnected = true;
        }
        break;
    }

    case L2CAP_EVENT_CHANNEL_CLOSED: {
        const uint16_t cid = l2cap_event_channel_closed_get_local_cid(packet);
        if (cid == directControlCid) {
            directControlCid = 0;
        } else if (cid == directInterruptCid) {
            directInterruptCid = 0;
        } else {
            break;
        }
        if (directConnected) {
            Serial.println("[direct] channel closed");
        }
        directFail(ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION);
        break;
    }

    default:
        break;
    }
}

// SDP を使わず PSM 0x11 / 0x13 に直接つなぐ。
static bool connectDirect(const uint8_t *address) {
    memcpy(directAddr, address, sizeof(directAddr));
    directFailStatus = 0;
    directConnected = false;
    Serial.printf("  connecting directly to the HID PSMs (no SDP, security level %d)...\n",
                  DIRECT_SECURITY_LEVEL);

    uint8_t status;
    {
        BluetoothLock lock;
        // l2cap_create_channel() は呼び出し時点の gap_get_security_level() を
        // チャネルの要求レベルにするので、この呼び出しの間だけ切り替える。
        const gap_security_level_t saved = gap_get_security_level();
        gap_set_security_level((gap_security_level_t)DIRECT_SECURITY_LEVEL);
        uint16_t controlCid = 0;
        status = l2cap_create_channel(&directL2capHandler, directAddr,
                                      BLUETOOTH_PSM_HID_CONTROL, 0xffff, &controlCid);
        directControlCid = (status == ERROR_CODE_SUCCESS) ? controlCid : 0;
        gap_set_security_level(saved);
    }
    if (status != ERROR_CODE_SUCCESS) {
        Serial.printf("  direct connect request rejected (0x%02x)\n", status);
        return false;
    }

    if (waitUntilConnected(CONNECT_TIMEOUT_MS)) {
        return true;
    }
    if (!directFailStatus) {
        Serial.println("  timed out opening the HID channels");
    }
    {
        BluetoothLock lock;
        directFail(directFailStatus);
    }
    directFailStatus = 0;
    forceDisconnect();
    delay(1000);
    return false;
}

// --- SDP ダンプ (診断用) ------------------------------------------------
// 0x11 で失敗したとき、相手が SDP で何を公開しているかを出す。
// L2CAP (UUID 0x0100) で検索するので、HID 以外も含めた全レコードが対象。
//   0x0001 ServiceClassIDList  (HID なら UUID 0x1124 を含む)
//   0x0004 ProtocolDescriptorList          (HID Control PSM 0x0011)
//   0x000d AdditionalProtocolDescriptorLists (HID Interrupt PSM 0x0013)
//   0x0206 HIDDescriptorList (長さだけ出す)
static void sdpDumpHandler(uint8_t packetType, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel;
    (void)size;
    if (packetType != HCI_EVENT_PACKET) {
        return;
    }
    switch (hci_event_packet_get_type(packet)) {
    case SDP_EVENT_QUERY_ATTRIBUTE_BYTE: {
        const uint16_t id = sdp_event_query_attribute_byte_get_attribute_id(packet);
        const uint16_t len = sdp_event_query_attribute_byte_get_attribute_length(packet);
        const uint16_t off = sdp_event_query_attribute_byte_get_data_offset(packet);
        if (off < sizeof(sdpAttrValue)) {
            sdpAttrValue[off] = sdp_event_query_attribute_byte_get_data(packet);
        }
        if (off + 1 != len) {
            break;
        }
        if (id == 0x0000) {
            Serial.printf("  [sdp] record #%u\n", sdp_event_query_attribute_byte_get_record_id(packet));
        }
        if (id != 0x0001 && id != 0x0004 && id != 0x000d && id != 0x0206) {
            break;
        }
        Serial.printf("  [sdp]   attr 0x%04x len=%u:", id, len);
        if (id != 0x0206) {
            for (uint16_t i = 0; i < len && i < sizeof(sdpAttrValue); i++) {
                Serial.printf(" %02x", sdpAttrValue[i]);
            }
        }
        Serial.println();
        break;
    }
    case SDP_EVENT_QUERY_COMPLETE:
        Serial.printf("  [sdp] query complete: status 0x%02x\n",
                      sdp_event_query_complete_get_status(packet));
        sdpDumpRunning = false;
        break;
    default:
        break;
    }
}

static void dumpSdpRecords(const uint8_t *address) {
    bd_addr_t addr;
    memcpy(addr, address, sizeof(addr));
    Serial.println("  dumping the SDP records of the device...");

    // hid_host が SDP クライアントを手放すまで待つ。
    const unsigned long startedAt = millis();
    uint8_t status = SDP_QUERY_BUSY;
    while (millis() - startedAt < 5000) {
        {
            BluetoothLock lock;
            if (sdp_client_ready()) {
                sdpDumpRunning = true;
                status = sdp_client_query_uuid16(&sdpDumpHandler, addr, BLUETOOTH_PROTOCOL_L2CAP);
            }
        }
        if (status != SDP_QUERY_BUSY) {
            break;
        }
        delay(50);
    }
    if (status != ERROR_CODE_SUCCESS) {
        sdpDumpRunning = false;
        Serial.printf("  [sdp] could not start the query (0x%02x)\n", status);
        return;
    }
    const unsigned long queryStartedAt = millis();
    while (sdpDumpRunning && millis() - queryStartedAt < 20000) {
        updateConnectionLed();
        delay(20);
    }
    if (sdpDumpRunning) {
        Serial.println("  [sdp] query timed out");
    }
}

// 1 台に対して hid_host_connect() を投げ、HID ディスクリプタが揃うまで待つ。
// ライブラリの connectJoystick() は HID ディスクリプタを無限に待つため使わない。
static bool connectToAddress(const uint8_t *address) {
    if (DIRECT_L2CAP_FALLBACK && noSdpAddrValid && !memcmp(address, noSdpAddr, sizeof(noSdpAddr))) {
        return connectDirect(address);
    }

    bd_addr_t addr;
    memcpy(addr, address, sizeof(addr));
    hidConnectFailStatus = 0;
    uint8_t status;
    {
        BluetoothLock lock;
        status = hid_host_connect(addr, HID_PROTOCOL_MODE_REPORT, &hidCid);
    }
    if (status == ERROR_CODE_COMMAND_DISALLOWED) {
        // 0x0c: このアドレス宛の接続レコードが既にある。
        // コントローラ側から繋ぎに来ている途中か、前回失敗した接続が
        // 残っている。まずは完了を待ち、駄目ならリンクごと落として次へ。
        Serial.println("  a link to this device already exists: waiting for it");
        if (waitUntilConnected(CONNECT_TIMEOUT_MS)) {
            return true;
        }
        Serial.println("  stale/incomplete link: dropping it");
        forceDisconnect();
        delay(500);
        return false;
    }
    if (status != ERROR_CODE_SUCCESS) {
        Serial.printf("  connect request rejected (0x%02x)\n", status);
        hidCid = 0;
        return false;
    }

    if (waitUntilConnected(CONNECT_TIMEOUT_MS)) {
        return true;
    }

    // btstack が失敗を返した (HID イベントとして上がってきた)。
    if (hidConnectFailStatus) {
        const uint8_t failed = hidConnectFailStatus;
        hidConnectFailStatus = 0;
        hidCid = 0;
        if (failed != ERROR_CODE_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE) {
            forceDisconnect();
            delay(1000);
            return false;
        }
        dumpSdpRecords(address);
        forceDisconnect();
        delay(500);
        if (!DIRECT_L2CAP_FALLBACK) {
            return false;
        }
        memcpy(noSdpAddr, address, sizeof(noSdpAddr));
        noSdpAddrValid = true;
        return connectDirect(address);
    }

    // ここに来るパターンは 2 つある。
    //  1) [hid] connection opened が出ている
    //     -> L2CAP までは繋がったが HID ディスクリプタが取れない。
    //        8BitDo の X / Switch モードなど、標準 HID でない場合。
    //        D モード (START + B) か macOS モード (START + A) で起動し直す。
    //  2) [hid] の行が 1 つも出ていない
    //     -> SDP クエリの段階で止まっている。相手が SDP に応答していない。
    Serial.println("  timed out waiting for the HID descriptor");
    if (hidConnected) {
        Serial.println("  L2CAP is up but no HID descriptor: not a standard HID device?");
        Serial.println("  (8BitDo は D モード START+B か macOS モード START+A で起動する)");
    } else {
        Serial.println("  stalled in the SDP query (no HID event at all)");
    }
    forceDisconnect();
    delay(2000);   // btstack が後始末を終えるまで待つ
    return false;
}

// --- Class of Device の判定 ---------------------------------------------
// CoD は bits 12-8 が Major Device Class、bits 7-2 が Minor Device Class。
// bits 23-13 はサービスクラスで、Limited Discoverable かどうかで変わるため
// 種別の判定には使わない。
//   Major 0x02 = Phone (OPPO Reno3 などのスマートフォン)
//   Major 0x05 = Peripheral (キーボード / マウス / ゲームパッド)
// Peripheral の Minor は上位 2 ビットがキーボード/ポインタのフラグ、
// 下位 4 ビットが種別 (1 = ジョイスティック、2 = ゲームパッド、3 = リモコン)。
static const uint8_t COD_MAJOR_PERIPHERAL = 0x05;
static const uint8_t COD_PERIPHERAL_JOYSTICK = 0x01;
static const uint8_t COD_PERIPHERAL_GAMEPAD  = 0x02;

static uint8_t codMajorClass(uint32_t cod) {
    return (uint8_t)((cod >> 8) & 0x1f);
}

static uint8_t codMinorClass(uint32_t cod) {
    return (uint8_t)((cod >> 2) & 0x3f);
}

static bool codIsGamepad(uint32_t cod) {
    if (codMajorClass(cod) != COD_MAJOR_PERIPHERAL) {
        return false;
    }
    const uint8_t type = codMinorClass(cod) & 0x0f;
    return (type == COD_PERIPHERAL_JOYSTICK) || (type == COD_PERIPHERAL_GAMEPAD);
}

// ログ用の短い種別名。
static const char *codKindName(uint32_t cod) {
    if (codIsGamepad(cod)) {
        return "gamepad";
    }
    switch (codMajorClass(cod)) {
    case 0x01: return "computer";
    case 0x02: return "phone";
    case 0x04: return "audio";
    case COD_MAJOR_PERIPHERAL: return "peripheral";
    default: return "other";
    }
}

// TARGET_NAME の部分一致 (大文字小文字は無視)。TARGET_NAME が空なら常に真。
static bool nameMatches(const char *name) {
    if (!TARGET_NAME[0]) {
        return true;
    }
    if (!name) {
        return false;
    }
    const size_t needleLen = strlen(TARGET_NAME);
    const size_t nameLen = strlen(name);
    if (nameLen < needleLen) {
        return false;
    }
    for (size_t i = 0; i + needleLen <= nameLen; i++) {
        if (!strncasecmp(name + i, TARGET_NAME, needleLen)) {
            return true;
        }
    }
    return false;
}

// スキャンで見つかった機器を接続対象にするか。
static bool deviceAccepted(BTDeviceInfo &dev) {
    if (!nameMatches(dev.name())) {
        return false;
    }
    switch (DEVICE_FILTER) {
    case 0:  return codIsGamepad(dev.deviceClass());
    case 1:  return codMajorClass(dev.deviceClass()) == COD_MAJOR_PERIPHERAL;
    default: return true;
    }
}

// TARGET_ADDR が設定されているか (全ゼロなら未設定扱い)。
static bool haveTargetAddr() {
    for (size_t i = 0; i < sizeof(TARGET_ADDR); i++) {
        if (TARGET_ADDR[i]) {
            return true;
        }
    }
    return false;
}

// クラシック Bluetooth (BR/EDR) で接続する。
// まず MAC 決め打ちで page し、駄目ならスキャンして見つかった順に試す。
static bool connectClassic() {
    if (haveTargetAddr()) {
        Serial.printf("connecting to %02x:%02x:%02x:%02x:%02x:%02x (fixed address)...\n",
                      TARGET_ADDR[0], TARGET_ADDR[1], TARGET_ADDR[2],
                      TARGET_ADDR[3], TARGET_ADDR[4], TARGET_ADDR[5]);
        if (connectToAddress(TARGET_ADDR)) {
            return true;
        }
    }

    // マスクなしで拾って、絞り込みは deviceAccepted() で自前に行う。
    Serial.printf("scanning %d s (filter %d)...\n", SCAN_TIME_SEC, DEVICE_FILTER);
    std::vector<BTDeviceInfo> found = hid.scan(BluetoothHIDMaster::any_cod, SCAN_TIME_SEC);

    if (found.empty()) {
        Serial.println("  no device found");
        return false;
    }

    // 何が見えていて、何を弾いたかが分かるように全件出す。
    int accepted = 0;
    for (auto &dev : found) {
        const bool ok = deviceAccepted(dev);
        accepted += ok ? 1 : 0;
        Serial.printf("  %s \"%s\" [%s] cod=0x%06lx (%s) rssi=%d\n",
                      ok ? "found " : "skip  ", dev.name(), dev.addressString(),
                      (unsigned long)dev.deviceClass(),
                      codKindName(dev.deviceClass()), dev.rssi());
    }
    if (!accepted) {
        Serial.println("  no controller among them (DEVICE_FILTER で条件を緩められる)");
        return false;
    }

    for (auto &dev : found) {
        if (!deviceAccepted(dev)) {
            continue;
        }
        // 決め打ちのアドレスは既に試したので飛ばす。
        if (haveTargetAddr() && !memcmp(dev.address(), TARGET_ADDR, sizeof(TARGET_ADDR))) {
            continue;
        }
        Serial.printf("connecting to \"%s\" [%s]...\n", dev.name(), dev.addressString());
        if (connectToAddress(dev.address())) {
            return true;
        }
    }
    return false;
}

static void onConnected() {
    resetTrackingState();
    if (USE_BLE) {
        const uint8_t *addr = hid.lastConnectedAddress();
        Serial.printf("controller connected: %02x:%02x:%02x:%02x:%02x:%02x (BLE)\n",
                      addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    } else {
        Serial.printf("controller connected: %s (BR/EDR%s)\n", connectedAddrText,
                      directConnected ? ", direct L2CAP / DS4 report format" : "");
    }
    if (PRINT_INPUT_LOG) {
        Serial.println("press buttons / move sticks to see the reports");
    }
    wasConnected = true;
}

// 接続を 1 サイクルだけ試す。ブロックする処理はすべて上限時間を持つので、
// 接続できてもできなくても必ず戻ってくる。繰り返しは loop() 側で行う。
static void attemptConnect() {
    hidConnectFailStatus = 0;
    directFailStatus = 0;
    // ペアリング済みならコントローラ側から繋ぎに来るので、まず着信を待つ。
    if (waitUntilConnected(INCOMING_WAIT_MS)) {
        onConnected();
        return;
    }

    // 見つからなければこちらからスキャンする (コントローラはペアリングモード)。
    const bool ok = USE_BLE ? hid.connectBLE() : connectClassic();
    if (ok && isControllerConnected()) {
        onConnected();
        return;
    }

    if (millis() - lastWaitingLogMs >= WAITING_LOG_INTERVAL_MS) {
        Serial.println("still waiting: put the controller into pairing mode ('f' to forget keys)");
        lastWaitingLogMs = millis();
    }
}

// シリアルから 'f' を受けたらペアリング情報を消して繋ぎ直す。
static void handleSerialCommand() {
    while (Serial.available()) {
        const int c = Serial.read();
        if (c == 'f' || c == 'F') {
            Serial.println("forgetting pairing keys");
            stopOutputs();
            forceDisconnect();
            {
                BluetoothLock lock;
                gap_delete_all_link_keys();
            }
            wasConnected = false;
        }
    }
}

// ---------------------------------------------------------------------------
// レポート処理
// ---------------------------------------------------------------------------

// ボタンのビット割り当ては機種と起動モードで変わる。名前を決め打ちせず、
// 押されたビット番号 (HID の Button usage と同じ 1 始まり) を出す。
static void printButtonChanges(uint32_t before, uint32_t after) {
    const uint32_t changed = before ^ after;
    for (int bit = 0; bit < 32; bit++) {
        const uint32_t mask = 1UL << bit;
        if (!(changed & mask)) {
            continue;
        }
        Serial.printf("[button] #%d %s\n", bit + 1, (after & mask) ? "DOWN" : "UP");
    }
}

// スティックが動いたか (しきい値以上の変化があったか)。
static bool axesMoved(const GamepadReport &a, const GamepadReport &b) {
    return abs(a.x - b.x) >= AXIS_LOG_THRESHOLD ||
           abs(a.y - b.y) >= AXIS_LOG_THRESHOLD ||
           abs(a.z - b.z) >= AXIS_LOG_THRESHOLD ||
           abs(a.rz - b.rz) >= AXIS_LOG_THRESHOLD;
}

// blue_test.ino と同じ形式で、生値のまま操作ログを出す。
static void logInputReport(const GamepadReport &report) {
    if (report.buttons != lastLoggedRaw.buttons) {
        printButtonChanges(lastLoggedRaw.buttons, report.buttons);
        Serial.printf("[buttons] 0x%08lx\n", (unsigned long)report.buttons);
    }
    if (report.hat != lastLoggedRaw.hat) {
        Serial.printf("[hat] %s (0x%02x)\n", hatName(report.hat), report.hat);
    }
    if (firstInputLog || axesMoved(report, lastLoggedRaw)) {
        Serial.printf("[axes] LX=%4d LY=%4d RX=%4d RY=%4d\n",
                      report.x, report.y, report.z, report.rz);
        lastLoggedRaw.x = report.x;
        lastLoggedRaw.y = report.y;
        lastLoggedRaw.z = report.z;
        lastLoggedRaw.rz = report.rz;
    }
    lastLoggedRaw.hat = report.hat;
    lastLoggedRaw.buttons = report.buttons;
    firstInputLog = false;
}

static void reportButtons(uint32_t buttons) {
    const uint32_t changed = buttons ^ lastButtons;
    if (!changed) {
        return;
    }
    for (int i = 0; i < 32; i++) {
        const uint32_t mask = 1UL << i;
        if (!(changed & mask)) {
            continue;
        }
        Serial.printf("button %2d %s\n", i, (buttons & mask) ? "pressed" : "released");
    }
    lastButtons = buttons;
}

static void reportHat(uint8_t hat) {
    if (hat == lastHat) {
        return;
    }
    int dx = 0, dy = 0;
    hatToDirection(hat, &dx, &dy);
    Serial.printf("dpad %s (%d, %d)\n", hatName(hat), dx, dy);
    lastHat = hat;
}

static void reportAxes(float lx, float ly, float rx, float ry) {
    const unsigned long now = millis();
    const bool moved = fabsf(lx - lastLX) >= AXIS_REPORT_DELTA ||
                       fabsf(ly - lastLY) >= AXIS_REPORT_DELTA ||
                       fabsf(rx - lastRX) >= AXIS_REPORT_DELTA ||
                       fabsf(ry - lastRY) >= AXIS_REPORT_DELTA;
    if (!moved || now - lastAxisReportMs < REPORT_INTERVAL_MS) {
        return;
    }
    Serial.printf("stick L(%+.2f, %+.2f) R(%+.2f, %+.2f)\n", lx, ly, rx, ry);
    lastLX = lx;
    lastLY = ly;
    lastRX = rx;
    lastRY = ry;
    lastAxisReportMs = now;
}

static void processReport(const GamepadReport &report) {
    if (PRINT_RAW_AXES) {
        Serial.printf("raw x=%d y=%d z=%d rz=%d hat=%u buttons=0x%08lx\n",
                      report.x, report.y, report.z, report.rz,
                      (unsigned)report.hat, (unsigned long)report.buttons);
    }

    // 左スティック = x/y、右スティック = z/rz。上方向が負になる。
    const float leftX = normalizeAxis(report.x);
    const float leftY = normalizeAxis(report.y);
    const float rightX = normalizeAxis(report.z);
    const float rightY = normalizeAxis(report.rz);

    if (PRINT_INPUT_LOG) {
        logInputReport(report);
    }
    if (PRINT_NORMALIZED_LOG) {
        reportButtons(report.buttons);
        reportHat(report.hat);
        reportAxes(leftX, leftY, rightX, rightY);
    }

    applyOutputs(leftX, leftY, rightX, rightY);
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    const unsigned long startedAt = millis();
    while (!Serial && millis() - startedAt < 3000) {
        delay(10);
    }

    Serial.println();
    Serial.println("Pico 2 W Bluetooth gamepad host");
    Serial.printf("transport: %s\n", USE_BLE ? "BLE (HOGP)" : "Bluetooth Classic (BR/EDR)");
    if (!USE_BLE) {
        Serial.printf("BR/EDR security level: %d\n", BREDR_SECURITY_LEVEL);
    }
    Serial.println("send 'f' to forget pairing keys and reconnect");

    setupMotors();

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    hid.onJoystick(onJoystickReport);

    if (USE_BLE) {
        hid.begin(true, BLE_MASTER_NAME);
    } else {
        hid.begin();
    }

    // HCI が上がるまで待つ (最大 5 秒)。この後の gap_* 呼び出しのため。
    const unsigned long hciWaitStartedAt = millis();
    while (!hid.hciRunning() && millis() - hciWaitStartedAt < 5000) {
        delay(10);
    }

    {
        // ACL リンクの上げ下げを追う (失敗した接続を強制的に切るため)。
        BluetoothLock lock;
        hciEventRegistration.callback = &hciEventHandler;
        hci_add_event_handler(&hciEventRegistration);

        if (!USE_BLE) {
            gap_set_security_level((gap_security_level_t)BREDR_SECURITY_LEVEL);
            gap_ssp_set_io_capability(SSP_IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
            gap_ssp_set_auto_accept(true);

            // HID の L2CAP サービスは hid.begin() の中の hid_host_init() が
            // 既に登録済みで、その required_security_level は登録時点の
            // gap_get_security_level() (btstack 既定の LEVEL_2) で固定される。
            // l2cap_register_service() は 2 回目を
            // L2CAP_SERVICE_ALREADY_REGISTERED で弾くだけなので、上で
            // gap_set_security_level() を呼んでも着信側には反映されない。
            // 着信 (ペアリング済みコントローラからの再接続) が認証待ちで
            // 止まると、その後の hid_host_connect() まで 0x0c
            // (ERROR_CODE_COMMAND_DISALLOWED) で巻き添えになるため、
            // いったん登録を外してから登録し直す。
            l2cap_unregister_service(BLUETOOTH_PSM_HID_INTERRUPT);
            l2cap_unregister_service(BLUETOOTH_PSM_HID_CONTROL);

            // HID ホストを自前のバッファ/ハンドラで初期化する。
            // ここでサービスが BREDR_SECURITY_LEVEL で登録し直される。
            hid_host_init(hidDescriptorStorage, sizeof(hidDescriptorStorage));
            hid_host_register_packet_handler(&hidHostEventHandler);

            // ペアリング済みのコントローラから繋ぎに来られるようにする。
            gap_connectable_control(1);
            gap_discoverable_control(1);
        }
    }

    if (FORGET_KEYS_ON_BOOT) {
        Serial.println("clearing stored pairing keys");
        BluetoothLock lock;
        gap_delete_all_link_keys();
    }

    Serial.println("waiting for a controller...");
}

void loop() {
    handleSerialCommand();
    pumpLinkLog();
    updateConnectionLed();

    if (!isControllerConnected()) {
        if (wasConnected) {
            Serial.println("controller disconnected");
            stopOutputs();
            wasConnected = false;
            hidCid = 0;
        }
        attemptConnect();
        return;
    }

    GamepadReport report;
    bool pending;
    noInterrupts();
    pending = reportPending;
    report = sharedReport;
    reportPending = false;
    interrupts();

    if (!pending) {
        delay(1);
        return;
    }

    processReport(report);
}
