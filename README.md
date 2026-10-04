# Pico 2 W Bluetooth Gamepad Host

Raspberry Pi Pico 2 W 上で Bluetooth ゲームコントローラを直接受け取り、その入力を DRV8833 で駆動するモーター制御用の Arduino スケッチです。

このプロジェクトは、PC やスマートフォンを中継せずに Pico 2 W 自身が HID ホストとして動作し、コントローラの左右スティック入力を 4ch のモーター出力へ変換します。

## 概要

- Raspberry Pi Pico 2 W が Bluetooth HID ホストになる
- クラシック Bluetooth (BR/EDR) と BLE の両方に対応
- 左右スティック入力を 4 つのモーター出力にマッピング
- DRV8833 x 2 を使って 4 モーターを制御
- シリアルモニタで入力値とモーター状態を確認可能

## 対応する入力ソース

- BR/EDR: 8BitDo、DualSense、DualShock 4、Switch Pro など
- BLE (HOGP): Xbox Series コントローラなど

## 対応ハードウェア

- Raspberry Pi Pico 2 W
- DRV8833 motor driver x 2
- DC モーター x 4
- USB 接続用の安定電源
- Bluetooth ゲームコントローラ

## 配線

以下のピン割り当てを使用します。

```text
DRV8833 #1
  Motor A: AIN1 -> GP2, AIN2 -> GP3
  Motor B: BIN1 -> GP4, BIN2 -> GP5

DRV8833 #2
  Motor A: AIN1 -> GP6, AIN2 -> GP7
  Motor B: BIN1 -> GP8, BIN2 -> GP9
```

入力マッピングは次の通りです。

```text
左 X  -> DRV8833 #1 Motor A
左 Y  -> DRV8833 #1 Motor B
右 X  -> DRV8833 #2 Motor A
右 Y  -> DRV8833 #2 Motor B
```

Y 軸は `INVERT_STICK_Y` により反転可です。

## 必要な環境

Arduino IDE で以下を設定してください。

1. ボードマネージャから `Raspberry Pi Pico/RP2040/RP2350` をインストール
2. ボード選択: `Raspberry Pi Pico 2 W`
3. Tools > IP/Bluetooth Stack で `IPv4 + Bluetooth` または `IPv4/IPv6 + Bluetooth` を選択
4. 必要に応じてライブラリを追加
   - `BluetoothHIDMaster`
   - `BluetoothLock`

> 注: Bluetooth のスタックを選ばないとビルド時にエラーになることがあります。

## 主要設定項目

スケッチ先頭の定数を調整してください。

```cpp
static const bool USE_BLE = false;
static const char *BLE_MASTER_NAME = "PICO2W";
static uint8_t TARGET_ADDR[6] = {0, 0, 0, 0, 0, 0};
static const char *TARGET_NAME = "";
static const float AXIS_DEADZONE = 0.12f;
static const bool INVERT_STICK_Y = true;
```

- `USE_BLE = false` : クラシック Bluetooth 接続
- `USE_BLE = true` : BLE 接続
- `TARGET_ADDR` : 接続先 MAC アドレスを固定したいときに使用
- `TARGET_NAME` : 機器名で絞り込みたいときに使用
- `AXIS_DEADZONE` : 中央付近のゆらぎを無視する値
- `INVERT_STICK_Y` : 上下方向の入力反転を切り替える

## 使い方

1. Pico 2 W にスケッチを書き込む
2. コントローラの電源を入れる
3. シリアルモニタを開いて 115200 bps で確認する
4. コントローラが自動検出・接続されるのを待つ
5. スティックを動かしてモーターが反応することを確認する

### BR/EDR (クラシック Bluetooth) の場合

```cpp
static const bool USE_BLE = false;
```

コントローラが見つからない場合は、`TARGET_ADDR` に MAC アドレスをセットするか、`TARGET_NAME` を指定して絞り込みます。

### BLE の場合

```cpp
static const bool USE_BLE = true;
```

Xbox Series コントローラのように BLE のみの機種ではこの設定が必要です。

## シリアルログ

このスケッチは接続状態と入力状態をシリアル出力します。

- 接続の進行状況
- コントローラが見つかったか
- ボタンの押下状態
- スティックの生値または正規化後の値
- モーターの回転方向とデューティ

`PRINT_INPUT_LOG`、`PRINT_NORMALIZED_LOG`、`PRINT_MOTOR_STATE` を true/false で切り替えられます。

## トラブルシューティング

### コントローラに接続できない

- `TARGET_ADDR` を明示する
- `TARGET_NAME` で機器名を絞る
- `USE_BLE` の設定が合っているか確認する
- コントローラが専用のペアリングモードになっているか確認する

### HID ディスクリプタが取得できない

- `DIRECT_L2CAP_FALLBACK` を有効にする
- 8BitDo など一部の機種では別モードで起動が必要なことがある
- ブルートゥースの接続設定やペアリング状態を確認する

### モーターが動かない

- DRV8833 の GND と Pico の GND を共通化しているか確認
- PWM 出力ピンの配線が正しいか確認
- 電源容量が不足していないか確認
- `applyOutputs()` でスティック入力が正しく出力されていることをログで確認

## 実装メモ

`loop()` 側で `sharedReport` を追跡し、左右の軸値を正規化し、`applyOutputs()` で各モーターに出力します。

- `_deadzone` による中央の誤差除去
- `normalizeAxis()` による -1.0 ～ 1.0 の変換
- `setSpeed()` による DRV8833 の PWM 制御

## ファイル構成

```text
.
├── pico2w_gamepad.ino
├── README.md
└── .gitignore
```

## ライセンス

ライセンスは明記されていません。個人利用・実験用途に合わせて運用してください。
