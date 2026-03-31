## 概要
- **時間管理**: Tickerによる定周期割り込みと、TaskSchedulerによる非ブロッキングなイベント処理。
- **タスク管理**: 割り込み内では実行トリガー（restart）のみを発行し、実処理をタスクで行う。
- **低負荷**: タスク実行時以外は アイドル（disable）状態 とし、WiFi通信等のシステムリソースを最大限に確保。

## 動作環境
- **ハードウェア**: 
  - IOT Integrated Controller V1　RRH-G101A REV-A (ESP8266搭載)
  - 一般的な ESP-WROOM-02D 4M (ESP8266搭載)
- **開発ツール**: PlatformIO (VSCode)
- **使用ライブラリ**: 
  - [TaskScheduler](https://github.com/arkhipenko/TaskScheduler)
  - Ticker (ESP8266 Core内蔵)

## 接続設定
- **LEDピン**: GPIO16 (ボード上のJP1をLED側に設定)
- **シリアル通信**: 115200 bps

## セットアップと書き込み
1. VSCodeでこのプロジェクトフォルダを開きます。
2. PlatformIOの `Upload and Monitor` を実行します。
3. LEDが約500ms周期で点滅し、シリアルモニターにタスク実行ログが表示されます。

## ライセンス
[MIT License](https://opensource.org/license/mit)