/**
 * タスクスケジューラーとティッカー ライブラリーを使ったスリープモード
 * 信号のエッジ検出、ESP WROOM 02 内部ＡＤＣの読込のサンプルプログラム
 * for PlatformIO
 * 
 * 対応ボード：IOT Integrated Controller V1
 *            RRH-G101A REV-B
 * 
 * 2026-03-30
 * Copyright (c) 2026 rinwado
 * Licensed under the MIT License.
 * See LICENSE file in the project root for full license text.
 */

#include <Arduino.h>
#include <Ticker.h>
#include <TaskScheduler.h>
extern "C" {
  #include "user_interface.h"               //SDKの関数(system_get_rst_info)を使うため
}

/// 定義
#define ADC_VCC_MODE          (0)           //1:ADCの入力がチップ内部の電源ラインを読む、 0:ADCの入力が外部のA0ピンの値を読む
#define ADC_A0_LSB_MV         (0.976562F)   //1000mV ÷ 1024
#define CHECK_INTERRUPT_TIME  (0)           //onTickTimerISR　の処理股間を確認する場合（１）
#define SW2_PIN               (0)           //プログラムボタンと兼用しているので、起動時の検出はできない
#define SW2_BUTTON            (0x01)
#define RTC_RAM_CHECK_CODE    (0x86C3A524)  //4byte, RTC RAM 初期化判断

/// 構造体
typedef struct rtc_mem
{ //４バイト単位でアクセスする必要があるので、４バイトの変数使用
  uint32_t pu_ram_code;
  uint32_t pu_counter;
  int32_t sleep_mode;  
} rtc_mem_t;

/// プロトタイプ宣言
void IRAM_ATTR onTickTimerISR(void);
void MainWork_Callback(void);
void TskOneShot_ProcCallback(void);

/// オブジェクト生成
Scheduler TskRunner;                      //スケジューラ
Ticker tick_timer1;                       //ティッカー割り込み
//Task(周期ms, 実行回数[-1は無限], 実行する関数, スケジューラへのポインタ)
//Taskの宣言した順にタスク管理リストに登録され、管理リストの順に実行タイミングがチェックされる。
Task tsk_main_work(1, TASK_FOREVER, &MainWork_Callback, &TskRunner);                      //[main_work]タスクの作成
Task tsk_OneShot_01(TASK_IMMEDIATE, TASK_ONCE, &TskOneShot_ProcCallback, &TskRunner);     //[Proc]OneShotタスクの作成

/// 変数
volatile uint16_t gn_cnt1 = 0;            //汎用カウンタ１
volatile uint16_t gn_cnt2 = 0;            //汎用カウンタ２
volatile uint8_t f_FE_SignalDetect, f_RE_SignalDetect;
volatile bool f_counter_trigger = false;  //割込みカウンタによるトリガフラグ
#if(CHECK_INTERRUPT_TIME)
volatile uint32_t elapsed_cycles = 0;     //経過サイクル数(割込み処置にかかった経過時間計測のために)
#endif

uint8_t doWork_count = 0;

struct rst_info* ResetStartInfo;  //リセット情報
rtc_mem_t RTC_RAM_B1;             //RTC RAM


#if(ADC_VCC_MODE)
ADC_MODE(ADC_VCC);
#define ESP8266_ADC_READ  ESP.getVcc()
#else
#define ESP8266_ADC_READ  analogRead(A0)
#endif


/**
 * @brief Arduino setup
 * 
 */
void setup()
{
  //--- IOピン設定
  pinMode(SW2_PIN, INPUT);

  //--- リセットスタート時の起動情報を取得
  ResetStartInfo = system_get_rst_info();

  //Deep-sleepでも値を保持できるのRTCメモリ（RTC RAM）を使用
  ESP.rtcUserMemoryRead(0, (uint32_t*)&RTC_RAM_B1, sizeof(RTC_RAM_B1));
  if(RTC_RAM_CHECK_CODE != RTC_RAM_B1.pu_ram_code)
  { //電源が完全に切れた後の初回起動と判断
    RTC_RAM_B1.pu_ram_code = RTC_RAM_CHECK_CODE;  //起動時にコードをセット
    RTC_RAM_B1.pu_counter = 0;                    //初回起動時に初期化
    RTC_RAM_B1.sleep_mode = -1;                   //
  }
  RTC_RAM_B1.pu_counter++;
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&RTC_RAM_B1, sizeof(RTC_RAM_B1));

  //--- シリアル
  Serial.begin(115200);
  Serial.printf("\r\nFlash Real Size: %u bytes\r\n", ESP.getFlashChipRealSize()); //ESP8266に搭載されているフラッシュメモリ容量

  //--- 起動情報
  switch(ResetStartInfo->reason)
  {
    case REASON_DEFAULT_RST:
      Serial.printf("[%d] Normal startup by power on. PowerUP Counter=%d\r\n", (int)ResetStartInfo->reason, (int)RTC_RAM_B1.pu_counter); 
    break;
    case REASON_WDT_RST:
      Serial.printf("[%d] Hardware watch dog reset. PowerUP Counter=%d\r\n", (int)ResetStartInfo->reason, (int)RTC_RAM_B1.pu_counter); 
    break;
    case REASON_EXCEPTION_RST:
      Serial.printf("[%d] Exception reset, GPIO status won’t change. PowerUP Counter=%d\r\n", (int)ResetStartInfo->reason, (int)RTC_RAM_B1.pu_counter); 
      Serial.printf("     Fatal exception (%d):\r\n", (int)ResetStartInfo->exccause);
      Serial.printf("     epc1=0x%08x, epc2=0x%08x, epc3=0x%08x, excvaddr=0x%08x, depc=0x%08x\r\n",
                          ResetStartInfo->epc1, ResetStartInfo->epc2, ResetStartInfo->epc3, ResetStartInfo->excvaddr, ResetStartInfo->depc); 
    break;
    case REASON_SOFT_WDT_RST:
      Serial.printf("[%d] Software watch dog reset, GPIO status won’t change. PowerUP Counter=%d\r\n", (int)ResetStartInfo->reason, (int)RTC_RAM_B1.pu_counter); 
    break;
    case REASON_SOFT_RESTART:
      Serial.printf("[%d] Software restart, system_restart, GPIO status won’t change. PowerUP Counter=%d\r\n", (int)ResetStartInfo->reason, (int)RTC_RAM_B1.pu_counter); 
    break;
    case REASON_DEEP_SLEEP_AWAKE:
      Serial.printf("[%d] Wake up from deep-sleep. PowerUP Counter=%d\r\n", (int)ResetStartInfo->reason, (int)RTC_RAM_B1.pu_counter); 
    break;
    case REASON_EXT_SYS_RST:
      Serial.printf("[%d] External system reset. PowerUP Counter=%d\r\n", (int)ResetStartInfo->reason, (int)RTC_RAM_B1.pu_counter); 
    break;
    default:
      Serial.printf("[%d] Unknown  system reset. PowerUP Counter=%d\r\n", (int)ResetStartInfo->reason, (int)RTC_RAM_B1.pu_counter); 
    break;
  }

  //--- ティッカーの開始 秒数で指定(0.003333 = 3.333ms, onTimerISR関数をセット)
  tick_timer1.attach(0.003333, onTickTimerISR);
  //--- タスクを有効化
  tsk_main_work.enable();
  //tsk_OneShot_01.enable();  //TASK_ONCE　なので、ここではenableにしない
}



/**
 * @brief Arduino loop
 * 
 */
void loop()
{
  //--- スケジューラを回す
  TskRunner.execute();

  //Sleep に入るタイミング確認
  if(40 <= doWork_count)
  { //４０カウント（0.5 * 40 = 20S）動作後にスリープに入り、３０秒スリープ
    doWork_count = 0;
    if(0 < (RTC_RAM_B1.sleep_mode & 0x00000003))
    { //Sleep にはいる
      Serial.printf("Going to Deep-sleep for 30 seconds...\r\n");
      ESP.deepSleep(30 * 1000000); 
      delay(1000);
    }
  }
}



/**
 * @brief ティッカーによる割込み処理
 *        3.333mS ごとに処理
 *        ここには長い処理やdelay()などは書かない、１ｍｓ以下で処理が完了するような内容が望ましい
 */
void IRAM_ATTR onTickTimerISR(void)
{
  static uint8_t Signal_Filter[14] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  static uint8_t IO_SignalMonitorVal = 0x00;
  static uint8_t re, fe;
  static int8_t  r, ssv;
  
  #if(CHECK_INTERRUPT_TIME)
  uint32_t start_cycle = ESP.getCycleCount();           //入った時のカウント数
  #endif

  //信号サンプリング
  IO_SignalMonitorVal = (0 != digitalRead(SW2_PIN))?  (IO_SignalMonitorVal | SW2_BUTTON)  : (IO_SignalMonitorVal & ~SW2_BUTTON);  //IO0 (sw2 button)

  //フィルターと信号安定チェック
  ssv = 0;
  for(r=11; r>0; r--)
  { Signal_Filter[r] = Signal_Filter[r-1];              //データシフト
    if(Signal_Filter[r] != IO_SignalMonitorVal) ssv++;  //最新データと異なっていたらカウントアップ
  }
  Signal_Filter[r] = IO_SignalMonitorVal;               //最新データ

  if(0 == ssv)
  {	//信号サンプリングデータが１２連続(3.333ms x 12 = 40ms)同じ安定
      //信号 ON/OFF エッジ抽出
      Signal_Filter[13] = ~Signal_Filter[12];
      Signal_Filter[12] = Signal_Filter[11];
      fe = ~(Signal_Filter[13] | Signal_Filter[12]);    //信号の立下がりエッジ検出
      re =   Signal_Filter[13] & Signal_Filter[12];     //信号の立上がりエッジ検出

      if((fe != 0x00) || (re != 0x00))
      {	//立下り、立ち上がりの変化あり
          //信号　変化(H -> L)
          if(0 != (fe & SW2_BUTTON))     f_FE_SignalDetect |= SW2_BUTTON;     //bit0:

          //信号　変化(L -> H)
          if(0 != (re & SW2_BUTTON))     f_RE_SignalDetect |= SW2_BUTTON;     //bit0:
      }
  }


  //-----
  gn_cnt1++;
  #if(0)
  if(300 <= gn_cnt1)
  { //main_work 用のトリガフラグ (1000ms)
    gn_cnt1 = 0;
    f_counter_trigger = true;
  }
  #endif
  //-----
  gn_cnt2++;
  if(150 <= gn_cnt2)
  { //ワンショット処理タスク (500ms)
    gn_cnt2 = 0;
    tsk_OneShot_01.restart();
  }

  #if(CHECK_INTERRUPT_TIME)
  uint32_t end_cycle = ESP.getCycleCount();             //抜け出すときにカウント数
  elapsed_cycles = end_cycle - start_cycle;             //処理にかかったカウント数
  #endif
}

/**
 * @brief TaskScheduler コールバック関数
 *        １ｍｓ 間隔で実行される
 */
void MainWork_Callback(void)
{
  #if(0)
  if(f_counter_trigger)
  { f_counter_trigger = false;
    Serial.printf("TASK: MainWork Proc.\r\n");
  }
  #endif

  //ＳＷ２ボタンフラグ確認（押下）
  if(0 != (f_FE_SignalDetect & SW2_BUTTON))
  { //ＳＷ２が押下された
    Serial.printf("TASK: MainWork Proc (SW2 ON...)\r\n");

    #if(CHECK_INTERRUPT_TIME)
    //80MHz動作の場合、1サイクル = 0.0125μS (1μS = 80サイクル)
    float elapsed_us = elapsed_cycles / 80.0; 
    Serial.printf("Elapsed cycles: %d, Elapsed time: %d[us]\r\n", (int)elapsed_cycles, (int)elapsed_us);
    #endif

    f_FE_SignalDetect &= ~SW2_BUTTON;
  }

  //ＳＷ２ボタンフラグ確認（離上）
  if(0 != (f_RE_SignalDetect & SW2_BUTTON))
  { //ＳＷ２が離された
    Serial.printf("TASK: MainWork Proc (SW2 OFF..)\r\n");

    RTC_RAM_B1.sleep_mode++;
    Serial.printf("Sleep Mode (%d), 0:no sleep, 1..3:deep sleep\r\n", (int)(RTC_RAM_B1.sleep_mode & 0x00000003));
    ESP.rtcUserMemoryWrite(0, (uint32_t*)&RTC_RAM_B1, sizeof(RTC_RAM_B1));

    f_RE_SignalDetect &= ~SW2_BUTTON;
  }
}

/**
 * @brief ワンショット処理タスク
 *        ティックタイマー割込みの「リスタート」で処理が行われる
 *        処理が終わるとタスクは、disable（休止状態）
 */
void TskOneShot_ProcCallback(void)
{
  static int8_t  add_cnt = 0;
  static uint32_t adc_dfata = 0;

  adc_dfata += (uint32_t)ESP8266_ADC_READ;
  add_cnt++;
  if(3 <= add_cnt)
  { //３回サンプリングの平均
    Serial.printf("TASK: OneShot Proc.\r\n");
    #if(ADC_VCC_MODE)
    Serial.printf("ESP-WROOM VCC: %d[mV]\r\n", (uint16_t)(adc_dfata / 3));
    #else
    int voltage = (int)((float)adc_dfata / 3.0) * ADC_A0_LSB_MV;
    Serial.printf("ESP-WROOM A0(%d): %d[mV]\r\n", (int)(adc_dfata / 3), voltage);
    #endif
    add_cnt = 0;
    adc_dfata = 0;
  }
  doWork_count++; //0.5S に一回カウント
}