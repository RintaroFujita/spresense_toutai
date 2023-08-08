#include <MP.h>
#include <Audio.h>
#include <SpresenseNeoPixel.h>


//////////////////////////////////////////////////////////////////////////
// LEDテープの初期設定

const uint16_t PIN = 6;            // データ用のピンを指定
const uint16_t NUM_PIXELS = 24;    // 点灯させるLEDの数を指定

//////////////////////////////////////////////////////////////////////////
// 色の初期設定

float hue = 0.0;                  // 色相（0.0〜1.0）
float bright = 0.5;               // 明るさ（0.0〜1.0）
float saturation = 1.0;           // 彩度（0.0〜1.0）

//////////////////////////////////////////////////////////////////////////
// 音の初期設定

const int PITCH_NUM = 48;         // 検出する音の数
const int LENGTH_NUM = 2;         // 検出する音の長さ

// 検出する周波数のしきい値テーブル
int freqThreshold[PITCH_NUM] = {70,90,110,130,150,170,190,210,230,250,270,290,310,330,350,370,390,410,430,450,470,490,510,530,550,570,590,610,630,650,670,690,710,730,750,770,790,810,830,850,870,890,910,930,950,970,990,1200};

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////





// LEDの設定
SpresenseNeoPixel<PIN, NUM_PIXELS> neopixel;
char pixelArry[NUM_PIXELS];       // LEDテープを制御するための配列
int col[3];                       // 色を格納する配列
int intervalCount1 = 0;           // LEDの処理待ちインターバル用のカウント
bool intervalFlg = false;         // LEDの処理待ちインターバル用のフラグ
bool ledOnFlg = false;            // LED点灯フラグ
bool fadeOnFlg = false;           // フェードアウト用のフラグ
float prevHue = 0.0;
float prevBright = 0.0;
int intervalCount2 = 0;           // フェードアウト終了後に確実に消灯する用のカウント
bool clearFlg = false;            // フェードアウト終了後に確実に消灯する用のフラグ


// 音の設定
int foundPitchState = 0;          // 見つけた音のステート
int foundPitchCount[PITCH_NUM];   // 周波数ごと回数カウント配列
int foundVolume = 0;              // 音のボリューム
bool foundOnFlg = false;

//////////////////////////////////////////////////////////////////////////


AudioClass *theAudio;

/* Select mic channel number */
const int mic_channel_num = 1;
//const int mic_channel_num = 2;
//const int mic_channel_num = 4;

const int subcore = 1;

struct Request {
  void *buffer;
  int  sample;
  int  channel;
};

struct Result {
  float peak[mic_channel_num];
  int volume[mic_channel_num];
  int  channel;
};


void setup()
{
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Init Audio Library");
  theAudio = AudioClass::getInstance();
  theAudio->begin();

  Serial.println("Init Audio Recorder");
  /* Select input device as AMIC */
  theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC, 21); // 入力ゲインを上げる場合はこちら
  // theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC);

  /* Set PCM capture */
  uint8_t channel;
  switch (mic_channel_num) {
  case 1: channel = AS_CHANNEL_MONO;   break;
  case 2: channel = AS_CHANNEL_STEREO; break;
  case 4: channel = AS_CHANNEL_4CH;    break;
  }
  theAudio->initRecorder(AS_CODECTYPE_PCM, "/mnt/sd0/BIN", AS_SAMPLINGRATE_48000, channel);

  /* Launch SubCore */
  int ret = MP.begin(subcore);
  if (ret < 0) {
    printf("MP.begin error = %d\n", ret);
  }
  /* receive with non-blocking */
  MP.RecvTimeout(1);

  Serial.println("Rec start!");
  theAudio->startRecorder();

  neopixel.clear();
  neopixel.framerate(40);
}

void loop()
{
  int8_t   sndid = 100; /* user-defined msgid */
  int8_t   rcvid = 0;
  Request  request;
  Result*  result;

  static const int32_t buffer_sample = 768 * mic_channel_num;
  static const int32_t buffer_size = buffer_sample * sizeof(int16_t);
  static char  buffer[buffer_size];
  uint32_t read_size;

  /* Read frames to record in buffer */
  int err = theAudio->readFrames(buffer, buffer_size, &read_size);

  if (err != AUDIOLIB_ECODE_OK && err != AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA) {
    printf("Error err = %d\n", err);
    sleep(1);
    theAudio->stopRecorder();
    exit(1);
  }
  if ((read_size != 0) && (read_size == buffer_size)) {
    request.buffer   = buffer;
    request.sample = buffer_sample / mic_channel_num;
    request.channel  = mic_channel_num;
    MP.Send(sndid, &request, subcore);
  } else {
    /* Receive detector results from SubCore */
    int ret = MP.Recv(&rcvid, &result, subcore);
    if (ret >= 0) {
      for (int i=0;i<mic_channel_num;i++) {
        printf("%8.3f, ", result->peak[i]);
      }
      printf("\n");


      //
      // 音のピーク周波数を取得して、LEDを点灯させる
      //
      float foundPeak = result->peak[0];

      // 入力した音のボリュームを取得（使っていない）
      if (result->volume[0] < 1000) {
        if (result->volume[0] < 1000) {
          foundVolume = result->volume[0];
        }
      }
      
      for (int j=0; j<PITCH_NUM; j++) {

        // しきい値で分けた周波数の配列を一つずつ比較し、取得したピーク周波数に当てはまるならカウントする
        if(foundPeak > freqThreshold[j] && foundPeak <= freqThreshold[j+1]) {

          // 当てはまった周波数の配列にカウント足し、それ以外のカウントは0に戻す
          int tmpCount = foundPitchCount[j];
          memset(foundPitchCount, 0, sizeof foundPitchCount);
          foundPitchCount[j] = tmpCount + 1;
        }
      }

      // 指定したしきい値以外の場合は、全てのカウントを0に戻す
      if(foundPeak < freqThreshold[0] || foundPeak > freqThreshold[PITCH_NUM-1]) {
        memset(foundPitchCount, 0, sizeof foundPitchCount);
        foundOnFlg = false;
      }
      
      // 前回と同じ配列のカウントが進む→続けて音が鳴っているとし、指定の長さ分カウントが続いたら、音を見つけたと定義する
      for (int k=0; k<PITCH_NUM; k++) {
        if(foundPitchCount[k] > LENGTH_NUM) {
          //printf("FOUND %d, %d, %d, %d, %d, ", k+1, foundPitchCount[0], foundPitchCount[1], foundPitchCount[2], foundPitchCount[3]);
          printf("FOUND %d, ", k+1, foundPitchCount[k]);
          foundPitchState = k+1;  // 音のステートfoundPitchStateに値を指定
          foundOnFlg = true;
        }
      }

      // 指定したしきい値以外の場合は、foundPitchStateに0を指定
      if (!foundOnFlg) {
        foundPitchState = 0;
      }

      // LEDを点灯
      ledControl();
    }

    
  }
}



//
// テープLEDを点灯
//
void ledControl() {

  // LEDのインターバル待ち間は点灯処理はしない
  if(!intervalFlg) {

    // 音のステートに合わせてLEDをを点灯
    for (int i=1; i<PITCH_NUM; i++) {
      if (foundPitchState == i) {
//        if (!ledOnFlg) {
          // 色相を音のステートごとにマッピング
          hue = map(i, 0, PITCH_NUM-1, 100, 3) / 100.0;
          hsv2rgb(hue, saturation, bright, col);
          neopixel.set(col[0], col[1], col[2]);
          neopixel.show();
          
          ledOnFlg = true;
          intervalFlg = true;
          fadeOnFlg = false;
          clearFlg = false;
//        }
      }
    }

    // LEDを消す
    if (foundPitchState == 0) {
      if (ledOnFlg) {
        ledOnFlg = false;
        intervalFlg = true;
        fadeOnFlg = true;
        prevHue = hue;
        prevBright = bright;
      }
    }

    // 消す前にフェードアウトさせる
    if(fadeOnFlg) {
      if(prevBright > 0.0) {
        prevBright -= 0.0125;
        hsv2rgb(prevHue, saturation, prevBright, col);
        neopixel.set(col[0], col[1], col[2]);
        neopixel.show();
      } else {
        // フェードが終わったらLEDを消す
        neopixel.set(0, 0, 0); // 消灯（黒を指定）
        neopixel.show();
        fadeOnFlg = false;
        clearFlg = true;
      }
      
    }
  }

  // 速いとLEDが反応してなさそうなので、インターバルを設定
  if(intervalFlg) {
    intervalCount1 += 1;
    if (intervalCount1 > 4) {
      intervalCount1 = 0;
      intervalFlg = false;
    }
  }

  // フェードアウト後にLEDが消えないことがあるので、インターバルを設定して確実に消す
  if(clearFlg) {
    intervalCount2 += 1;
    if (intervalCount2 > 8) {
      neopixel.set(0, 0, 0);
      neopixel.show();
      intervalCount2 = 0;
      clearFlg = false;
    }
  }

}


//
// HSV->RGB / RGB->HSV 
// https://gist.github.com/postspectacular/2a4a8db092011c6743a7
// 
float fract(float x) { return x - int(x); }
float mix(float a, float b, float t) { return a + (b - a) * t; }
float step(float e, float x) { return x < e ? 0.0 : 1.0; }

int* hsv2rgb(float h, float s, float b, int* rgb) {
  float rgbf[3];
  rgbf[0] = b * mix(1.0, constrain(abs(fract(h + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  rgbf[1] = b * mix(1.0, constrain(abs(fract(h + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  rgbf[2] = b * mix(1.0, constrain(abs(fract(h + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  rgb[0] = rgbf[0] * 255;
  rgb[1] = rgbf[1] * 255;
  rgb[2] = rgbf[2] * 255;
  return rgb;
}

float* rgb2hsv(float r, float g, float b, float* hsv) {
  float s = step(b, g);
  float px = mix(b, g, s);
  float py = mix(g, b, s);
  float pz = mix(-1.0, 0.0, s);
  float pw = mix(0.6666666, -0.3333333, s);
  s = step(px, r);
  float qx = mix(px, r, s);
  float qz = mix(pw, pz, s);
  float qw = mix(r, px, s);
  float d = qx - min(qw, py);
  hsv[0] = abs(qz + (qw - py) / (6.0 * d + 1e-10));
  hsv[1] = d / (qx + 1e-10);
  hsv[2] = qx;
  return hsv;
}



/*
 *  MainAudio.ino - FFT Example with Audio (Peak detector)
 *  Copyright 2019 Sony Semiconductor Solutions Corporation
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
