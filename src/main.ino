#include "esp_pm.h"

#include "BluetoothA2DPSink.h"
#include "SpectrumAnalyzer.hpp"

#ifdef CORE2
  #include <M5Core2.h>

  #define Graphics M5Display

  #define	MAXWAVE     225
  #define	MAXSPEC     63
  #define WAVEHEIGHT  80
  #define SPEC_Y_OFS  15

  #define DIV_STEP    0.2
  #define DIV_MAX     650

  #define TXT_HEIGHT  16

  #define BT_NAME     "Core2 Player"

  #define ROTATION    1
  #define TXT_SIZE    2

  #define I2S_MODE    (I2S_MODE_MASTER | I2S_MODE_TX)
  #define SPK_ENABLE  true

  #define btn1 M5.BtnA
  #define btn2 M5.BtnC
  #define tft M5.Lcd
#else
  #include <SPI.h>
  #include <TFT_eSPI.h>
  #include <Button2.h>

  #define Graphics TFT_eSPI

  #define BUTTON_1    35
  #define BUTTON_2    0

  #define	MAXWAVE     220
  #define	MAXSPEC     47
  #define WAVEHEIGHT  49
  #define SPEC_Y_OFS  1

  #define DIV_STEP    0.2
  #define DIV_MAX     650

  #define TXT_HEIGHT  16

  #define BT_NAME     "ESP32 BT Player"

  #define ROTATION    3
  #define TXT_SIZE    2

  #define I2S_MODE    (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN)

  Button2 btn1(BUTTON_1);
  Button2 btn2(BUTTON_2);
  TFT_eSPI tft = TFT_eSPI(TFT_WIDTH, TFT_HEIGHT); // Invoke custom library
#endif

int w,h;
int wavX,wavY,wavH,spX,spY,spW,spH;
SpectrumAnalyzer<Graphics> *spec;
BluetoothA2DPSink a2dp_sink;

double divisor = 0;
bool simulate = false;

void avrc_metadata_callback(uint8_t data1, const uint8_t *data2) {
  Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
  if (data1 == 0x1 || data1 == 0x02) {
    if (data1 == 0x1)
      spec->clearSpecView();
    //tft.drawString((const char *)data2, tft.width() / 2, tft.height() / 2 - (TXT_HEIGHT/2) + TXT_HEIGHT * (data1 -1));
  }
}

void read_data_stream(const uint8_t *data, uint32_t length)
{
  // int16_t sample_l_int = (int16_t)((data[ofs*4 + 1] << 8) | data[ofs*4 + 0]);
  // int16_t sample_r_int = (int16_t)((data[ofs*4 + 3] << 8) | data[ofs*4 + 2]);
  int ofs=1;
  while (ofs < length) {
    int8_t sample1 = (int8_t)data[ofs]   >> 2;
    int8_t sample2 = (int8_t)data[ofs+4] >> 1;
    int8_t sample3 = (int8_t)data[ofs+8] >> 2;
    spec->addSample(sample1+sample2+sample3); // downsample
    ofs += 12; // 4 * samples
  }
  spec->draw();
}

void setup() {
  Serial.begin(115200);

  // highest clockspeed needed
  esp_pm_lock_handle_t powerManagementLock;
  esp_pm_lock_create(ESP_PM_CPU_FREQ_MAX, 0, "PowerManagementLock", &powerManagementLock);
  esp_pm_lock_acquire(powerManagementLock);

  Serial.println("Starting " BT_NAME);

#ifdef CORE2
  M5.begin(true,false,false,false);
  M5.Axp.SetSpkEnable(SPK_ENABLE);
  M5.Axp.SetLcdVoltage(3300);
  M5.Axp.SetLed(0);

  M5.BtnA.addHandler([](Event & e) {
    if (e == E_DBLTAP) {
      Serial.println("previous (btn1)...");
      a2dp_sink.previous();
    }
  });
  M5.BtnB.addHandler([](Event & e) {
    if (e == E_TAP) {
      Serial.println("play (btnB)...");
      // a2dp_sink.play();
      spec->clearWaveView();
      spec->clearSpecView();
    }
    if (e == E_DBLTAP) {
      Serial.println("stop (btnB)...");
      // a2dp_sink.stop();
    }
  });
  M5.BtnC.addHandler([](Event & e) {
    if (e == E_DBLTAP) {
      Serial.println("next (btn2)...");
      a2dp_sink.next();
    }
  });

  // Route output to speaker
  i2s_pin_config_t core2_pin_config = {
    .bck_io_num = 12,
    .ws_io_num = 0,
    .data_out_num = 2,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  a2dp_sink.set_pin_config(core2_pin_config);
#else
  tft.init();

  btn1.setDoubleClickHandler([](Button2 &b) {
    Serial.println("prev (btn1)...");
    a2dp_sink.previous();
  });
  btn2.setDoubleClickHandler([](Button2 &b) {
    Serial.println("next (btn2)...");
    a2dp_sink.next();
  });
#endif
  tft.setRotation(ROTATION);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(TXT_SIZE);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);

  w = tft.width();
  h = tft.height();

  wavX = 1;
  wavY = 1;
  wavH = WAVEHEIGHT;

  spX = 0;
  spY = WAVEHEIGHT+SPEC_Y_OFS;
  spW = w;
  spH = h-spY;

  spec = new SpectrumAnalyzer<Graphics>(tft,MAXWAVE,MAXSPEC);
  spec->waveView(wavX,wavY,wavH,TFT_WHITE,TFT_RED,TFT_BLACK);
  spec->specView(spX+2,spY+2,spW-4,spH-4,TFT_YELLOW,TFT_BLACK);

  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t) I2S_MODE,
    .sample_rate = 44100, // corrected by info from bluetooth
    .bits_per_sample = (i2s_bits_per_sample_t) 16, // the DAC module will only take the 8bits from MSB
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_MSB,
    .intr_alloc_flags = 0, // default interrupt priority
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true // avoiding noise in case of data unavailability
  };

  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_stream_reader(read_data_stream);

  a2dp_sink.set_mono_downmix(true);
  a2dp_sink.set_volume(80);
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.start(BT_NAME);

  tft.fillRect(0,0,w,h,TFT_BLACK);
  tft.drawRect(wavX-1,wavY-1,MAXWAVE+2,WAVEHEIGHT+2,TFT_WHITE);
  tft.drawRect(spX,spY,spW,spH,TFT_WHITE);
  spec->clearWaveView();
  spec->clearSpecView();
}

void loop() {
#ifdef CORE2
  M5.update();
  M5.Axp.SetLed(a2dp_sink.is_connected());
#else
  btn1.loop();
  btn2.loop();
#endif
  if (btn1.isPressed()) {
    int vol = a2dp_sink.get_volume();
    if (vol > 0)
      vol = vol - 1;
    Serial.printf("vol- (btn1) vol: %d...\n",vol);
    a2dp_sink.set_volume(vol);
    delay(100);
  }
  if (btn2.isPressed()) {
    int vol = a2dp_sink.get_volume();
    if (vol < 100)
      vol += 1;
    Serial.printf("vol+ (btn2), vol: %d...\n",vol);
    a2dp_sink.set_volume(vol);
    delay(100);
  }

  if (simulate) {
    spec->generateWave(divisor);
    divisor += DIV_STEP;
    if (divisor > DIV_MAX)
      divisor = 0;
  }
}