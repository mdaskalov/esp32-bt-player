
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Button2.h>

#include "BluetoothA2DPSink.h"
#include "SpectrumAnalyzer.h"

#define BUTTON_1    35
#define BUTTON_2    0

#define TXT_HEIGHT  16

#define	MAXWAVE     220
#define	MAXSPEC     47
#define WAVEHEIGHT  49

#define DIV_STEP    0.2
#define DIV_MAX     650

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library

Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

int w,h;
int wavX,wavY,wavH,spX,spY,spW,spH;
BluetoothA2DPSink a2dp_sink;
SpectrumAnalyzer *spec;

double divisor = 0;
bool simulate = false;

void avrc_metadata_callback(uint8_t data1, const uint8_t *data2) {
  Serial.printf("AVRC metadata rsp: attribute id 0x%x, %s\n", data1, data2);
  if (data1 == 0x1 || data1 == 0x02) {
    if (data1 == 0x1)
      spec->clearSpecView();
    tft.drawString((const char *)data2, tft.width() / 2, tft.height() / 2 - (TXT_HEIGHT/2) + TXT_HEIGHT * (data1 -1));
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
  Serial.println("Starting ESP32 BT Player");

  tft.init();

  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setTextDatum(MC_DATUM);

  w = tft.width();
  h = tft.height();

  wavX = 1;
  wavY = 1;
  wavH = WAVEHEIGHT;

  spX = 0;
  spY = WAVEHEIGHT+1;
  spW = w;
  spH = h-spY;

  tft.fillRect(0,0,w,h,TFT_BLACK);
  spec = new SpectrumAnalyzer(tft,MAXWAVE,MAXSPEC);
  spec->waveView(wavX,wavY,wavH,TFT_WHITE,TFT_RED,TFT_BLACK);
  spec->specView(spX+2,spY+2,spW-4,spH-4,TFT_YELLOW,TFT_BLACK);
  tft.drawString("ESP32 BT Player", w / 2, h / 2);

  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN),
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

  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.start("ESP32 BT Player");

  tft.fillRect(0,0,w,h,TFT_BLACK);
  tft.drawRect(wavX-1,wavY-1,MAXWAVE+2,WAVEHEIGHT+2,TFT_WHITE);
  tft.drawRect(spX,spY,spW,spH,TFT_WHITE);
  spec->clearWaveView();
  spec->clearSpecView();

  btn1.setTapHandler([](Button2 &b) {
    Serial.println("prev (btn1)...");
    a2dp_sink.previous();
  });
  btn2.setTapHandler([](Button2 &b) {
    Serial.println("next (btn2)...");
    a2dp_sink.next();
  });
}

void loop() {
  btn1.loop();
  btn2.loop();

  if (simulate) {
    spec->generateWave(divisor);
    divisor += DIV_STEP;
    if (divisor > DIV_MAX)
      divisor = 0;
  }
}