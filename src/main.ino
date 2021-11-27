#include <M5Core2.h>

#include "BluetoothA2DPSink.h"
#include "SpectrumAnalyzer.h"

#define TXT_HEIGHT  16

#define	MAXWAVE     225
#define	MAXSPEC     63
#define WAVEHEIGHT  80

#define DIV_STEP    0.2
#define DIV_MAX     650

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
    // M5.Lcd.drawString((const char *)data2, M5.Lcd.width() / 2, M5.Lcd.height() / 2 - (TXT_HEIGHT/2) + TXT_HEIGHT * (data1 -1));
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

  M5.begin(true,false,false,false);
  M5.Axp.SetSpkEnable(true);
  M5.Axp.SetLcdVoltage(3300);
  M5.Axp.SetLed(0);

  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_WHITE);
  M5.Lcd.setTextDatum(MC_DATUM);

  w = M5.Lcd.width();
  h = M5.Lcd.height();

  wavX = 1;
  wavY = 1;
  wavH = WAVEHEIGHT;

  spX = 0;
  spY = WAVEHEIGHT+15;
  spW = w;
  spH = h-spY;

  M5.Lcd.fillRect(0,0,w,h,TFT_BLACK);
  spec = new SpectrumAnalyzer(M5.Lcd,MAXWAVE,MAXSPEC);
  spec->waveView(wavX,wavY,wavH,TFT_WHITE,TFT_RED,TFT_BLACK);
  spec->specView(spX+2,spY+2,spW-4,spH-4,TFT_YELLOW,TFT_BLACK);
  M5.Lcd.drawString("Core2 BT Player", w / 2, h / 2);

  i2s_pin_config_t core2_pin_config = {
    .bck_io_num = 12,
    .ws_io_num = 0,
    .data_out_num = 2,
    .data_in_num = I2S_PIN_NO_CHANGE
  };
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
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
  a2dp_sink.set_pin_config(core2_pin_config);
  a2dp_sink.set_i2s_config(i2s_config);
  a2dp_sink.start("Core2 Player");

  M5.Lcd.fillRect(0,0,w,h,TFT_BLACK);
  M5.Lcd.drawRect(wavX-1,wavY-1,MAXWAVE+2,WAVEHEIGHT+2,TFT_WHITE);
  M5.Lcd.drawRect(spX,spY,spW,spH,TFT_WHITE);
  spec->clearWaveView();
  spec->clearSpecView();

  M5.BtnA.addHandler([](Event & e) {
    if (e == E_DBLTAP) {
      Serial.println("previous (btnB)...");
      a2dp_sink.previous();
    }
  });
  M5.BtnB.addHandler([](Event & e) {
    if (e == E_TAP) {
      Serial.println("play (btnB)...");
      a2dp_sink.play();
      spec->clearWaveView();
      spec->clearSpecView();
    }
    if (e == E_DBLTAP) {
      Serial.println("stop (btnB)...");
      a2dp_sink.stop();
    }
  });
  M5.BtnC.addHandler([](Event & e) {
    if (e == E_DBLTAP) {
      Serial.println("next (btnB)...");
      a2dp_sink.next();
    }
  });
}

void loop() {
  M5.update();

  if (simulate) {
    spec->generateWave(divisor);
    divisor += DIV_STEP;
    if (divisor > DIV_MAX)
      divisor = 0;
  }

  M5.Axp.SetLed(a2dp_sink.is_connected());

  if (M5.BtnA.isPressed()) {
    int vol = a2dp_sink.get_volume();
    if (vol > 0)
      vol = vol - 1;
    Serial.printf("vol- (btnA) vol: %d...\n",vol);
    a2dp_sink.set_volume(vol);
    delay(100);
  }
  if (M5.BtnC.isPressed()) {
    int vol = a2dp_sink.get_volume();
    if (vol < 100)
      vol += 1;
    Serial.printf("vol+ (btnC), vol: %d...\n",vol);
    a2dp_sink.set_volume(vol);
    delay(100);
  }
}