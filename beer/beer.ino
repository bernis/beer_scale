////////////////////////////////////////
//
// Beer Scale v1.0
// Bernhard Stampfer 2024
//
////////////////////////////////////////

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "HX711.h"           // scale adc
#include <TonePlayer.h>
#include "AiEsp32RotaryEncoder.h"
#include "ESP32_WS2812_Lib.h"
//#include "driver/ledc.h"
//#include "hal/ledc_types.h"
#include <esp_now.h>
#include <WiFi.h>
#include "beer_gfx.h"

//#define DEBUG

// DISPLAY (Aliexpress 1.77" TFT)
// https://github.com/adafruit/Adafruit-ST7735-Library
#define TFT_CS0     -1 // manual
#define TFT_CS1     16
#define TFT_CS2     17
#define TFT_CS3     21
#define TFT_CS4     22
#define TFT_RST     -1 // reset on reset
#define TFT_DC      5
uint8_t TFT_BL[5] = {0, 13, 25, 32, 33};
// MOSI, SCK using hardware VSPI 23/18
Adafruit_ST7735 tfts[5] = { {TFT_CS0, TFT_DC, TFT_RST}, // fake tft, used to print on all
                            {TFT_CS1, TFT_DC, TFT_RST},
                            {TFT_CS2, TFT_DC, TFT_RST},
                            {TFT_CS3, TFT_DC, TFT_RST},
                            {TFT_CS4, TFT_DC, TFT_RST} };

// SCALE (Aliexpress 10KG Load Cell with green HX711 board)
// https://github.com/RobTillaart/HX711
#define SCALE_MISO  27//12
#define SCALE_CLK   26//14
// MISO, SCK using hardware HSPI? -- hardware spi not working as pin 14 can't be high on boot
HX711 scale;

// ROTARY ENCODER/BUTTON (AliExpress EC11 15mm plum handle, https://de.aliexpress.com/item/1005005983134515.html)
// https://github.com/igorantolic/ai-esp32-rotary-encoder
#define ROTARY_A GPIO_NUM_2
#define ROTARY_B GPIO_NUM_4
#define ROTARY_T GPIO_NUM_15
// library seems unstable (v1.7) but can handle pulldown encoder with interrupt, high active button not working, handled manually
AiEsp32RotaryEncoder rotary = AiEsp32RotaryEncoder(ROTARY_A, ROTARY_B, -1, -1, 4, true);

// PIEZO SPEAKER (random part)
// https://github.com/ZulNs/TonePlayer
#define PIEZO_P 14
// library seems hardcoded to ledc timer0 channel0, which requires some workaround for display pwm and light sleep mode
TonePlayer piezo(PIEZO_P);

// WS2812 LED RING, 24 LEDs
// https://github.com/Zhentao-Lin/ESP32_WS2812_Lib
#define LED_DI 12
// how much current does this thing need in standby? additional pin to switch 5V line with a mosfet?
ESP32_WS2812 leds = ESP32_WS2812(24, LED_DI, 0);

// GLOBALS
// scale measure
float raw[5] = {0};     //g  weight raw value buffer
uint8_t pos = 0;        //g  weight raw value buffer position
float avg = 0;          //g  weight       fir/iir filtered
float avg_l = 0;        //g  weight, last
float davg = 0;         //g  weight difference
float tar = 0;          //g  tara value   fir/iir filtered below 2g

// gui / hardware
float disp_avg = 0;     //g  weight, remote or local avg
float disp_avg_l = 0;   //g  weight, remote or local avg
float disp_stable = 0;  //g  weight, remote or local avg
uint8_t disp_remote = 0;//   display remote weight
uint8_t disp_tar = 0;   //   display tare changing, boolean
uint8_t disp_fix = 0;   //   display fix, boolean
uint32_t tlast;         //ms loop timing
uint8_t sleep_en = 1;   //   allow sleeping in main loop
uint8_t btn_hist = 0;   //   last button states f. debouncing
uint8_t menu = 0;       //   which menu we are in, 0=none
uint8_t bri_tft = 7;    //   0..7
uint8_t bri_led = 5;    //   0..7
uint8_t volume = 7;     //   0..7 -> 0..255 ()

// esp now
uint32_t peer_start = 0; //ms start time of peering 
uint8_t peer_last = 0;  //ms last time
uint8_t peer_found = 0; //   we found a second scale
float avg_rec = 0;      //g  weight, received from other scale
float stable_rec = 0;   //   ratio of stable time to STABLE_t_1, received
uint8_t greeting[]  = "Bier mit mir?!";
uint8_t intro[]     = "BS1.0";
uint8_t broadcast[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

// text
#define TXT_L 10
#define TXT_T 52
uint16_t FG[5] = {ST77XX_BLACK, ST77XX_BLACK, ST77XX_WHITE, ST77XX_WHITE, ST77XX_BLACK};
uint16_t BG[5] = {ST77XX_WHITE, ST77XX_GREEN, ST77XX_RED,   ST77XX_BLUE,  ST77XX_YELLOW};

// tara sign
#define TAR_L 159 - 6*3 - 4 //137 to 154
#define TAR_T 52            

// get stable value
#define STABLE_L   5    
#define STABLE_T   45 
#define STABLE_W   150
#define STABLE_H   3 
#define STABLE_t_0 200    //ms start displaying bar 700
#define STABLE_t_1 2200   //ms consider value stable
uint32_t stable_ms = 0;   //ms time without change
int16_t  stable_avg;      //g  stable value, full integers
uint8_t  stable_x_l = 0;  //px last drawn x value
uint8_t  stable_xled_l = 0;//px last drawn x value on leds
float    stable = 0;      //   ratio of stable time to STABLE_t_1

// scale display
#define BGSCALE ST77XX_BLACK
#define FGSCALE ST77XX_WHITE
#define SCALE_L 0
#define SCALE_R 160
#define SCALE_T 128-41
#define SCALE_B 128-8
#define SCALE_M 80.5       //px "center" of scale for calculation
#define ZOOM    3          //px/g zoom factor for scale width
uint8_t gfx_h[100] = {0};  //px scale line heights, indexed in rounded g

// beer display
#define BEER_T  4
#define BEER_L  10
uint16_t BEER_m_0[3]  = {16,  380, 345};  //g  mass of empty beer
uint16_t BEER_m_1[3]  = {516, 880, 845};  //g  mass of full beer
uint8_t BEER_x_1[3]   = {85,  112, 100};  //px x of full beer
uint8_t BEER_x_M[3]   = {95,  140, 120};  //px array length
uint16_t BEER_c_w[3]  = {0x8C51, 0x0000, 0x7161};  //   wall color
uint16_t BEER_c_e[3]  = {0xCE37, 0xCB20, 0xBAE4};  //   empty color
uint16_t BEER_c_b[3]  = {0xFE21, 0xFCC2, 0xE487};  //   beer color
uint16_t BEER_c_f[3]  = {0xFFFF, 0xEE96, 0xEDF2};  //   foam color
uint8_t beer_disp = 255;//   0..can, 1..nrw, 2..eur, .., 255..disable
uint8_t beer_x_l = 0;   //px last drawn x value 
uint8_t *beer_gfx;      //   points to correct beer graphic
uint16_t *beer_gfx_img; //   points to correct beer graphic bitmap

// bubbles
#define BUBBLE_N    20      //   number of bubbles
uint8_t bubble_x[BUBBLE_N] = {0}; //px x positions of bubbles
uint8_t bubble_y[BUBBLE_N] = {0}; //px y positions ob bubbles

// others
#define DINIT   F("displays initialized")
#define SINIT   F("scale initialized")
#define GINIT   F("graphics initialized")
uint16_t BR10[] = {1, 3, 8, 20, 60, 180, 400, 1023};
uint8_t  BR8[]  = {0, 1, 2, 5,  15, 45,  100, 255};

const PROGMEM uint8_t SONG_START[] = { NC6, 256-2, ND6, 256-2, NE6,  256-2, NF5,   256-2};
const PROGMEM uint8_t SONG_FIX[]   = { NC6, 256-4 };
const PROGMEM uint8_t SONG_420[]   = {
  NGIS5, 256-32, NGIS5, 256-32, NGIS5, 256-32, NGIS5, 256-32, NGIS5, 256-16, NGIS5, 256-32,
  NGIS5, 256-8,  NGIS5, 256-32, NGIS5, 256-32, NGIS5, 256-32, NG5, 256-16, NF5, 0, 3, 8, 
  NGIS5, 256-32, NGIS5, 256-32, NGIS5, 256-32, NAIS5, 256-32, NAIS5, 256-16, NAIS5, 256-32, 
  NAIS5, 256-32, NAIS5, 256-32, NAIS5, 256-32, NAIS5, 256-8,  NG5, 256-32, NG5, 256-32, //50
  NG5, 256-32, NF5, 256-16, NDIS5, 0, 3, 8, NAIS5, 256-8, NC6, 256-32, NC6, 256-32, 
  NC6, 256-32, NC6, 256-16, NAIS5, 256-16, NGIS5, 256-16, NGIS5, 256-16, NGIS5, 256-32,  
  NDIS6, 256-32, NDIS6, 256-32, NC6, 256-32, NDIS6, 256-32, NCIS6, 256-4, NF6, 0, 3, 32,//90 
  NF6, 256-32, NDIS6, 256-32, NDIS6, 256-32, NCIS6, 256-16, NC6, 0, 3, 32, NC6, 256-32, 
  NCIS6, 256-32, NCIS6, 256-32, NGIS5, 265-16, NAIS5, 0, 3, 32, NGIS5, 256-32,  NAIS5, 256-32, 
  NAIS5, 256-32, NG5, 256-32, NAIS5, 256-16, NGIS5, 0, 3, 16, NDIS6, 256-32, NF6, 256-32,
  NDIS6, 256-16, NCIS6, 256-16, NC6, 256-32, NC6, 256-32, NAIS5, 256-16, REPEAT, 0, 0, 142, 0
};
//RTTTL: x:b=50,o=5,d=32:p,g#,g#,g#,g#,16g#,g#,8g#,g#,g#,g#,16g,4f.,g#,g#,g#,a#,16a#,a#,a#,a#,a#,8a#,g,g,g,16f,4d#.,8a#,c6,c6,c6,16c6,16a#,16g#,16g#,g#,d#6,d#6,c6,d#6,4c#6,16f.6,f6,d#6,d#6,16c#6,16c.6,c6,c#6,c#6,16g#,16a#.,g#,a#,a#,g,16a#,8g#.,d#6,f6,16d#6,16c#6,c6,c6,16a#,

// CODE
void IRAM_ATTR readEncoderISR(){
    rotary.readEncoder_ISR();
}

void setup() {
  btStop();
  //#ifdef DEBUG
  Serial.begin(115200);
  //#endif

  // LEDs
  leds.begin();
  leds.setLedColor(0, 0, 255, 0);

  // BACKLIGHTS
  // timer 0 used for sound, attach backlight pwm to channels using other timers
  for(uint8_t i=1; i<5; i++){
    pinMode(TFT_BL[i], OUTPUT);
    ledcAttachChannel(TFT_BL[i], 100, 10, 9+i); // 10,11 -> T1 12,13 -> T2
    ledcWrite(TFT_BL[i], 0);
    leds.setLedColor(i, 0, 255, 0);
  }
  // set timers 1,2 to use low speed rc clock which runs during light sleep
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
                                    .timer_num = LEDC_TIMER_1, .freq_hz = 100, .clk_cfg = LEDC_USE_RC_FAST_CLK};
  ledc_timer_config(&ledc_timer);
  ledc_timer.timer_num = LEDC_TIMER_2;
  ledc_timer_config(&ledc_timer);
  leds.setLedColor(5, 0, 255, 0);
  
  // INIT PIEZO
  //pinMode(PIEZO_P, OUTPUT);
  piezo.setOnToneCallback(onTone);
  piezo.setOnMuteCallback(onMute);
  piezo.setSong((uint8_t*)SONG_START, sizeof(SONG_START), 175);
  piezo.play();
  leds.setLedColor(6, 0, 255, 0);

  // INIT DISPLAYS
  pinMode(TFT_CS1, OUTPUT);
  pinMode(TFT_CS2, OUTPUT);
  pinMode(TFT_CS3, OUTPUT);
  pinMode(TFT_CS4, OUTPUT);
  tft_cs_all(HIGH);
  for(uint8_t i=0; i<5; i++){
    tfts[i].setSPISpeed(10000000);
    tfts[i].initR(INITR_BLACKTAB);
    tfts[i].setRotation(1);
    tfts[i].setTextWrap(false);
    if(i>0){
      tfts[i].setTextColor(FG[i], BG[i]);
      //tfts[i].fillScreen(BG[i]);
      tfts[i].setTextSize(3);
    }else{
      tfts[i].setTextColor(FG[i]);
      tfts[i].setTextSize(1);
    }
    leds.setLedColorData(7+i*3, 0, 255, 0);
    leds.setLedColorData(8+i*3, 0, 255, 0);
    leds.setLedColorData(9+i*3, 0, 255, 0);
    leds.show();
  }
  
  tft_print_all(DINIT, 2, 2);
  #ifdef DEBUG
    Serial.println(DINIT);
  #endif

  // INIT SCALE ADC, MEASURE INITIAL TARE
  scale.begin(SCALE_MISO, SCALE_CLK);
  scale.set_scale(222.22); // //n:222.22 //b:220.56
  tar = scale.get_units(10); // bad first value?
  avg = tar = scale.get_units(10);

  tft_print_all(SINIT, 2, 10);
  #ifdef DEBUG
    Serial.println(SINIT);
  #endif
  leds.setLedColor(23, 0, 255, 0);

  // INIT ROTARY
  rotary.begin();
  rotary.setup(readEncoderISR);
  rotary.setAcceleration(0);
  rotary.correctionOffset=0;
  pinMode(ROTARY_T, INPUT_PULLDOWN);

  for(uint8_t i=0; i<24; i++)
    leds.setLedColorData(i, 0, 0, 0);
  leds.show();

  // INIT GRAPHICS
  gfx_init();

  init_main();
  
  for(uint8_t i=1; i<5; i++)
        ledcWrite(TFT_BL[i], BR10[bri_tft]);
  
  #ifdef DEBUG
    Serial.println(GINIT);
  #endif

  //gpio_dump_all_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
  connect();
}

void init_main() {
  menu = 0;
  rotary.setBoundaries(0, 7, false);
  rotary.setEncoderValue(bri_tft);

  for(uint8_t i = 1; i < 5; i++)
    tfts[i].fillScreen(BG[i]);

  // draw scale background
  tft_cs_all(LOW);
  tfts[0].fillRect(0, 128-48, 160, 48, BGSCALE);      
  tfts[0].drawFastHLine(0, 128-49, 160, FGSCALE);
  tfts[0].drawFastVLine(SCALE_M, 128-48, 19, ST77XX_RED);
  tfts[0].drawFastVLine(SCALE_M, 128-20, 20, ST77XX_RED);
  tft_cs_all(HIGH);

  tlast = millis();

  // draw fixed text
  tft_print_all_it("    . g", TXT_L, TXT_T, 3);

  tft_print_all(GINIT, 2, 18);

  // draw placeholder image
  beer_disp = 255;
  for(uint8_t i = 1; i < 5; i++)
    tfts[i].fillRect(0, 0, 160, BEER_T + 36, BG[i]);

  uint16_t* c = (uint16_t*)BEER_LOGO_IMG;
  tft_cs_all(LOW);
  for(uint8_t y = 2; y < 44; y++)
    for(uint8_t x = 160-70; x < 160-70+64; x++){
      if(*c != 0xF81F) //magenta
        tfts[0].writePixel(x, y, *c);
      c++;
    }
  tfts[0].fillRect(6, 20, 160-70-6, 2, 0x047a);
  tft_cs_all(HIGH);
  if(beer_disp != 255)
    beer_init();
}

void loop() {

  // measure and average
  measure();

  // esp now
  if(((millis() - peer_start) < 10000) || peer_found)
    send_greeting();
  else if(peer_start > 0)
    disconnect();
  if(peer_found && !disp_tar)
    send_weight();

  // draw main - menu only drawn on input
  if(menu == 0){
    // new text
    txt_draw();
    // graphics
    stable_draw(); // determines FIX
    gfx_draw();
    beer_draw();
  }

  // input, music stuff
  piezo.loop();
  if (rotary.encoderChanged()) rotary_cb(rotary.readEncoder());
  button_loop();

  // frame rate limiter, use sleep to save some battery if possible
  if(33 - min(33lu, millis() - tlast) > 0){
    if(1)//!sleep_en)
      delay(33 - min(33lu, millis() - tlast));
    else{
      gpio_wakeup_enable(ROTARY_A, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable(ROTARY_B, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable(ROTARY_T, GPIO_INTR_HIGH_LEVEL);
      esp_sleep_enable_gpio_wakeup();
      if(peer_found || peer_start){
        esp_now_deinit();
        WiFi.mode(WIFI_OFF);
      }
      esp_sleep_enable_timer_wakeup(1000*(33-min(33lu, millis() - tlast)));
      //Serial.printf("dt2: %d\n", millis() - tlast);
      esp_light_sleep_start(); 
      if(peer_found || peer_start) {
        connect();
      }
    }
  }
  uint32_t tnow = millis();
 
  #ifdef DEBUG
    Serial.printf("dt: %d\n", tnow - tlast);
  #endif
  tlast = tnow;
  }

// weight calculation, tar update, stability update ----------
void measure(){
  // read value
  raw[pos] = scale.get_units(1) - tar;
  #ifdef DEBUG
    Serial.printf("raw: %fg\n", raw[pos]);
  #endif
  pos = (pos+1) % 5;

  // fir moving average/outlier filter
  float vmin = 1e7, vmax = -1e7, val = 0;
  for(int i=0; i<5; i++){
    if(raw[i] < vmin) vmin = raw[i];
    if(raw[i] > vmax) vmax = raw[i];
    val += raw[i];
  }
  val = (val - vmin - vmax) / 3;
  
  // iir ema with increased sample weight for large differences
  davg = 0.9 * davg + 0.1 * (val - avg);  //g change
  float dnoise = 0.5;                     //g expected sample standard deviation in davg
  float alpha = 0.009 + 0.180 * ((fabs(davg)/dnoise))/((fabs(davg)/dnoise)+1);
  
  //float delta = millis()-stable_ms;
  //float alpha = 0.01 + 0.15 * max(0.0f, min(1.0f, (float)STABLE_t_0 * STABLE_t_0 / delta / delta ));
  //if(fabs(avg) < 2) alpha = 0.04;
  avg_l = avg;
  avg = (1-alpha) * avg + alpha * val;

  // iir tare below 2g to account for beer sweat on scale
  if(fabs(avg) < 10.0 && fabs(val) < 10.0){
    tar = tar + 0.01 * val;
    if(disp_tar == 0){
      tft_print_all_it("TAR", TAR_L, TAR_T, 1);
      for(uint8_t i=0; i<24; i++)
        leds.setLedColorData(i, 0, 0, BR8[bri_led] / 32); // blue
      leds.show();
      send_weight();
      disp_tar = 1;
    }
  }
  else{
    if(disp_tar == 1){
      tft_print_all_it("   ", TAR_L, TAR_T, 1);
      for(uint8_t i=0; i<24; i++)
        leds.setLedColorData(i, BR8[bri_led] / 2, BR8[bri_led] / 2, 0); // yellow
      leds.show();
      disp_tar = 0;
    }
  }

  // reset stable time if value changed
  // based on rate of change
  uint16_t sta = (uint16_t)floor(round(avg*10)/10);
  if((fabs(avg) < 10.0) || fabs(davg) > 0.5){
    stable_ms = millis();
    stable_avg = sta;
  }
  stable = float(millis() - stable_ms) / STABLE_t_1;
}

// draws the weight, local or remote
void txt_draw(){
  char txt[19];

  disp_avg = avg;
  disp_stable = stable;
  if(peer_found){
    if(avg_rec > 10.0 && disp_tar){
      if(disp_remote != 1){
        tft_print_all_it(" ", 4, TAR_T+8, 1);
        tft_print_all_it("R", 4, TAR_T+16, 1);
        disp_remote = 1;
        for(uint8_t i=0; i<24; i++)
        leds.setLedColorData(i, BR8[bri_led] / 2, BR8[bri_led] / 2, 0); // yellow
        leds.show();
      }
      else
        // otherwise remote avg not drawn
        disp_stable = stable_rec; 
      disp_avg = avg_rec;
    }
    else{
      if(disp_remote != 0){
        tft_print_all_it("L", 4, TAR_T+8, 1);
        tft_print_all_it(" ", 4, TAR_T+16, 1);
        disp_remote = 0;
        disp_tar = 0; //reprints TAR
      }
    }
  }

  if(round(10*disp_avg) == round(10*disp_avg_l)) 
    return;

  if(disp_avg <= -1000)
    sprintf(txt, " low", disp_avg);
  else{
    if(disp_avg > 4000)
      sprintf(txt, "high", disp_avg);
    else{
        sprintf(txt, "%6.1f", disp_avg);
    }
  }

  if(disp_stable < 1)
    tft_print_all_it(txt, TXT_L, TXT_T, 3);
  disp_avg_l = disp_avg;
}

// draws the stable line,  leds and FIX/420 text ----------
void stable_draw(){
  // bar
  uint8_t x = max(0.0f, min((float)STABLE_W, STABLE_W * disp_stable));
  for(uint8_t i=1; i<5; i++){
    if(x > stable_x_l){
      tfts[i].fillRect(STABLE_L + stable_x_l, STABLE_T, x - stable_x_l, STABLE_H, FG[i]);
    }
    else if (x < stable_x_l){ // reset
      tfts[i].fillRect(STABLE_L + x,          STABLE_T, stable_x_l - x, STABLE_H, BG[i]);
    }
  }

  // same for leds
  uint8_t xled = max(0.0f, min((float)25, 25 * disp_stable));
  if(xled > stable_xled_l){
    for(uint8_t i=stable_xled_l; i<xled; i++){
        //leds.setLedColorData(i, int((5-i)/5.0*255), int(i/5.0*255), 0);
        leds.setLedColorData(i, BR8[bri_led] / 2, 0, BR8[bri_led] / 2); // purple
        }
     leds.show();
  }
  else if (x < stable_x_l){ // reset
    for(uint8_t i=0; i<24; i++)
        leds.setLedColorData(i, BR8[bri_led] / 2, BR8[bri_led] / 2, 0); // yellow
    leds.show();
  }

  // logos
  if(disp_stable >=1 ){
    if(disp_fix == 0){
      tft_print_all_it("FIX", TAR_L, TAR_T, 1);
      for(uint8_t i=0; i<24; i++)
        leds.setLedColorData(i, 0, BR8[bri_led], 0); // green
          leds.show();
      disp_fix = 1;
      piezo.setSong((uint8_t*)SONG_FIX, sizeof(SONG_FIX), 169);
      if(stable_avg == 420){
        tft_print_all_it("420", TAR_L, TAR_T+8, 1);
        piezo.setSong((uint8_t*)SONG_420, sizeof(SONG_420), 12);
      }
      piezo.play();
    }
  }
  else{ 
    if(disp_fix == 1){
      tft_print_all_it("   ", TAR_L, TAR_T, 1);
      tft_print_all_it("   ", TAR_L, TAR_T+8, 1);
      disp_fix = 0;
    }
  }
  stable_x_l = x;
  stable_xled_l = xled;
}

// beer draw functions ----------
void beer_init(){
  // clear whole top area, individual displays
  for(uint8_t i = 1; i < 5; i++)
    tfts[i].fillRect(0, 0, 160, BEER_T + 40, BG[i]);

  // draw bottle/can top half border from beer_gfx array
  #define hhalf 18
  #define twall 3
  uint8_t h;
  tft_cs_all(LOW);
  for(uint8_t x = 0; x < BEER_x_M[beer_disp]; x++){
    if(x < twall || x > (BEER_x_M[beer_disp] - twall -1))
      h = hhalf - beer_gfx[x];  // full wall, no beer
    else
      h = twall;
    tfts[0].drawFastVLine(BEER_L + x, BEER_T + beer_gfx[x],     h,                       BEER_c_w[beer_disp]);
    tfts[0].drawFastVLine(BEER_L + x, BEER_T + beer_gfx[x] + h, hhalf - beer_gfx[x] - h, BEER_c_e[beer_disp]);
  }    
  tft_cs_all(HIGH);

  // copy bottle/can bottom half from beer_gfx_img bitmap
  uint16_t* c = beer_gfx_img;
  tft_cs_all(LOW);
  for(uint8_t y = BEER_T + 18; y < BEER_T + 36; y++)
    for(uint8_t x = BEER_L; x < BEER_L + BEER_x_M[beer_disp]; x++){
      if(*c != 0xF81F) //magenta
        tfts[0].writePixel(x, y, *c);
      c++;
    }
  tft_cs_all(HIGH);
}

void beer_draw(){
  #define tol 15  //g  tolerance mass, bottles and filling varies

  // if no beer type specified, either recognize if possible and init or leave
  if(beer_disp == 255){
    if(!disp_fix)
      return;
    if(abs(stable_avg - BEER_m_1[0]) < tol){
      beer_disp = 0; // can
      beer_gfx = (uint8_t*)BEER_CAN_BRD;
      beer_gfx_img = (uint16_t*)BEER_CAN_IMG;
      tft_print_all_it("CAN", TAR_L, TAR_T+16, 1);
    }else if(abs(stable_avg - BEER_m_1[1]) < tol){
      beer_disp = 1; // bottle nrw
      beer_gfx = (uint8_t*)BEER_NRW_BRD;
      beer_gfx_img = (uint16_t*)BEER_NRW_IMG;
      tft_print_all_it("NRW", TAR_L, TAR_T+16, 1);
    }else if(abs(stable_avg - BEER_m_1[2]) < tol){
      beer_disp = 2; // bottle eu
      beer_gfx = (uint8_t*)BEER_EUR_BRD;
      beer_gfx_img = (uint16_t*)BEER_EUR_IMG;
      tft_print_all_it("EUR", TAR_L, TAR_T+16, 1);
    }else return;
    beer_init();
  }

  #define hhalf    18
  #define twall    3
  #define wfoam    3      //px foam width
  #define BEER_x_0 twall  //px start of content
  #define mfull    (BEER_m_1[beer_disp] - BEER_m_0[beer_disp])
  #define xfull    (BEER_x_1[beer_disp] - BEER_x_0)
  
  // draw beer, overwrite affected area
  float mbeer = avg - BEER_m_0[beer_disp];
  uint8_t beer_x = min((float)BEER_x_1[beer_disp], max((float)BEER_x_0, BEER_x_0 + xfull * mbeer / mfull));
  beer_x_l = max(beer_x_l, (uint8_t)BEER_x_0);
  if(beer_x < beer_x_l){
    tft_cs_all(LOW);
    // less beer, overwrite from last foam top to new foam top with empty
    for(uint8_t x = beer_x_l + wfoam - 1; x > beer_x + wfoam - 1; x--)
      tfts[0].drawFastVLine(BEER_L + x, BEER_T + beer_gfx[x] + twall, hhalf - twall - beer_gfx[x], BEER_c_e[beer_disp]);
    // less beer, overwrite from new foam top to new beer top with foam
    for(uint8_t x = beer_x + wfoam - 1; x > beer_x - 1; x--)
      tfts[0].drawFastVLine(BEER_L + x, BEER_T + beer_gfx[x] + twall, hhalf - twall - beer_gfx[x], BEER_c_f[beer_disp]);
    tft_cs_all(HIGH);
  }
  else if(beer_x > beer_x_l){
    tft_cs_all(LOW);
    // more beer, overwrite from last beer top to new beer top with beer
    for(uint8_t x = beer_x_l; x < beer_x; x++)
      tfts[0].drawFastVLine(BEER_L + x, BEER_T + beer_gfx[x] + twall, hhalf - twall - beer_gfx[x], BEER_c_b[beer_disp]);
    // more beer, overwrite from new beer top to new foam top with foam
    for(uint8_t x = beer_x; x < beer_x + wfoam; x++)
      tfts[0].drawFastVLine(BEER_L + x, BEER_T + beer_gfx[x] + twall, hhalf - twall - beer_gfx[x], BEER_c_f[beer_disp]);
    tft_cs_all(HIGH);
  }

  beer_x_l = beer_x;

  // draw bubbles, remove old, calculate and draw new
  tft_cs_all(LOW);
  for(uint8_t b = 0; b < BUBBLE_N; b++){
    if(bubble_y[b] < hhalf && bubble_x[b] >= twall && bubble_x[b] < beer_x)
      tfts[0].writePixel(BEER_L + bubble_x[b], BEER_T + bubble_y[b], BEER_c_b[beer_disp]);
    if(bubble_y[b] < hhalf && bubble_x[b]+1 >= twall && bubble_x[b]+1 < beer_x)
      tfts[0].writePixel(BEER_L + bubble_x[b]+1, BEER_T + bubble_y[b], BEER_c_b[beer_disp]);
    bubble_x[b] += 1;// - random(0, 10) / 7; // 7/10 chance of rising
    bubble_y[b] += random(-6, 7) / 6; // rounds to zero -> 2/13 chance of 1px y movement
    // bubble at wall -> follow wall (ignores "bottom" wall)
    bubble_y[b] = max((int)bubble_y[b], beer_gfx[bubble_x[b]] + twall);
    // bubble hit max beer -> new bubble, limit was beer_x (foam), but this keeps bubble density constant
    if(bubble_x[b] >= BEER_x_1[beer_disp] || bubble_x[b] < twall){
      bubble_x[b] = random(twall, beer_x/3*2); // don't start at the foam
      bubble_y[b] = random(beer_gfx[bubble_x[b]] + twall, 2*hhalf - twall - beer_gfx[bubble_x[b]]);
    }
    if(bubble_y[b] < hhalf && bubble_x[b] >= twall && bubble_x[b] < beer_x)
      tfts[0].writePixel(BEER_L + bubble_x[b], BEER_T + bubble_y[b], BEER_c_f[beer_disp]);
    if(bubble_y[b] < hhalf && bubble_x[b]+1 >= twall && bubble_x[b]+1 < beer_x)
      tfts[0].writePixel(BEER_L + bubble_x[b]+1, BEER_T + bubble_y[b], 0b1100011000011000);
  }
  tft_cs_all(HIGH);

}

// scale draw functions ----------
// pre calculate height of scale lines
void gfx_init(){
  for(uint8_t i = 0; i<100; i++){
    if(i % 20 == 0)       gfx_h[i] = 8;
    else if(i % 10 == 0)  gfx_h[i] = 12;
    else if(i % 2 == 0)   gfx_h[i] = 3;
  }
}

void gfx_draw(){

  int16_t a_x, a_x_l;
  uint8_t changed = 0;
  uint8_t h, h_l;

  // iterate all x, calculate corresponding mass and array index for previous and current measurement, look up and draw
  for(int16_t x = SCALE_L; x < SCALE_R; x++){
      if(x == (uint8_t)SCALE_M) continue; // center line here
      a_x   = round(gfx_xtoa(avg  , x+0.5)); // get mass at pixel center, round to full g
      a_x_l = round(gfx_xtoa(avg_l, x+0.5));
      if(changed == 0 && x > ZOOM) return; // no change at all -> leave
      if(a_x == a_x_l) continue; // no change for this x, skip
      changed++;
      uint8_t h   = gfx_h[(a_x   + 10000) % 100];
      uint8_t h_l = gfx_h[(a_x_l + 10000) % 100];
      // new line longer, draw fg from old to new length
      if(h > h_l){
        tft_cs_all(LOW);
        tfts[0].drawFastVLine(x, SCALE_T + h_l, h - h_l, FGSCALE);
        tfts[0].drawFastVLine(x, SCALE_B - h,   h - h_l, FGSCALE);
        tft_cs_all(HIGH);
      }
      // old line longer, draw bg from new to old length
      else if(h_l > h){
        tft_cs_all(LOW);
        tfts[0].drawFastVLine(x, SCALE_T + h,   h_l - h, BGSCALE);
        tfts[0].drawFastVLine(x, SCALE_B - h_l, h_l - h, BGSCALE);
        tft_cs_all(HIGH);
      }
  }
  gfx_draw_text(avg_l, BGSCALE);
  gfx_draw_text(avg,   FGSCALE);
}

void gfx_draw_text(float midval, uint16_t color){
  // TEXT
  #define diff 20
  int16_t v = (int16_t)((midval + 10000) / diff - 1) * diff - 10000;
  int16_t x = gfx_atox(midval, v);
  uint8_t textshift;
  for(uint8_t i = 0; i < 4; i++){
    if(v > 999)         textshift = 9;
    else if(v > 99)     textshift = 6;
    else if(v > 9)      textshift = 3;
    else if(v > -1)     textshift = 0;
    else if(v > -10)    textshift = 3;
    else if(v > -100)   textshift = 6;
    else if(v > -1000)  textshift = 9;
    else if(v > -10000) textshift = 12;

    tfts[0].setTextColor(color);
    tft_print_all(v, x - 2 - textshift, ((uint16_t)SCALE_T + SCALE_B) / 2 - 3);
    v += diff;
    x += diff * ZOOM;
  }
}

// display x position corresponding to array index / mass 
// pixel values are defined to be at the left edge of the pixel
// ZOOM=1 midval=5g SCALE_M=80.5: [4.500g, 5.500g[ -> x=80 (int)
// ZOOM=2 midval=5g SCALE_M=80:   [4.500g, 5.000g[ -> x=80 (int)
// ZOOM=3 midval=5g SCALE_M=80.5: [4.833g, 5.133g[ -> x=80 (int)
uint16_t gfx_atox(float midval, float a){
  return floor(SCALE_M + (a - midval) * ZOOM);
}

// array index / avg mass corresponding to display x value,
// pixel values are defined to be at the left edge of the pixel
// ZOOM=1 midval=5g SCALE_M=80.5:  x=79 -> 3.5g   / x=80 -> 4.5g   / x=81 -> 5.5g
// ZOOM=2 midval=5g SCALE_M=80:    x=79 -> 4.5g   / x=80 -> 5g     / x=81 -> 5.5g
// ZOOM=3 midval=5g SCALE_M=80.5:  x=79 -> 4.5g   / x=80 -> 4.833g / x=81 -> 5.133g / x=82 -> 5.5
float gfx_xtoa(float midval, float x){
  return midval + (x - SCALE_M) / ZOOM;
}

// tft helper functions ----------
void tft_print_all_it(const __FlashStringHelper* s, int16_t x, int16_t y, uint8_t sz){
  for(uint8_t i=1; i<5; i++){
    tfts[i].setTextSize(sz);
    tfts[i].setCursor(x, y);
    tfts[i].print(s);
  }
}
void tft_print_all_it(const char* s, int16_t x, int16_t y, uint8_t sz){
  for(uint8_t i=1; i<5; i++){
    tfts[i].setTextSize(sz);
    tfts[i].setCursor(x, y);
    tfts[i].print(s);
  }
}

void tft_print_all(const __FlashStringHelper* s, int16_t x, int16_t y){
  tft_cs_all(LOW);
  tfts[0].setCursor(x, y);
  tfts[0].print(s);
  tft_cs_all(HIGH);
}
void tft_print_all(int16_t v, int16_t x, int16_t y){
  tft_cs_all(LOW);
  tfts[0].setCursor(x, y);
  tfts[0].print(v);
  tft_cs_all(HIGH);
}

void tft_cs_all(uint8_t val){
  digitalWrite(TFT_CS1, val);
  digitalWrite(TFT_CS2, val);
  digitalWrite(TFT_CS3, val);
  digitalWrite(TFT_CS4, val);
}

// disable sleep on sound playback
void onTone(uint16_t f){ sleep_en = 0; }
void onMute(){ sleep_en = 1; }

// inputs, menu
void rotary_cb(long v) {
  #ifdef DEBUG
    Serial.printf("rotate: %i\n", v);
  #endif
  draw_menu(v, 0);
}

void button_loop() {
  uint8_t btn = digitalRead(ROTARY_T);
  btn_hist = (btn_hist << 1) | btn;
  if((btn_hist & 0b00011111) == 0b0001111){
    #ifdef DEBUG
      Serial.printf("click \n");
    #endif
    uint8_t pos = rotary.readEncoder();
    draw_menu(pos, 1);
  }
}

void draw_menu(uint8_t v, uint8_t click){
  if(click){ // button pressed, new menu
    switch(menu){
      case 0:
        // 0 -> 1: initialize menu
        for(uint8_t i=1; i<5; i++)
          tfts[i].fillScreen(BG[i]);
        tft_print_all_it(" back...", 16, 10, 2);
        tft_print_all_it(" link...", 16, 10+16, 2);
        tft_print_all_it(" led bri.", 16, 10+32, 2);
        tft_print_all_it(" tft bri.", 16, 10+48, 2);
        tft_print_all_it(" snd vol.", 16, 10+64, 2);
        menu = 1;
        rotary.setBoundaries(0, 4, true);
        rotary.setEncoderValue(0);
        v = 0;
        break;
      case 1:
        // 1 -> x: select
        if(v == 0){
          init_main(); // includes menu=0, encoder stuff
          return;
        }
        if(v == 1){
          connect();
          init_main();
          return;
        }
        if(v == 2){
          menu = 12;
          tft_print_all_it("=", 10, 10+2*16, 2);
          rotary.setBoundaries(0, 7, false);
          rotary.setEncoderValue(bri_led);
        }
        if(v == 3){
          menu = 13;
          tft_print_all_it("=", 10, 10+3*16, 2);
          rotary.setBoundaries(0, 7, false);
          rotary.setEncoderValue(bri_tft);
        }
        if(v == 4){
          menu = 14;
          tft_print_all_it("=", 10, 10+4*16, 2);
          rotary.setBoundaries(0, 7, false);
          rotary.setEncoderValue(volume);
        }
        break;
      case 12:
        menu = 1;
        rotary.setEncoderValue(2);
        rotary.setBoundaries(0, 4, true);
        break;
      case 13:
        menu = 1;
        rotary.setEncoderValue(3);
        rotary.setBoundaries(0, 4, true);
        break;
      case 14:
        menu = 1;
        rotary.setEncoderValue(4);
        rotary.setBoundaries(0, 4, true);
        break;
    }
    v = rotary.readEncoder();
  }

  // encoder input
  //if(menu == 0){
  //  if(bri_led == bri_tft) // change both if same
  //    bri_led = v;
  //  bri_tft = v;
  //  ledcWrite(TFT_BL[1], BR10[v]);
  //  ledcWrite(TFT_BL[2], BR10[v]);
  //  ledcWrite(TFT_BL[3], BR10[v]);
  //  ledcWrite(TFT_BL[4], BR10[v]);
  //}

  if(menu == 12 || menu == 0){
    bri_led = v;
    for(uint8_t i=0; i<24; i++)
      leds.setLedColorData(i, 0, BR8[bri_led], 0);
    leds.show();
  }
  if(menu == 13){
    bri_tft = v;
    ledcWrite(TFT_BL[1], BR10[v]);
    ledcWrite(TFT_BL[2], BR10[v]);
    ledcWrite(TFT_BL[3], BR10[v]);
    ledcWrite(TFT_BL[4], BR10[v]); 
  }
  if(menu == 14){
    volume = v;
    piezo.setVolume((uint8_t)((v * 255ul) / 7)); 
    piezo.setSong((uint8_t*)SONG_FIX, sizeof(SONG_FIX), 169);
    piezo.play();
  }

  // draw
  if(menu == 1){
    for(uint8_t i=0; i<5; i++){
      if(i == v) 
        tft_print_all_it(">", 10, 10+i*16, 2);
      else 
        tft_print_all_it(" ", 10, 10+i*16, 2);
    }
  }
  if(menu == 1 || menu == 12){
    char txt[2];
    sprintf(txt, "%d", bri_led);
    tft_print_all_it(txt, 130, 42, 2);  
  }
  if(menu == 1 || menu == 13){
    char txt[2];
    sprintf(txt, "%d", bri_tft);
    tft_print_all_it(txt, 130, 58, 2);  
  }
  if(menu == 1 || menu == 14){
    char txt[2];
    sprintf(txt, "%d", volume);
    tft_print_all_it(txt, 130, 74, 2);  
  }
}


// esp-net
void data_recv(const uint8_t * mac, const uint8_t *incomingData, int len){

  Serial.printf("received %d byte\n", len);
  if(len == sizeof(intro)+sizeof(avg)+sizeof(stable_rec)){
    if(!strncmp((char*)intro, (char*)incomingData, sizeof(intro))){
      memcpy(&avg_rec, incomingData+sizeof(intro), sizeof(avg_rec));
      memcpy(&stable_rec, incomingData+sizeof(intro)+sizeof(avg_rec), sizeof(stable_rec));
    }
    return;
  }
  if(len == sizeof(greeting))
    if(!strncmp((char*)greeting, (char*)incomingData, sizeof(greeting))){
      peer_found = 1;
      #ifdef DEBUG
        Serial.println("found peer!!");
      #endif
    }
}

void send_weight(){
  uint8_t buffer[sizeof(intro) + sizeof(avg) + sizeof(stable)];
  memcpy(buffer, intro, sizeof(intro));
  memcpy(buffer + sizeof(intro), &avg, sizeof(avg));
  memcpy(buffer + sizeof(intro) + sizeof(avg), &stable, sizeof(stable));
  esp_err_t result = esp_now_send(broadcast, buffer, sizeof(buffer));
}

void send_greeting(){
  //Serial.println("greet!!");
  if(millis() - peer_last < 300)
    return;
  peer_last = millis();
  esp_err_t result = esp_now_send(broadcast, greeting, sizeof(greeting));
  if(millis() / 500 % 2 || peer_found)
    tft_print_all_it("C", 4, TAR_T, 1);
  else
    tft_print_all_it(" ", 4, TAR_T, 1);
}

void connect() {
  if(peer_start == 0){
    Serial.println("connect!!");
    peer_start = millis();
    peer_found = 0;
    disp_remote = 2;
  }

  WiFi.mode(WIFI_STA);
  //WiFi.disconnect();
  WiFi.STA.begin();

  esp_now_init();
  esp_now_register_recv_cb(esp_now_recv_cb_t(data_recv));

  esp_now_peer_info_t peer = {};
  memset(peer.peer_addr, 0xff, 6);
  peer.channel = 0;  
  peer.encrypt = false;
  esp_now_add_peer(&peer);
}

void disconnect(){
  Serial.println("disconnecting");
  tft_print_all_it(" ", 4, TAR_T, 1);
  peer_start = 0;
  esp_now_deinit();
  //WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}
