////////////////////////////////////////
//
// Beer Scale v1.0
// Bernhard Stampfer 2024
//
////////////////////////////////////////

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "HX711.h"           // scale adc
#include "beer_gfx.h"
#include <TonePlayer.h>
//#include "driver/ledc.h"
//#include "hal/ledc_types.h"
#include "AiEsp32RotaryEncoder.h"

//#define DEBUG

// DISPLAY (Aliexpress 1.77" TFT)
// https://github.com/adafruit/Adafruit-ST7735-Library/blob/master/examples/graphicstest/graphicstest.ino
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
// https://github.com/RobTillaart/HX711/blob/master/examples/HX_kitchen_scale/HX_kitchen_scale.ino
#define SCALE_MISO  27//12
#define SCALE_CLK   26//14
// MISO, SCK using hardware HSPI? -- hardware spi not working as pin 14 can't be high on boot
HX711 scale;

// ROTARY ENCODER/BUTTON
// AliExpress EC11 15mm plum handle, https://de.aliexpress.com/item/1005005983134515.html
#define ROTARY_A GPIO_NUM_2
#define ROTARY_B GPIO_NUM_4
#define ROTARY_T GPIO_NUM_15
// library seems unstable (v1.7) but can handle pulldown encoder with interrupt, high active button not working, handled manually
AiEsp32RotaryEncoder rotary = AiEsp32RotaryEncoder(ROTARY_A, ROTARY_B, -1, -1, 4, true);

// PIEZO SPEAKER
#define PIEZO_P 14
// library seems hardcoded to ledc timer0 channel0, which requires some workaround for display pwm and light sleep mode
TonePlayer piezo(PIEZO_P);

// GLOBALS
// scale scale
float raw[5] = {0};     //g  weight raw value buffer
uint8_t pos = 0;        //g  weight raw value buffer position
float avg = 0;          //g  weight       fir/iir filtered
float avg_l = 0;        //g  weight, last
float davg = 0;         //g  weight difference
float tar = 0;          //g  tara value   fir/iir filtered below 2g
uint8_t disp_tar = 0;   //   diplayed tare changing, boolean
uint8_t disp_fix = 0;   //   diplayed fix, boolean
uint8_t sleep_en = 1;   //   allow sleeping in main loop
uint32_t tlast;         //ms loop timing
uint8_t btn_hist = 0;   //   last button states f. debouncing
uint8_t menu = 0;       //   which menu we are in, 0=none

// text
#define TXT_L 10
#define TXT_T 50
uint16_t FG[5] = {ST77XX_BLACK, ST77XX_BLACK, ST77XX_WHITE, ST77XX_WHITE, ST77XX_BLACK};
uint16_t BG[5] = {ST77XX_WHITE, ST77XX_GREEN, ST77XX_RED,   ST77XX_BLUE,  ST77XX_YELLOW};

// tara sign
#define TAR_L 159 - 6*3 - 4 //137 to 154
#define TAR_T 50            

// get stable value
#define STABLE_L   5    
#define STABLE_T   42 
#define STABLE_W   150
#define STABLE_H   3 
#define STABLE_t_0 700    //ms start displaying bar
#define STABLE_t_1 2200   //ms consider value stable
uint32_t stable_ms = 0;   //ms time without change
int16_t  stable_avg;      //g  stable value, full integers
uint32_t stable_a_ms = 0; //ms time without change
int16_t  stable_a_avg;    //g  stable value, +0.5
uint32_t stable_b_ms = 0; //ms time without change
int16_t  stable_b_avg;    //g  stable value, +0.5
uint8_t  stable_x_l = 0;  //px last drawn x value

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
#define BEER_T  2
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
uint16_t brightness[8] = {1, 3, 8, 20, 60, 180, 400, 1023};

const PROGMEM uint8_t SONG_START[] = { NC6, 256-2, ND6, 256-2, NE6,  256-2, NF5,   256-2, 0, 0, 64, 0};
const PROGMEM uint8_t SONG_FIX[] = { NC6, 256-2, 0, 0, 64, 0};
const PROGMEM uint8_t SONG_420[] = {
  NFIS6, 256-2, NFIS6, 256-2, ND6,  256-2, NB5,   256-2, REST,  256-2, NB5,   256-2, REST,  256-2, NE6,   256-2,
  REST,  256-2, NE6,   256-2, REST, 256-2, NE6,   256-2, NGIS6, 256-2, NGIS6, 256-2, NA6,   256-2, NB6,   256-2,
  NA6,   256-2, NA6,   256-2, NA6,  256-2, NE6,   256-2, REST,  256-2, ND6,   256-2, REST,  256-2, NFIS6, 256-2,
  REST,  256-2, NFIS6, 256-2, REST, 256-2, NFIS6, 256-2, NE6,   256-2, NE6,   256-2, NFIS6, 256-2, NE6,   256-2,
  REPEAT, 0, 0, 64, 0
};


// CODE
void IRAM_ATTR readEncoderISR(){
    rotary.readEncoder_ISR();
}

void setup() {
  btStop();
  //#ifdef DEBUG
  Serial.begin(115200);
  //#endif

  // BACKLIGHTS
  // timer 0 used for sound, attach backlight pwm to channels using other timers
  for(uint8_t i=1; i<5; i++){
    pinMode(TFT_BL[i], OUTPUT);
    ledcAttachChannel(TFT_BL[i], 100, 10, 9+i); // 10,11 -> T1 12,13 -> T2
    ledcWrite(TFT_BL[i], 0);
  }
  // set timers 1,2 to use low speed rc clock which runs during light sleep
  ledc_timer_config_t ledc_timer = {.speed_mode = LEDC_LOW_SPEED_MODE, .duty_resolution = LEDC_TIMER_10_BIT,
                                    .timer_num = LEDC_TIMER_1, .freq_hz = 100, .clk_cfg = LEDC_USE_RC_FAST_CLK};
  ledc_timer_config(&ledc_timer);
  ledc_timer.timer_num = LEDC_TIMER_2;
  ledc_timer_config(&ledc_timer);

  // INIT PIEZO
  //pinMode(PIEZO_P, OUTPUT);
  piezo.setOnToneCallback(onTone);
  piezo.setOnMuteCallback(onMute);
  piezo.setSong((uint8_t*)SONG_START, sizeof(SONG_START), 169);
  piezo.play();  // Begins playback a song.

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
      ledcWrite(TFT_BL[i], 1023);
      tfts[i].setTextColor(FG[i], BG[i]);
      tfts[i].fillScreen(BG[i]);
      tfts[i].setTextSize(3);
    }else{
      tfts[i].setTextColor(FG[i]);
      tfts[i].setTextSize(1);
    }
  }
  
  tft_print_all(DINIT, 2, 2);
  #ifdef DEBUG
    Serial.println(DINIT);
  #endif

  // INIT SCALE ADC, MEASURE INITIAL TARE
  scale.begin(SCALE_MISO, SCALE_CLK);
  scale.set_scale(220.56);
  tar = scale.get_units(10); // bad first value?
  avg = tar = scale.get_units(10);

  tft_print_all(SINIT, 2, 10);
  #ifdef DEBUG
    Serial.println(SINIT);
  #endif

  // INIT ROTARY
  rotary.begin();
  rotary.setup(readEncoderISR);
  //rotary.setAcceleration(0);
  //rotary.correctionOffset=2;
  pinMode(ROTARY_T, INPUT_PULLDOWN);
  
  // INIT GRAPHICS
  gfx_init();
  draw_main();
  
  #ifdef DEBUG
    Serial.println(GINIT);
  #endif

  //gpio_dump_all_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
}

void draw_main() {
  menu = 0;
  rotary.setBoundaries(0, 7, false);
  rotary.setEncoderValue(7);

  for(uint8_t i = 1; i < 5; i++)
    tfts[i].fillScreen(BG[i]);

  // draw scale background
  tft_cs_all(LOW);
  tfts[0].fillRect(0, 128-49, 160, 49, BGSCALE);      
  tfts[0].drawFastHLine(0, 128-50, 160, FGSCALE);
  tfts[0].drawFastVLine(SCALE_M, 128-49, 20, ST77XX_RED);
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
  for(uint8_t y = 0; y < 42; y++)
    for(uint8_t x = 160-70; x < 160-70+64; x++){
      if(*c != 0xF81F) //magenta
        tfts[0].writePixel(x, y, *c);
      c++;
    }
  tfts[0].fillRect(6, 20, 160-70-6, 2, 0x047a);
  tft_cs_all(HIGH);
}

void draw_menu1(){
  menu = 1;

  tft_cs_all(LOW);
  tfts[0].fillScreen(BG[0]);
  tft_print_all_it("> back", 10, 10, 2);
  tft_print_all_it("  test1", 10, 10+16, 2);
  tft_print_all_it("  test2", 10, 10+32, 2);
  tft_cs_all(LOW);
  
  rotary.setBoundaries(0, 2, true);
  rotary.setEncoderValue(0);
}

void loop() {

  // measure and average
  measure();

  // new text
  if(menu == 0){
      
    char txt[19];
    if(avg <= -1000)
      sprintf(txt, " low", avg);
    else{
      if(avg > 4000)
        sprintf(txt, "high", avg);
      else{
          sprintf(txt, "%6.1f", avg);
      }
    }
    if((millis()-stable_ms) < STABLE_t_1 && round(10*avg) != round(10*avg_l))
      tft_print_all_it(txt, TXT_L, TXT_T, 3);
    
    // draw graphics
    stable_draw();
    gfx_draw();
    beer_draw();
  }
  if(menu == 1){
    // only redraw on input
  }

  // button, music stuff
  piezo.loop();
  if (rotary.encoderChanged()) rotary_cb(rotary.readEncoder());
  button_loop();

  // frame rate limiter, use sleep to save some battery if possible
  if(33 - min(33lu, millis() - tlast) > 0){
    if(!sleep_en)
      delay(33 - min(33lu, millis() - tlast));
    else{
      gpio_wakeup_enable(ROTARY_A, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable(ROTARY_B, GPIO_INTR_HIGH_LEVEL);
      gpio_wakeup_enable(ROTARY_T, GPIO_INTR_HIGH_LEVEL);
      esp_sleep_enable_gpio_wakeup();
      //esp_deep_sleep(1000*(millis() - tlast)); kills all data outside rtc ram
      esp_sleep_enable_timer_wakeup(1000*(33-min(33lu, millis() - tlast)));
      esp_light_sleep_start(); 
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
  float alpha = 0.008 + 0.15 * ((fabs(davg)/dnoise))/((fabs(davg)/dnoise)+1);
  
  //float delta = millis()-stable_ms;
  //float alpha = 0.01 + 0.15 * max(0.0f, min(1.0f, (float)STABLE_t_0 * STABLE_t_0 / delta / delta ));
  //if(fabs(avg) < 2) alpha = 0.04;
  avg_l = avg;
  avg = (1-alpha) * avg + alpha * val;

  // iir tare below 2g to account for beer sweat on scale
  if(fabs(avg) < 2.0 && fabs(val) < 2.0){
    tar = tar + 0.004 * val;
    if(disp_tar == 0){
      tft_print_all_it("TAR", TAR_L, TAR_T, 1);
      disp_tar = 1;
    }
  }
  else{
    if(disp_tar == 1){
      tft_print_all_it("   ", TAR_L, TAR_T, 1);
      disp_tar = 0;
    }
  }

  // reset stable time if value changed
  #if 1
  // based on rate of change
  uint16_t sta = (uint16_t)floor(round(avg*10)/10);
  if((fabs(avg) < 2) || fabs(davg) > 0.5){
    stable_ms = millis();
    stable_avg = sta;
  }
  #endif

  #if 0
  // based on difference from stable value
  uint16_t sta = (uint16_t)floor(round(avg*10)/10);
  if((fabs(avg) < 2) || (stable_a_avg - sta) != 0){
    stable_a_ms = millis();
    stable_a_avg = sta;
    stable_ms = stable_b_ms;
    stable_avg = stable_b_avg;
  }
  uint16_t stb = (uint16_t)round(round(avg*10)/10);
  if((fabs(avg) < 2) || (stable_b_avg - stb) != 0){
    stable_b_ms = millis();
    stable_b_avg = sta;
    stable_ms = stable_a_ms;
    stable_avg = stable_a_avg;
  }
  #endif
}

// draws the line and FIX text ----------
void stable_draw(){
  //if(stable_avg < 2) return;
  float t = millis() - stable_ms;
  uint8_t x = max(0.0f, min((float)STABLE_W, STABLE_W * (t - STABLE_t_0) / (STABLE_t_1 - STABLE_t_0)));
  // bar
  for(uint8_t i=1; i<5; i++){
    if(x > stable_x_l){
      tfts[i].fillRect(STABLE_L + stable_x_l, STABLE_T, x - stable_x_l, STABLE_H, FG[i]);
    }
    else if (x < stable_x_l){ // reset
      tfts[i].fillRect(STABLE_L + x,          STABLE_T, stable_x_l - x, STABLE_H, BG[i]);
    }
  }

  // logos
  if(x==STABLE_W){
    if(disp_fix == 0){
      tft_print_all_it("FIX", TAR_L, TAR_T, 1);
      disp_fix = 1;
      piezo.setSong((uint8_t*)SONG_FIX, sizeof(SONG_FIX), 169);
      if(stable_avg == 420){
        tft_print_all_it("420", TAR_L, TAR_T+8, 1);
        piezo.setSong((uint8_t*)SONG_420, sizeof(SONG_420), 169);
      }
      piezo.play();  // Begins playback a song.     
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
    if((millis() - stable_ms) < STABLE_t_1)
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

void onTone(uint16_t f){ sleep_en = 0; }
void onMute(){ sleep_en = 1; }

void rotary_cb(long v) {
  #ifdef DEBUG
    Serial.printf("rotate: %i\n", v);
  #endif
  if(menu == 0){
  //if(v == 7) sleep_en = 1; else sleep_en = 0;
    ledcWrite(TFT_BL[1], brightness[v]);
    ledcWrite(TFT_BL[2], brightness[v]);
    ledcWrite(TFT_BL[3], brightness[v]);
    ledcWrite(TFT_BL[4], brightness[v]);
  }
  if(menu != 0){
    for(uint8_t i=0; i<3; i++){
      if(i == v) 
        tft_print_all_it(">", 10, 10+i*16, 2);
      else 
        tft_print_all_it(" ", 10, 10+i*16, 2);
    }
  }
}

void button_loop() {
  uint8_t btn = digitalRead(ROTARY_T);
  btn_hist = (btn_hist << 1) | btn;
    if((btn_hist & 0b00011111) == 0b0001111){
      //Serial.printf("click \n");
      if(menu == 0) draw_menu1();
      else if(menu == 1) {
        uint8_t pos = rotary.readEncoder();
        if(pos == 0) draw_main();
        else Serial.printf("menu %d", pos);
      }
    }
}
