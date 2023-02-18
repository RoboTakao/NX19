#include <Wire.h>          // I2C setting
#include <PCA9685.h>       //for PCA9685
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "1010"
#define CHRX_UUID "1012"
#define CHTX_UUID "1011"

byte joyLX, joyLY, joyRX=100, joyRY=100, joyLSW, joyRSW, joyLDistance, joyRDistance;

BLEServer* pServer = NULL;
BLECharacteristic* pCharTx = NULL;
BLECharacteristic* pCharRx = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define LGFX_USE_V1

#include <LovyanGFX.hpp>

#define MPU6050_ADDR         0x68 // MPU-6050 device address
#define MPU6050_SMPLRT_DIV   0x19 // MPU-6050 register address
#define MPU6050_CONFIG       0x1a
#define MPU6050_GYRO_CONFIG  0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6b

const uint8_t srv_CH0 = 0, srv_CH1 = 1, srv_CH2 = 2, srv_CH3 = 3, srv_CH4 = 4, srv_CH5 = 5, srv_CH6 = 6, srv_CH7 = 7, srv_CH8 = 8; //PCA9685チャンネル 0-8
const uint8_t srv_CH9 = 9, srv_CH10 = 10, srv_CH11 = 11, srv_CH12 = 12, srv_CH13 = 13, srv_CH14 = 14, srv_CH15 = 15; //PCA9685チャンネル 0-15
const uint8_t srv_CH16 = 0, srv_CH17 = 1; //PWMチャンネル 16-17

const float Pi = 3.141593;

double offsetX = 0, offsetY = 0, offsetZ = 0;
double offsetX_acc = 0, offsetY_acc = 0;
double gyro_angle_x = 0, gyro_angle_y = 0, gyro_angle_z = 0;
float angleX, angleY, angleZ;
float interval, preInterval;
float acc_x, acc_y, acc_z, acc_angle_x, acc_angle_y;
float gx, gy, gz, dpsX, dpsY, dpsZ;

int angZero[] = {84,78,99,86,96,100,95,94,93,97,84,84,90,92,83,94,99,92};
int angHome[] = {-11,14,-1,0,13,3,11,14,-1,11,-14,1,0,-13,-3,-11,-14,1};
int ang0[18];
int ang1[18];
int ang_b[18];
char ang_c[18];
float ts=100;  //100msごとに次のステップに移る
float td=20;   //10回で分割

float L1 = 40;
float L2 = 70;
float L0 = 13;

float X0[6] = {-25, -50, -25, 25, 50, 25};
float Z0[6] = {43.3, 0, -43.3, 43.3, 0, -43.3};

float H0 = 60; //Height
float Zc = 31.5; //Half Width
float Nee = 20;

float Fs = 15; //FWDstep
float Ss = 20; //SideStep
float Dis = 77.9; //Distance
float Adj = 10; //Adjust
float Up = 20; //FootUp
float Wd = 90; //Width
//float TnX = 0.0; //TurnX
float TnY = 10.0; //TurnY
//float TnZ = 0.0; //TurnZ
float CenX = 0; //CenterX
float CenZ = 0; //CenterZ

int walk_status = 0; //stay 0 , right foot up & walk 1,
int stay_mode = -1; //rotation -1 , slide = 1
int walk_mode = -1; //walk -1, staying 1
int UnitV_mode = 1;
int IMU_mode = 0;

int eye_u_x = 50;
int eye_u_y = 50;
int head_UnitV_x = 0;
int head_UnitV_y = 0;
int head_IMU_x = 0;
int head_IMU_z = 0;
float af = 0.7;
float af_imu = 0.8;

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    std::string value = pChar->getValue();
    if (value.length()>0) {
      joyLX=value[0];
      joyLY=value[1];
      joyRX=value[2];
      joyRY=value[3];
      joyLDistance=value[4];
      joyRDistance=value[5];
      joyLSW=value[6];
      joyRSW=value[7];
      //Serial.print(joyLX);
      //Serial.print(" ");
      //Serial.print(joyLY);
      //Serial.print(" ");
      //Serial.print(joyRX);
      //Serial.print(" ");
      //Serial.print(joyRY);
      //Serial.print(" ");
      //Serial.print(joyLDistance);
      //Serial.print(" ");
      //Serial.print(joyRDistance);
      //Serial.print(" ");
      //Serial.print(joyLSW);
      //Serial.print(" ");
      //Serial.println(joyRSW);
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class LGFX_M5Stamp_SPI_GC9A01 : public lgfx::LGFX_Device
{
lgfx::Panel_GC9A01      _panel_instance;
lgfx::Bus_SPI       _bus_instance;   // SPIバスのインスタンス
lgfx::Light_PWM     _light_instance;
public:
  LGFX_M5Stamp_SPI_GC9A01(void)
  {
    { // バス制御の設定を行います。
      auto cfg = _bus_instance.config();    // バス設定用の構造体を取得します。

// SPIバスの設定
      cfg.spi_host = SPI2_HOST;     // 使用するSPIを選択  ESP32-S2,C3 : SPI2_HOST or SPI3_HOST / ESP32 : VSPI_HOST or HSPI_HOST
      // ※ ESP-IDFバージョンアップに伴い、VSPI_HOST , HSPI_HOSTの記述は非推奨になるため、エラーが出る場合は代わりにSPI2_HOST , SPI3_HOSTを使用してください。
      cfg.spi_mode = 0;             // SPI通信モードを設定 (0 ~ 3)
      cfg.freq_write = 40000000;    // 送信時のSPIクロック (最大80MHz, 80MHzを整数で割った値に丸められます)
      cfg.freq_read  = 16000000;    // 受信時のSPIクロック
      cfg.spi_3wire  = true;        // 受信をMOSIピンで行う場合はtrueを設定
      cfg.use_lock   = true;        // トランザクションロックを使用する場合はtrueを設定
      cfg.dma_channel = SPI_DMA_CH_AUTO; // 使用するDMAチャンネルを設定 (0=DMA不使用 / 1=1ch / 2=ch / SPI_DMA_CH_AUTO=自動設定)
      // ※ ESP-IDFバージョンアップに伴い、DMAチャンネルはSPI_DMA_CH_AUTO(自動設定)が推奨になりました。1ch,2chの指定は非推奨になります。
      cfg.pin_sclk = 4;            // SPIのSCLKピン番号を設定
      cfg.pin_mosi = 6;            // SPIのMOSIピン番号を設定
      cfg.pin_miso = -1;            // SPIのMISOピン番号を設定 (-1 = disable)
      cfg.pin_dc   = 1;            // SPIのD/Cピン番号を設定  (-1 = disable)
     // SDカードと共通のSPIバスを使う場合、MISOは省略せず必ず設定してください。
      _bus_instance.config(cfg);    // 設定値をバスに反映します。
      _panel_instance.setBus(&_bus_instance);      // バスをパネルにセットします。
    }

    { // 表示パネル制御の設定を行います。
      auto cfg = _panel_instance.config();    // 表示パネル設定用の構造体を取得します。
      cfg.pin_cs           =    7;  // CSが接続されているピン番号   (-1 = disable)
      cfg.pin_rst          =    0;  // RSTが接続されているピン番号  (-1 = disable)
      cfg.pin_busy         =    -1;  // BUSYが接続されているピン番号 (-1 = disable)
      // ※ 以下の設定値はパネル毎に一般的な初期値が設定されていますので、不明な項目はコメントアウトして試してみてください。
      cfg.memory_width     =   240;  // ドライバICがサポートしている最大の幅
      cfg.memory_height    =   240;  // ドライバICがサポートしている最大の高さ
      cfg.panel_width      =   240;  // 実際に表示可能な幅
      cfg.panel_height     =   240;  // 実際に表示可能な高さ
      cfg.offset_x         =     0;  // パネルのX方向オフセット量
      cfg.offset_y         =     0;  // パネルのY方向オフセット量
      cfg.offset_rotation  =     0;  // 回転方向の値のオフセット 0~7 (4~7は上下反転)
      cfg.dummy_read_pixel =     8;  // ピクセル読出し前のダミーリードのビット数
      cfg.dummy_read_bits  =     1;  // ピクセル以外のデータ読出し前のダミーリードのビット数
      cfg.readable         =  true;  // データ読出しが可能な場合 trueに設定
      cfg.invert           =  true;  // パネルの明暗が反転してしまう場合 trueに設定
      cfg.rgb_order        = false;  // パネルの赤と青が入れ替わってしまう場合 trueに設定
      cfg.dlen_16bit       = false;  // データ長を16bit単位で送信するパネルの場合 trueに設定
      cfg.bus_shared       =  true;  // SDカードとバスを共有している場合 trueに設定(drawJpgFile等でバス制御を行います)

      _panel_instance.config(cfg);
    }
    { // バックライト制御の設定を行います。（必要なければ削除）
      auto cfg = _light_instance.config();    // バックライト設定用の構造体を取得します。

      cfg.pin_bl = 10;              // バックライトが接続されているピン番号
      cfg.invert = false;           // バックライトの輝度を反転させる場合 true
      cfg.freq   = 44100;           // バックライトのPWM周波数
      cfg.pwm_channel = 3;          // 使用するPWMのチャンネル番号

      _light_instance.config(cfg);
      _panel_instance.setLight(&_light_instance);  // バックライトをパネルにセットします。
    }
    setPanel(&_panel_instance); // 使用するパネルをセットします。
  }
};

static LGFX_M5Stamp_SPI_GC9A01 lcd;  //LGFX_M5Stamp_SPI_GC9A01のインスタンスを作成。
static LGFX_Sprite sprite(&lcd);
static LGFX_Sprite sprite2(&sprite);

HardwareSerial VSerial(1);

typedef struct
{
    int16_t dx;
    int16_t dy;
}v_response_t;

// Base Step
int bs_s[11][18]={
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
// Walk motion Step
float wm_s[11][18]={
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};
// Base_angle
float Theta[3]={0.0,0.0,0.0};
// Base Head Step
float bh_s[18]={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

//PWM
const uint8_t Srv16 = 18;
const uint8_t Srv17 = 19;

const double PWM_Hz = 50;   //PWM周波数
const uint8_t PWM_level = 14; //PWM 14bit(0～16384)
//const uint8_t PWM_level = 16; //PWM 16bit(0～65535)


int pulseMIN = 410;  //0deg 500μsec 50Hz 14bit : PWM周波数(Hz) x 2^14(bit) x PWM時間(μs) / 10^6
int pulseMAX = 2048;  //180deg 2500μsec 50Hz 14bit : PWM周波数(Hz) x 2^14(bit) x PWM時間(μs) / 10^6

//int pulseMIN = 1640;  //0deg 500μsec 50Hz 16bit : PWM周波数(Hz) x 2^16(bit) x PWM時間(μs) / 10^6
//int pulseMAX = 8190;  //180deg 2500μsec 50Hz 16bit : PWM周波数(Hz) x 2^16(bit) x PWM時間(μs) / 10^6

int cont_min = 0;
int cont_max = 180;

//PCA9685のアドレス指定
PCA9685 pwm = PCA9685(0x40);

#define SERVOMIN 104            //Min pulse width (12bit 500μs) 
#define SERVOMAX 512            //Max pulse width (12bit 2500μs) 

void servo_write(int ch, int ang){
  ang = map(ang, 0, 180, SERVOMIN, SERVOMAX); //angle（0～180）-> pulse width（150～500）
  pwm.setPWM(ch, 0, ang);
}

//PWM
void Srv_drive(int srv_CH,int SrvAng){
  SrvAng = map(SrvAng, cont_min, cont_max, pulseMIN, pulseMAX);
  ledcWrite(srv_CH, SrvAng);
}

void eye()
{
  sprite2.fillCircle(70,70,68,TFT_BLUE);
  sprite2.fillCircle(70,70,45,TFT_CYAN);
  sprite2.fillCircle(70,70,30,TFT_NAVY);
  sprite2.fillCircle(90,50,15,TFT_WHITE);
}

void Sleep_eye()
{
  sprite.fillScreen(TFT_BLACK);
  eye();

  for(int i=0 ; i < 8 ; i++)
  {
    lcd.setBrightness(128-16*(i+1));
    sprite2.fillCircle(70,0,i*20,TFT_BLACK);
    sprite2.pushSprite(50,50);
    sprite.pushSprite(0,0);
    delay(50);
  }
  
  for(int i=7 ; i > 0 ; i--)
  {
    lcd.setBrightness(128-16*(i+1));
    eye();
    sprite2.fillCircle(70,0,i*20,TFT_BLACK);
    sprite2.pushSprite(50,50);
    sprite.pushSprite(0,0);
    delay(50);
  }
}

void Motion_eye(int eyeX, int eyeY)
{
  lcd.setBrightness(128);
  sprite.fillScreen(TFT_BLACK);
  eye();
  sprite2.pushSprite(eyeX,eyeY);
  sprite.pushSprite(0,0);
}

void ik(float *X3,float *Y3,float *Z3,float *Xx0,float *Zz0,float *Theta1_deg,float *Theta2_deg,float *Theta3_deg)
{
  float L = sqrt(pow(*X3 - *Xx0,2) + pow(*Z3 - *Zz0,2));
  float Xd2 = L - L0;

  float Theta1_rad = -acos((pow(Xd2,2) + pow(*Y3,2) + pow(L1,2) - pow(L2,2))/(2*L1*sqrt(pow(Xd2,2) + pow(*Y3,2)))) + atan2(*Y3,Xd2);
  float Theta2_rad = atan2(*Y3 - L1 * sin(Theta1_rad),Xd2 - L1 * cos(Theta1_rad)) - Theta1_rad;
  float Theta3_rad = atan2(*Z3 - *Zz0, *X3 - *Xx0);

  *Theta1_deg = Theta1_rad / Pi * 180;
  *Theta2_deg = Theta2_rad / Pi * 180;
  *Theta3_deg = Theta3_rad / Pi * 180;
}

void rot(float *X3, float *Y3, float *Z3, float *XX3, float *YY3, float *ZZ3, float *Tr_X, float *Tr_Y, float *Tr_Z)
{
  float TrX_c = *Tr_X/180*Pi;
  float TrY_c = *Tr_Y/180*Pi;
  float TrZ_c = *Tr_Z/180*Pi;
  
  *X3 = *XX3 * cos(TrZ_c)*cos(TrY_c) + *YY3*(cos(TrZ_c)*sin(TrY_c)*sin(TrX_c) - sin(TrZ_c)*cos(TrX_c)) + *ZZ3*(cos(TrZ_c)*sin(TrY_c)*cos(TrX_c)+sin(TrZ_c)*sin(TrX_c));
  *Y3 = *XX3 * sin(TrZ_c)*cos(TrY_c) + *YY3*(sin(TrZ_c)*sin(TrY_c)*sin(TrX_c) + cos(TrZ_c)*cos(TrX_c)) + *ZZ3*(sin(TrZ_c)*sin(TrY_c)*cos(TrX_c)-cos(TrZ_c)*sin(TrX_c));
  *Z3 = - *XX3 * sin(TrY_c) + *YY3 * cos(TrY_c)*sin(TrX_c) + *ZZ3*cos(TrY_c)*cos(TrX_c);
}

void walk_1st_s(float Fs_c, float Ss_c, float TnY_c_d)
{
  float Wd_c = Wd*cos(Pi/3);
  float TnY_c = TnY_c_d/180*Pi;
  
  float w_1s[18]={-Wd_c-Adj,H0,Dis+CenZ,-Wd,H0-Up,CenZ,-Wd_c-Adj,H0,-Dis+CenZ,Wd_c+Adj,H0-Up,Dis+CenZ,Wd,H0,CenZ,Wd_c+Adj,H0-Up,-Dis+CenZ};

  for (int i=0; i <=10 ; i++){
    ik(&w_1s[0],&w_1s[1],&w_1s[2],&X0[0],&Z0[0],&Theta[0],&Theta[1],&Theta[2]); //Right Front
    angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][0], &bs_s[0][1], &bs_s[0][2]);

    ik(&w_1s[3],&w_1s[4],&w_1s[5],&X0[1],&Z0[1],&Theta[0],&Theta[1],&Theta[2]); //Right Mid
    angle_cul_RM(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][3], &bs_s[0][4], &bs_s[0][5]);

    ik(&w_1s[6],&w_1s[7],&w_1s[8],&X0[2],&Z0[2],&Theta[0],&Theta[1],&Theta[2]); //Right Back
    angle_cul_RB(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][6], &bs_s[0][7], &bs_s[0][8]);

    ik(&w_1s[9],&w_1s[10],&w_1s[11],&X0[3],&Z0[3],&Theta[0],&Theta[1],&Theta[2]); //Left Front
    angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][9], &bs_s[0][10], &bs_s[0][11]);

    ik(&w_1s[12],&w_1s[13],&w_1s[14],&X0[4],&Z0[4],&Theta[0],&Theta[1],&Theta[2]); //Left Mid
    angle_cul_LM(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][12], &bs_s[0][13], &bs_s[0][14]);

    ik(&w_1s[15],&w_1s[16],&w_1s[17],&X0[5],&Z0[5],&Theta[0],&Theta[1],&Theta[2]); //Left Back
    angle_cul_LB(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][15], &bs_s[0][16], &bs_s[0][17]);
  }

  for (int j=0; j <=17 ; j++){
    ang1[j] = angZero[j] + bs_s[0][j];
  }
  servo_set();
}

void walk_s(float Fs_c, float Ss_c, float TnY_c_d)
{
  float Wd_c = Wd*cos(Pi/3);
  float TnY_c = TnY_c_d/180*Pi;
  
  float w_s[8][18]={
    {-Wd_c+Ss_c-Adj,H0,Dis-Fs_c+CenZ,-Wd-Ss_c,H0-Up,Fs_c+CenZ,-Wd_c+Ss_c-Adj,H0,-(Dis+Fs_c-CenZ),Wd_c-Ss_c+Adj,H0-Up,Dis+Fs_c+CenZ,Wd+Ss_c,H0,-Fs_c+CenZ,Wd_c-Ss_c+Adj,H0-Up,Dis-Fs_c-CenZ},
    {-Wd_c+Ss_c-Adj,H0,Dis-Fs_c+CenZ,-Wd-Ss_c,H0,Fs_c+CenZ,-Wd_c+Ss_c-Adj,H0,-(Dis+Fs_c-CenZ),Wd_c-Ss_c+Adj,H0,Dis+Fs_c+CenZ,Wd+Ss_c,H0,-Fs_c+CenZ,Wd_c-Ss_c+Adj,H0,Dis-Fs_c-CenZ},
    {-Wd_c+Ss_c-Adj,H0-Up,Dis-Fs_c+CenZ,-Wd-Ss_c,H0,Fs_c+CenZ,-Wd_c+Ss_c-Adj,H0-Up,-(Dis+Fs_c-CenZ),Wd_c-Ss_c+Adj,H0,Dis+Fs_c+CenZ,Wd+Ss_c,H0-Up,-Fs_c+CenZ,Wd_c-Ss_c+Adj,H0,Dis-Fs_c-CenZ},
    {-Wd_c-Adj,H0-Up,Dis+CenZ,-Wd,H0,CenZ,-Wd_c-Adj,H0-Up,-Dis+CenZ,Wd_c+Adj,H0,Dis+CenZ,Wd,H0-Up,CenZ,Wd_c+Adj,H0,-Dis+CenZ},
    {-Wd_c-Ss_c-Adj,H0-Up,Dis+Fs_c+CenZ,-Wd+Ss_c,H0,-Fs_c+CenZ,-Wd_c-Ss_c-Adj,H0-Up,-(Dis-Fs_c-CenZ),Wd_c+Ss_c+Adj,H0,Dis-Fs_c+CenZ,Wd-Ss_c,H0-Up,Fs_c+CenZ,Wd_c+Ss_c+Adj,H0,Dis+Fs_c-CenZ},
    {-Wd_c-Ss_c-Adj,H0,Dis+Fs_c+CenZ,-Wd+Ss_c,H0,-Fs_c+CenZ,-Wd_c-Ss_c-Adj,H0,-(Dis-Fs_c-CenZ),Wd_c+Ss_c+Adj,H0,Dis-Fs_c+CenZ,Wd-Ss_c,H0,Fs_c+CenZ,Wd_c+Ss_c+Adj,H0,Dis+Fs_c-CenZ},
    {-Wd_c-Ss_c-Adj,H0,Dis+Fs_c+CenZ,-Wd+Ss_c,H0-Up,-Fs_c+CenZ,-Wd_c-Ss_c-Adj,H0,-(Dis-Fs_c-CenZ),Wd_c+Ss_c+Adj,H0-Up,Dis-Fs_c+CenZ,Wd-Ss_c,H0,Fs_c+CenZ,Wd_c+Ss_c+Adj,H0-Up,Dis+Fs_c-CenZ},
    {-Wd_c-Adj,H0,Dis+CenZ,-Wd,H0-Up,CenZ,-Wd_c-Adj,H0,-Dis+CenZ,Wd_c+Adj,H0-Up,Dis+CenZ,Wd,H0,CenZ,Wd_c+Adj,H0-Up,-Dis+CenZ}};

  float wr_s[8][18]={
    {w_s[0][0]*cos(-TnY_c)-w_s[0][2]*sin(-TnY_c),w_s[0][1],w_s[0][0]*sin(-TnY_c)+w_s[0][2]*cos(-TnY_c),w_s[0][3]*cos(TnY_c)-w_s[0][5]*sin(TnY_c),w_s[0][4],w_s[0][3]*sin(TnY_c)+w_s[0][5]*cos(TnY_c),w_s[0][6]*cos(-TnY_c)-w_s[0][8]*sin(-TnY_c),w_s[0][7],w_s[0][6]*sin(-TnY_c)+w_s[0][8]*cos(-TnY_c),w_s[0][9]*cos(TnY_c)-w_s[0][11]*sin(TnY_c),w_s[0][10],w_s[0][9]*sin(TnY_c)+w_s[0][11]*cos(TnY_c),w_s[0][12]*cos(-TnY_c)-w_s[0][14]*sin(-TnY_c),w_s[0][13],w_s[0][12]*sin(-TnY_c)+w_s[0][14]*cos(-TnY_c),w_s[0][15]*cos(TnY_c)+w_s[0][17]*sin(TnY_c),w_s[0][16],w_s[0][15]*sin(TnY_c)-w_s[0][17]*cos(TnY_c)},
    {w_s[1][0]*cos(-TnY_c)-w_s[1][2]*sin(-TnY_c),w_s[1][1],w_s[1][0]*sin(-TnY_c)+w_s[1][2]*cos(-TnY_c),w_s[1][3]*cos(TnY_c)-w_s[1][5]*sin(TnY_c),w_s[1][4],w_s[1][3]*sin(TnY_c)+w_s[1][5]*cos(TnY_c),w_s[1][6]*cos(-TnY_c)-w_s[1][8]*sin(-TnY_c),w_s[1][7],w_s[1][6]*sin(-TnY_c)+w_s[1][8]*cos(-TnY_c),w_s[1][9]*cos(TnY_c)-w_s[1][11]*sin(TnY_c),w_s[1][10],w_s[1][9]*sin(TnY_c)+w_s[1][11]*cos(TnY_c),w_s[1][12]*cos(-TnY_c)-w_s[1][14]*sin(-TnY_c),w_s[1][13],w_s[1][12]*sin(-TnY_c)+w_s[1][14]*cos(-TnY_c),w_s[1][15]*cos(TnY_c)+w_s[1][17]*sin(TnY_c),w_s[1][16],w_s[1][15]*sin(TnY_c)-w_s[1][17]*cos(TnY_c)},
    {w_s[2][0]*cos(-TnY_c)-w_s[2][2]*sin(-TnY_c),w_s[2][1],w_s[2][0]*sin(-TnY_c)+w_s[2][2]*cos(-TnY_c),w_s[2][3]*cos(TnY_c)-w_s[2][5]*sin(TnY_c),w_s[2][4],w_s[2][3]*sin(TnY_c)+w_s[2][5]*cos(TnY_c),w_s[2][6]*cos(-TnY_c)-w_s[2][8]*sin(-TnY_c),w_s[2][7],w_s[2][6]*sin(-TnY_c)+w_s[2][8]*cos(-TnY_c),w_s[2][9]*cos(TnY_c)-w_s[2][11]*sin(TnY_c),w_s[2][10],w_s[2][9]*sin(TnY_c)+w_s[2][11]*cos(TnY_c),w_s[2][12]*cos(-TnY_c)-w_s[2][14]*sin(-TnY_c),w_s[2][13],w_s[4][12]*sin(-TnY_c)+w_s[2][14]*cos(-TnY_c),w_s[2][15]*cos(TnY_c)+w_s[2][17]*sin(TnY_c),w_s[2][16],w_s[2][15]*sin(TnY_c)-w_s[2][17]*cos(TnY_c)},
    {w_s[3][0],w_s[3][1],w_s[3][2],w_s[3][3],w_s[3][4],w_s[3][5],w_s[3][6],w_s[3][7],w_s[3][8],w_s[3][9],w_s[3][10],w_s[3][11],w_s[3][12],w_s[3][13],w_s[3][14],w_s[3][15],w_s[3][16],w_s[3][17]},
    {w_s[4][0]*cos(TnY_c)-w_s[4][2]*sin(TnY_c),w_s[4][1],w_s[4][0]*sin(TnY_c)+w_s[4][2]*cos(TnY_c),w_s[4][3]*cos(-TnY_c)-w_s[4][5]*sin(-TnY_c),w_s[4][4],w_s[4][3]*sin(-TnY_c)+w_s[4][5]*cos(-TnY_c),w_s[4][6]*cos(TnY_c)-w_s[4][8]*sin(TnY_c),w_s[4][7],w_s[4][6]*sin(TnY_c)+w_s[4][8]*cos(TnY_c),w_s[4][9]*cos(-TnY_c)-w_s[4][11]*sin(-TnY_c),w_s[4][10],w_s[4][9]*sin(-TnY_c)+w_s[4][11]*cos(-TnY_c),w_s[4][12]*cos(-TnY_c)-w_s[4][14]*sin(-TnY_c),w_s[4][13],w_s[4][12]*sin(TnY_c)+w_s[4][14]*cos(TnY_c),w_s[4][15]*cos(-TnY_c)+w_s[4][17]*sin(-TnY_c),w_s[4][16],w_s[4][15]*sin(-TnY_c)-w_s[4][17]*cos(-TnY_c)},
    {w_s[5][0]*cos(TnY_c)-w_s[5][2]*sin(TnY_c),w_s[5][1],w_s[5][0]*sin(TnY_c)+w_s[5][2]*cos(TnY_c),w_s[5][3]*cos(-TnY_c)-w_s[5][5]*sin(-TnY_c),w_s[5][4],w_s[5][3]*sin(-TnY_c)+w_s[5][5]*cos(-TnY_c),w_s[5][6]*cos(TnY_c)-w_s[5][8]*sin(TnY_c),w_s[5][7],w_s[5][6]*sin(TnY_c)+w_s[5][8]*cos(TnY_c),w_s[5][9]*cos(-TnY_c)-w_s[5][11]*sin(-TnY_c),w_s[5][10],w_s[5][9]*sin(-TnY_c)+w_s[5][11]*cos(-TnY_c),w_s[5][12]*cos(-TnY_c)-w_s[5][14]*sin(-TnY_c),w_s[5][13],w_s[5][12]*sin(TnY_c)+w_s[5][14]*cos(TnY_c),w_s[5][15]*cos(-TnY_c)+w_s[5][17]*sin(-TnY_c),w_s[5][16],w_s[5][15]*sin(-TnY_c)-w_s[5][17]*cos(-TnY_c)},
    {w_s[6][0]*cos(TnY_c)-w_s[6][2]*sin(TnY_c),w_s[6][1],w_s[6][0]*sin(TnY_c)+w_s[6][2]*cos(TnY_c),w_s[6][3]*cos(-TnY_c)-w_s[6][5]*sin(-TnY_c),w_s[6][4],w_s[6][3]*sin(-TnY_c)+w_s[6][5]*cos(-TnY_c),w_s[6][6]*cos(TnY_c)-w_s[6][8]*sin(TnY_c),w_s[6][7],w_s[6][6]*sin(TnY_c)+w_s[6][8]*cos(TnY_c),w_s[6][9]*cos(-TnY_c)-w_s[6][11]*sin(-TnY_c),w_s[6][10],w_s[6][9]*sin(-TnY_c)+w_s[6][11]*cos(-TnY_c),w_s[6][12]*cos(-TnY_c)-w_s[6][14]*sin(-TnY_c),w_s[6][13],w_s[6][12]*sin(TnY_c)+w_s[6][14]*cos(TnY_c),w_s[6][15]*cos(-TnY_c)+w_s[6][17]*sin(-TnY_c),w_s[6][16],w_s[6][15]*sin(-TnY_c)-w_s[6][17]*cos(-TnY_c)},
    {w_s[7][0],w_s[7][1],w_s[7][2],w_s[7][3],w_s[7][4],w_s[7][5],w_s[7][6],w_s[7][7],w_s[7][8],w_s[7][9],w_s[7][10],w_s[7][11],w_s[7][12],w_s[7][13],w_s[7][14],w_s[7][15],w_s[7][16],w_s[7][17]}};

  for (int i=0; i <=7 ; i++){
    ik(&wr_s[i][0],&wr_s[i][1],&wr_s[i][2],&X0[0],&Z0[0],&Theta[0],&Theta[1],&Theta[2]); //Right Front
    angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][0], &bs_s[i][1], &bs_s[i][2]);

    ik(&wr_s[i][3],&wr_s[i][4],&wr_s[i][5],&X0[1],&Z0[1],&Theta[0],&Theta[1],&Theta[2]); //Right Mid
    angle_cul_RM(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][3], &bs_s[i][4], &bs_s[i][5]);

    ik(&wr_s[i][6],&wr_s[i][7],&wr_s[i][8],&X0[2],&Z0[2],&Theta[0],&Theta[1],&Theta[2]); //Right Back
    angle_cul_RB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][6], &bs_s[i][7], &bs_s[i][8]);

    ik(&wr_s[i][9],&wr_s[i][10],&wr_s[i][11],&X0[3],&Z0[3],&Theta[0],&Theta[1],&Theta[2]); //Left Front
    angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][9], &bs_s[i][10], &bs_s[i][11]);

    ik(&wr_s[i][12],&wr_s[i][13],&wr_s[i][14],&X0[4],&Z0[4],&Theta[0],&Theta[1],&Theta[2]); //Left Mid
    angle_cul_LM(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][12], &bs_s[i][13], &bs_s[i][14]);

    ik(&wr_s[i][15],&wr_s[i][16],&wr_s[i][17],&X0[5],&Z0[5],&Theta[0],&Theta[1],&Theta[2]); //Left Back
    angle_cul_LB(&Theta[0], &Theta[1], &Theta[2], &bs_s[i][15], &bs_s[i][16], &bs_s[i][17]);
  }

  for (int i=0; i <=7 ; i++){
    for (int j=0; j <=17 ; j++){
      ang1[j] = angZero[j] + bs_s[i][j];
    }
    servo_set();
  }
}

void servo_init_set()
{
  int cn = 10;
  for (int j=0; j <=15; j++){
     servo_write(j,angZero[j]+ angHome[j]);
     delay(cn);
  }
  for (int j=0; j <=1; j++){
     Srv_drive(j, angZero[j+16]+ angHome[j+16]);
     delay(cn);
  }
}

void servo_set()
{
  int a[18],b[18];

  for (int j=0; j <=17 ; j++){
      a[j] = ang1[j] - ang0[j];
      b[j] = ang0[j];
      ang0[j] = ang1[j];//BluetoothSerial SerialBT;
  }
  
  for (int k=0; k <= td; k++){

    // PCA9685
      servo_write(srv_CH0,a[0]*float(k)/td+b[0]);
      servo_write(srv_CH1,a[1]*float(k)/td+b[1]);
      servo_write(srv_CH2,a[2]*float(k)/td+b[2]);
      servo_write(srv_CH3,a[3]*float(k)/td+b[3]);
      servo_write(srv_CH4,a[4]*float(k)/td+b[4]);
      servo_write(srv_CH5,a[5]*float(k)/td+b[5]);
      servo_write(srv_CH6,a[6]*float(k)/td+b[6]);
      servo_write(srv_CH7,a[7]*float(k)/td+b[7]);
      servo_write(srv_CH8,a[8]*float(k)/td+b[8]);
      servo_write(srv_CH9,a[9]*float(k)/td+b[9]);
      servo_write(srv_CH10,a[10]*float(k)/td+b[10]);
      servo_write(srv_CH11,a[11]*float(k)/td+b[11]);
      servo_write(srv_CH12,a[12]*float(k)/td+b[12]);
      servo_write(srv_CH13,a[13]*float(k)/td+b[13]);
      servo_write(srv_CH14,a[14]*float(k)/td+b[14]);
      servo_write(srv_CH15,a[15]*float(k)/td+b[15]);
    // PWM
      Srv_drive(srv_CH16, a[16]*float(k)/td+b[16]);
      Srv_drive(srv_CH17, a[17]*float(k)/td+b[17]);

      delay(ts/td);
  }
}

void angle_cul_RF(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  *bss_0 = 120 - int(*Theta_3);
  *bss_1 = -int(*Theta_1);
  *bss_2 = int(*Theta_2 - Nee) - 90;
}

void angle_cul_RM(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  if(*Theta_3 > 0){
    *bss_0 = 180 - int(*Theta_3);
  }
  else{
    *bss_0 = -180 - int(*Theta_3);
  }
  *bss_1 = -int(*Theta_1);
  *bss_2 = int(*Theta_2 - Nee) - 90;
}

void angle_cul_RB(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  *bss_0 = -120 - int(*Theta_3);
  *bss_1 = -int(*Theta_1);
  *bss_2 = -90 + int(*Theta_2 - Nee);
}

void angle_cul_LF(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  *bss_0 = 60 -int(*Theta_3);
  *bss_1 = int(*Theta_1);
  *bss_2 = 90 - int(*Theta_2 - Nee);
}

void angle_cul_LM(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  *bss_0 = -int(*Theta_3);
  *bss_1 = int(*Theta_1);
  *bss_2 = 90 - int(*Theta_2 - Nee);
}

void angle_cul_LB(float *Theta_1, float *Theta_2, float *Theta_3, int *bss_0, int *bss_1, int *bss_2)
{
  *bss_0 = -60 -int(*Theta_3);
  *bss_1 = int(*Theta_1);
  *bss_2 = 90 - int(*Theta_2 - Nee);
}

void rot_head_step(float TnX, float TnY, float TnZ)
{
  float Wd_c = Wd*cos(Pi/3);
  float ch_s[18] = {-Wd_c-Adj,H0,Dis+CenZ,-Wd,H0,CenZ,-Wd_c-Adj,H0,-Dis+CenZ,Wd_c+Adj,H0,Dis+CenZ,Wd,H0,CenZ,Wd_c+Adj,H0,-Dis+CenZ};
  
  rot(&bh_s[0], &bh_s[1], &bh_s[2], &ch_s[0], &ch_s[1], &ch_s[2], &TnX, &TnY, &TnZ);
  rot(&bh_s[3], &bh_s[4], &bh_s[5], &ch_s[3], &ch_s[4], &ch_s[5], &TnX, &TnY, &TnZ);
  rot(&bh_s[6], &bh_s[7], &bh_s[8], &ch_s[6], &ch_s[7], &ch_s[8], &TnX, &TnY, &TnZ);
  rot(&bh_s[9], &bh_s[10], &bh_s[11], &ch_s[9], &ch_s[10], &ch_s[11], &TnX, &TnY, &TnZ);
  rot(&bh_s[12], &bh_s[13], &bh_s[14], &ch_s[12], &ch_s[13], &ch_s[14], &TnX, &TnY, &TnZ);
  rot(&bh_s[15], &bh_s[16], &bh_s[17], &ch_s[15], &ch_s[16], &ch_s[17], &TnX, &TnY, &TnZ);
  
  ik(&bh_s[0],&bh_s[1],&bh_s[2],&X0[0],&Z0[0],&Theta[0],&Theta[1],&Theta[2]); //Right Front
  angle_cul_RF(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][0], &bs_s[0][1], &bs_s[0][2]);

  ik(&bh_s[3],&bh_s[4],&bh_s[5],&X0[1],&Z0[1],&Theta[0],&Theta[1],&Theta[2]); //Right Mid
  angle_cul_RM(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][3], &bs_s[0][4], &bs_s[0][5]);

  ik(&bh_s[6],&bh_s[7],&bh_s[8],&X0[2],&Z0[2],&Theta[0],&Theta[1],&Theta[2]); //Right Back
  angle_cul_RB(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][6], &bs_s[0][7], &bs_s[0][8]);

  ik(&bh_s[9],&bh_s[10],&bh_s[11],&X0[3],&Z0[3],&Theta[0],&Theta[1],&Theta[2]); //Left Front
  angle_cul_LF(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][9], &bs_s[0][10], &bs_s[0][11]);

  ik(&bh_s[12],&bh_s[13],&bh_s[14],&X0[4],&Z0[4],&Theta[0],&Theta[1],&Theta[2]); //Left Mid
  angle_cul_LM(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][12], &bs_s[0][13], &bs_s[0][14]);

  ik(&bh_s[15],&bh_s[16],&bh_s[17],&X0[5],&Z0[5],&Theta[0],&Theta[1],&Theta[2]); //Left Back
  angle_cul_LB(&Theta[0], &Theta[1], &Theta[2], &bs_s[0][15], &bs_s[0][16], &bs_s[0][17]);
  
  for (int j=0; j <=17 ; j++){
    ang1[j] = angZero[j] + bs_s[0][j];
  }
  servo_set();
}

//I2c書き込み MPU6050
void writeMPU6050(byte reg, byte data) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

//i2C読み込み MPU6050
byte readMPU6050(byte reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(true);
  Wire.requestFrom(MPU6050_ADDR, 1/*length*/); 
  byte data =  Wire.read();
  return data;
}

void IMU_Calibration()
{
  //キャリブレーション
  Serial.print("Calculate Calibration");
  for(int i = 0; i < 3000; i++){
    
    int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
    
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 14, true);
  
    raw_acc_x = Wire.read() << 8 | Wire.read();
    raw_acc_y = Wire.read() << 8 | Wire.read();
    raw_acc_z = Wire.read() << 8 | Wire.read();
    raw_t = Wire.read() << 8 | Wire.read();
    raw_gyro_x = Wire.read() << 8 | Wire.read();
    raw_gyro_y = Wire.read() << 8 | Wire.read();
    raw_gyro_z = Wire.read() << 8 | Wire.read();
    dpsX = ((float)raw_gyro_x) / 65.5;
    dpsY = ((float)raw_gyro_y) / 65.5;
    dpsZ = ((float)raw_gyro_z) / 65.5;
    offsetX += dpsX;
    offsetY += dpsY;
    offsetZ += dpsZ;
    
    //単位Gへ変換
    acc_x = ((float)raw_acc_x) / 16384.0;
    acc_y = ((float)raw_acc_y) / 16384.0;
    acc_z = ((float)raw_acc_z) / 16384.0;
  
    //加速度センサーから角度を算出
    acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
    acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;
    offsetX_acc += acc_angle_x;
    offsetY_acc += acc_angle_y;
  }

  offsetX /= 3000;
  offsetY /= 3000;
  offsetZ /= 3000;
  offsetX_acc /= 3000;
  offsetY_acc /= 3000;
}

void setup() 
{ 
  Serial.begin(151200);
  VSerial.begin(38400, SERIAL_8N1, 20, 21); //G20 Rx0 G21 Tx0
  
  setupBLE();

  pinMode(Srv16, OUTPUT);
  pinMode(Srv17, OUTPUT);

  //モータのPWMのチャンネル、周波数の設定
  ledcSetup(srv_CH16, PWM_Hz, PWM_level);
  ledcSetup(srv_CH17, PWM_Hz, PWM_level);

  //モータのピンとチャンネルの設定
  ledcAttachPin(Srv16, srv_CH16);
  ledcAttachPin(Srv17, srv_CH17);
  
  Wire.begin(8, 9); //SDA-8, SCL-9
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  pwm.begin();                   //initial setting (for 0x40) (PCA9685)
  pwm.setPWMFreq(50);            //PWM 50Hz (for 0x40) (PCA9685)

  //initial servo angle
  for (int j=0; j <=17 ; j++){
      ang0[j] = angZero[j] + angHome[j];
  }
  for (int j=0; j <=17 ; j++){
      ang1[j] = angZero[j] + angHome[j];
  }
  servo_init_set();

  lcd.init();

  sprite.createSprite(240,240);
  sprite2.createSprite(140,140);
  eye();
  sprite2.pushSprite(50,50);
  sprite.pushSprite(0,0);

    //正常に接続されているかの確認
  if (readMPU6050(MPU6050_WHO_AM_I) != 0x68) {
    Serial.println("\nWHO_AM_I error.");
    while (true) ;
  }

  //設定を書き込む
  writeMPU6050(MPU6050_SMPLRT_DIV, 0x00);   // sample rate: 8kHz/(7+1) = 1kHz
  writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
  writeMPU6050(MPU6050_GYRO_CONFIG, 0x08);  // gyro range: ±500dps
  writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
  writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro

  //キャリブレーション
  IMU_Calibration();
} 

v_response_t v_data;    // Data read back from V

//加速度、ジャイロから角度を計算
void calcRotation(){ //MPU6050 X-> Z ,Y -> X ,Z -> Y  

  int16_t raw_acc_x, raw_acc_y, raw_acc_z, raw_t, raw_gyro_x, raw_gyro_y, raw_gyro_z ;
  
  //レジスタアドレス0x3Bから、計14バイト分のデータを出力するようMPU6050へ指示
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
  
  //出力されたデータを読み込み、ビットシフト演算
  raw_acc_x = Wire.read() << 8 | Wire.read();
  raw_acc_y = Wire.read() << 8 | Wire.read();
  raw_acc_z = Wire.read() << 8 | Wire.read();
  raw_t = Wire.read() << 8 | Wire.read();
  raw_gyro_x = Wire.read() << 8 | Wire.read();
  raw_gyro_y = Wire.read() << 8 | Wire.read();
  raw_gyro_z = Wire.read() << 8 | Wire.read();
  
  //単位Gへ変換
  acc_x = ((float)raw_acc_x) / 16384.0;
  acc_y = ((float)raw_acc_y) / 16384.0;
  acc_z = ((float)raw_acc_z) / 16384.0;
  
  //加速度センサーから角度を算出
  acc_angle_y = atan2(acc_x, acc_z + abs(acc_y)) * 360 / -2.0 / PI;
  acc_angle_x = atan2(acc_y, acc_z + abs(acc_x)) * 360 / 2.0 / PI;

  dpsX = ((float)raw_gyro_x) / 65.5; // LSB sensitivity: 65.5 LSB/dps @ ±500dps
  dpsY = ((float)raw_gyro_y) / 65.5;
  dpsZ = ((float)raw_gyro_z) / 65.5;
  
  //前回計算した時から今までの経過時間を算出
  interval = millis() - preInterval;
  preInterval = millis();
  
  //数値積分
  gyro_angle_x += (dpsX - offsetX) * (interval * 0.001);
  gyro_angle_y += (dpsY - offsetY) * (interval * 0.001);
  gyro_angle_z += (dpsZ - offsetZ) * (interval * 0.001);
  
  //相補フィルター
  angleX = (0.9 * gyro_angle_x) + (0.1 * (acc_angle_x - offsetX_acc));
  angleY = (0.9 * gyro_angle_y) + (0.1 * (acc_angle_y - offsetY_acc));
  angleZ = gyro_angle_z;
  gyro_angle_x = angleX;
  gyro_angle_y = angleY;
  gyro_angle_z = angleZ;

  head_IMU_x = head_IMU_x + int(af_imu * angleY);
  head_IMU_z = head_IMU_z + int(af_imu * angleX);
  if (head_IMU_x >= 15) head_IMU_x = 15; 
  if (head_IMU_x <= -15) head_IMU_x = -15;
  if (head_IMU_z >= 15) head_IMU_z = 15; 
  if (head_IMU_z <= -15) head_IMU_z = -15;
}

void loop()
{ 
  VSerial.write(0xAF);

  if(VSerial.available())
  {
    uint8_t buffer[4];
    VSerial.readBytes(buffer, 4);
    v_data.dx = (buffer[0] << 8) | buffer[1];
    v_data.dy = (buffer[2] << 8) | buffer[3];

    eye_u_x = map(v_data.dx, 0, 320, 0, 100);
    eye_u_y = map(v_data.dy, 0, 240, 0, 100);
    
    float head_dx = map(v_data.dx, 320, 0, -20, 20);
    float head_dy = map(v_data.dy, 0, 240, -20, 20);
    head_UnitV_x = head_UnitV_x + int(af * head_dx);
    head_UnitV_y = head_UnitV_y + int(af * head_dy);
    if (head_UnitV_x >= 20) head_UnitV_x = 20; 
    if (head_UnitV_x <= -20) head_UnitV_x = -20;
    if (head_UnitV_y >= 20) head_UnitV_y = 20; 
    if (head_UnitV_y <= -20) head_UnitV_y = -20;
  }
  
  checkBLE();
  
  int rad = (int)map(joyLDistance, 0, 100, 0, 10);
  int eye_x = (int)map(joyLX, 200, 0, 100, 0);
  int eye_y = (int)map(joyLY, 0, 200, 100, 0);
  int head_x = (int)map(joyLY, 0, 200, -20, 20);
  int head_y = (int)map(joyLX, 200, 0, -20, 20);
  int head_z = (int)map(joyLX, 200, 0, -20, 20);
  int walk_fs = (int)map(joyLY, 0, 200, -20, 20);
  int walk_ss = (int)map(joyLX, 200, 0, -20, 20);
  int walk_ts = (int)map(joyLX, 200, 0, -10, 10);

  if (rad >2)
  {
    if(walk_mode == 1)
    {
      if(stay_mode == 1)
      {
        Motion_eye(eye_x,eye_y);
        rot_head_step(-head_x, head_y, 0.0);
      }
      if(stay_mode == -1)
      {
        Motion_eye(eye_x,eye_y);
        rot_head_step(-head_x, 0.0, head_z);
      }
    }
    if(walk_mode == -1)
    {
      if(stay_mode == 1)
      {
        Motion_eye(eye_x,50);
        if(walk_status == 0)
        {
          walk_1st_s(walk_fs, walk_ss, 0.0);
          walk_status = 1;
        } else if (walk_status = 1) {
          walk_s(walk_fs, walk_ss, 0.0);
        }
      }
      if(stay_mode == -1)
      {
        Motion_eye(eye_x,50);
        if(walk_status == 0)
        {
          walk_1st_s(walk_fs, walk_ss, 0.0);
          walk_status = 1;
        } else if (walk_status = 1) {
          walk_s(walk_fs, 0.0, walk_ts);
        }
      }  
    }
    UnitV_mode = 0;
    IMU_mode = 0;
  }

  if (rad <= 2)
  {
    if(UnitV_mode == 1)
    {
      Motion_eye(eye_u_x,eye_u_y);
      rot_head_step(head_UnitV_y, head_UnitV_x, 0.0);
    } else if(IMU_mode == 1)
    {
      Motion_eye(50,50);
      rot_head_step(head_IMU_x, 0.0, head_IMU_z);
    }else{
      Motion_eye(50,50); //Center Eye
      rot_head_step(0.0, 0.0, 0.0);
    //Serial.println("Center");
    }
    walk_status = 0;
  }

  if (joyRY < 20)
  {
    H0 += -2;
    Up += -2;
    //Serial.println("Height Down");
  }

  if (joyRY > 180)
  {
    H0 += 2;
    Up += 2;
    //Serial.print("Height Up");
  }

  if (joyLSW == 1)
  {
    Sleep_eye();
    UnitV_mode = 1;
    IMU_mode = 0;
  }

  if (joyRSW == 1)
  {
    Sleep_eye();
    IMU_mode = 1;
    UnitV_mode = 0;
  }

  if (joyRX > 150)
  {
    Sleep_eye();
    stay_mode = -1*stay_mode;
    UnitV_mode = 0;
    IMU_mode = 0;
  }

  if (joyRX < 50)
  {
    Sleep_eye();
    walk_mode = -1*walk_mode;
    UnitV_mode = 0;
    IMU_mode = 0;
  }

  calcRotation();

}

void setupBLE() {
  BLEDevice::init("NX19_M5StampC3");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharTx = pService->createCharacteristic(CHTX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharRx = pService->createCharacteristic(CHRX_UUID, BLECharacteristic::PROPERTY_WRITE_NR);
  pCharRx ->setCallbacks(new MyCallbacks());
  pCharTx->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void checkBLE() {
    // notify changed value
  if (deviceConnected) {
      pCharTx->setValue((uint8_t*)&value, 6);
      pCharTx->notify();
  }
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}
