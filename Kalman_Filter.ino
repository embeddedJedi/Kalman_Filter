
/* Bu kod, bir MPU6050 IMU sensörünün yönünü (açılarını) tahmin etmek için bir Kalman filtresi uygulamasıdır. 
MPU6050, ivmeyi ve dönüşü ölçen 6 eksenli bir cihazdır. Kod, önce MPU6050 ile I2C iletişimini başlatır ve seri iletişimi kurar. 
Daha sonra, daha sonraki hesaplamalarda ham verilerden çıkarılan ivme ve dönüşün ofset değerlerini hesaplar. 
Döngüde kod, MPU6050'den gelen ham verileri okur ve ivme verilerini açılara 
dönüştürür. Açılar, ağırlıklı hareketli ortalama filtresi kullanılarak yumuşatılır 
ve açıları daha fazla tahmin etmek için Kalman filtresi kullanılır. 
Nihai tahmin edilen açılar daha sonra ileri işleme veya kontrol için kullanılır. */

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"

MPU6050 mpu;

int x_ort = 0;      // x ekseni toplamı
int y_ort = 0;      // y ekseni toplamı
int z_ort = 0;      // z ekseni toplamı
int x_dongu = 0;  // x ekseni ortalaması
int y_dongu = 0;  // y ekseni ortalaması
int z_dongu = 0;  // z ekseni ortalaması
int count = 0;            // örnek sayacı

unsigned long now, lastTime = 0;
float dt;                                   // Delta zamanı

int16_t ax, ay, az, gx, gy, gz;             // mpu veriler
float aax=0, aay=0,aaz=0, agx=0, agy=0, agz=0;    // Açılar için değişken
long axo = 0, ayo = 0, azo = 0;             //ivme offset
long gxo = 0, gyo = 0, gzo = 0;             //gyro ofset

float son_x, son_y, son_z;
float pi = 3.1415926;
float AcceRatio = 16384.0;                  //İvme aralık / ölçek
float GyroRatio = 131.0;                    

uint8_t n_sample = 8;                       // Örnek numarası sayac
float aaxs[8] = {0}, aays[8] = {0}, aazs[8] = {0};         //x, y ekseni örnekleme sırası
long aax_sum, aay_sum,aaz_sum;                      

float a_x[10]={0}, a_y[10]={0},a_z[10]={0} ,g_x[10]={0} ,g_y[10]={0},g_z[10]={0}; //    Hesaplama sırası
float Px=1, Rx, Kx, Sx, Vx, Qx;             // KalmanX
float Py=1, Ry, Ky, Sy, Vy, Qy;             // KalmanY
float Pz=1, Rz, Kz, Sz, Vz, Qz;             // KalmanZ

void setup()
{
    Wire.begin();
    Serial.begin(115200);

    mpu.initialize();                 // mpu start

    unsigned short times = 200;             // timer
    for(int i=0;i<times;i++)
    {
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // eksen verilerini oku
        axo += ax; ayo += ay; azo += az;      
        gxo += gx; gyo += gy; gzo += gz;
    
    }
    
    axo /= times; ayo /= times; azo /= times; // ofset hesaplamaya başla
    gxo /= times; gyo /= times; gzo /= times; 
}

void loop()
{
    unsigned long now = millis();             // ms şimdiki zamandaki
    dt = (now - lastTime) / 1000.0;           // s ileri zamandakiler
    lastTime = now;                           // ms Son zamandaki

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); 

    float accx = ax / AcceRatio;              
    float accy = ay / AcceRatio;              
    float accz = az / AcceRatio;              

    aax = atan(accy / accz) * (-180) / pi;    //y ile z ekseni arasındaki açı
    aay = atan(accx / accz) * 180 / pi;       //x ile z ekseni arasındaki açı
    aaz = atan(accz / accy) * 180 / pi;       //z ile y ekseni arasındaki açı

    aax_sum = 0;                              // Algoritma
    aay_sum = 0;
    aaz_sum = 0;
  
    for(int i=1;i<n_sample;i++)
    {
        aaxs[i-1] = aaxs[i];
        aax_sum += aaxs[i] * i;
        aays[i-1] = aays[i];
        aay_sum += aays[i] * i;
        aazs[i-1] = aazs[i];
        aaz_sum += aazs[i] * i;
    
    }
    
    aaxs[n_sample-1] = aax;
    aax_sum += aax * n_sample;
    aax = (aax_sum / (11*n_sample/2.0)) * 9 / 7.0; //0-90 açı
    aays[n_sample-1] = aay;                        // v  katsayısı elde ediliyor
    aay_sum += aay * n_sample;                     
    aay = (aay_sum / (11*n_sample/2.0)) * 9 / 7.0;
    aazs[n_sample-1] = aaz; 
    aaz_sum += aaz * n_sample;
    aaz = (aaz_sum / (11*n_sample/2.0)) * 9 / 7.0;

    float gyrox = - (gx-gxo) / GyroRatio * dt; //x hız
    float gyroy = - (gy-gyo) / GyroRatio * dt; //y hız
    float gyroz = - (gz-gzo) / GyroRatio * dt; //z hız
    agx += gyrox;
    agy += gyroy;                            
    agz += gyroz;
    
    /* kalman start */
    Sx = 0; Rx = 0;
    Sy = 0; Ry = 0;
    Sz = 0; Rz = 0;
    
    for(int i=1;i<10;i++) // Ortalamalarını al
    {                 
        a_x[i-1] = a_x[i];                      // ortalama hızlanma
        Sx += a_x[i];
        a_y[i-1] = a_y[i];
        Sy += a_y[i];
        a_z[i-1] = a_z[i];
        Sz += a_z[i];
    
    }
    
    a_x[9] = aax;
    Sx += aax;
    Sx /= 10;                                 // Ortalama x
    a_y[9] = aay;
    Sy += aay;
    Sy /= 10;                                 // Ortalama y
    a_z[9] = aaz;
    Sz += aaz;
    Sz /= 10;

    for(int i=0;i<10;i++)
    {
        Rx += sq(a_x[i] - Sx);
        Ry += sq(a_y[i] - Sy);
        Rz += sq(a_z[i] - Sz);
    
    }
    
    Rx = Rx / 9;                              
    Ry = Ry / 9;                        
    Rz = Rz / 9;
  
    Px = Px + 0.0025;                         
    Kx = Px / (Px + Rx);                      
    agx = agx + Kx * (aax - agx);             
    Px = (1 - Kx) * Px;                       

    Py = Py + 0.0025;
    Ky = Py / (Py + Ry);
    agy = agy + Ky * (aay - agy); 
    Py = (1 - Ky) * Py;
  
    Pz = Pz + 0.0025;
    Kz = Pz / (Pz + Rz);
    agz = agz + Kz * (aaz - agz); 
    Pz = (1 - Kz) * Pz;


    /* kalman end */

  son_x = map(agx, -1800 , 1800, -240, 90);
  son_y = map(agy, -1800, 1800, -240, 90);
  son_z = map(agz, -1800 , 1800, -240, 90); 


 
  Serial.println(son_x);Serial.println("-------");
  Serial.println(son_y);Serial.println("--------");
  Serial.println(son_z);Serial.println(); 

  delay(1700); 
    
}
