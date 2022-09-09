#include "EEPROMAnything.h"         
int buzzer = 5;

#define PWM_max_value      255
#define PWM_min_value      30
#define PWM_value    254


int PWM_IN_MAX = 2000;
int PWM_IN_MIN = 1000;
int PWM_IN_MIN_ADRESS = 2;
int PWM_IN_MAX_ADRESS = 7;
int MIN_PWM_TO_STORE = 0;
int MAX_PWM_TO_STORE = 0;
int pwm_set_counter = 0;
int beeping_PWM_VALUE = 100;

byte sequence_step = 0, motor_speed; //Döngüler için sabit bir değişken atanmıştır.
unsigned int i;
//PWM giriş sinyalinin zaman genişliği değeri için değişkenler oluşturuyoruz
unsigned long counter_1, current_count;
byte last_PWM_state;
//1000us ile 2000us arasındaki değeri saklamak için değişkenler oluşturuyoruz
int PWM_INPUT=1119;      //PWM_IN pini Arduino'nun D8'idir.
bool MOTOR_SPINNING = false;
bool ESC_MODE_ON = false;
int motor_off_counter = 0;
bool PWM_RANGE_SET = false;
unsigned long previousMillis = 0; 
unsigned long currentMillis = 0;


void setup() {
  pinMode(buzzer,OUTPUT); 
  //Bu kod, kodu yükledikten sonra yalnızca bir kez çalışır
  if( EEPROM.read(1) != 1)
  {
    EEPROM_writeAnything(PWM_IN_MIN_ADRESS, PWM_IN_MIN);         
    EEPROM_writeAnything(PWM_IN_MAX_ADRESS, PWM_IN_MAX);  
    EEPROM.write(1, 1);
  }    
  Serial.begin(9600);
 
  // pins for the MNSFET drivers  2,3,4 and 9,10,11
   DDRD  |= B00011100; // D2,D3,D4 output olarak ayarlandı
  PORTD = 0x00; // D2,D3,D4 LOW'a ayarlandı
  DDRB |= 0X0E; // D9,D10,D11 output olarak ayarlandı
  PORTB = 0X00; // D9,D10,D11 LOW'a ayarlandı
  TCCR1A = 0; // timer 1 pininin clock'u sıfıra çekildi
  TCCR1B = 0x01; // timer 2 pini PWM pinini olarak kullanılması sağlandı
  TCCR2A = 0; // timer 2 pininin clock'u sıfıra çekildi
  TCCR2B = 0x01; // timer 2 pini PWM pinini olarak kullanılması sağlandı
  ACSR   = 0x10;  //ACI - Analog Karşılaştırıcı Kesme Flag'ı aktif hale getirilmiştir.
  PCICR |= 0x01;  //PCMSK0 aktif hale getirilmiştir.                                                 
  PCMSK0 |= 0x01; //Digital 8 pini kesme olarak ayarlanmıştır.

  
  delay(200);
  //Güç verildiğinde
  if(PWM_INPUT > PWM_IN_MIN + 120)
  {
     ESC_MODE_ON = false;           //Konfigürasyon modu tamamlanana kadar motor dönüşü KAPALI
    while(PWM_RANGE_SET)
    { 
      currentMillis = millis();
      if(currentMillis >= 500+previousMillis)
      {
        OCR1A = beeping_PWM_VALUE;
        previousMillis += 500;    
        buzz();         
      }
      if(PWM_INPUT > MAX_PWM_TO_STORE)
      {
        MAX_PWM_TO_STORE = PWM_INPUT;
      }
      
      if(PWM_INPUT < 2000)
      {
        if(pwm_set_counter > 1000)
        {
          MIN_PWM_TO_STORE = PWM_INPUT;
          EEPROM_writeAnything(PWM_IN_MIN_ADRESS, MIN_PWM_TO_STORE);  // Gelen PWM degerleri EEPROM'a kaydedilir.
               
          EEPROM_writeAnything(PWM_IN_MAX_ADRESS, MAX_PWM_TO_STORE); 
          ESC_MODE_ON = true;
          PWM_RANGE_SET = true;
          int y = 0;
          buzz();
          while(y < 3)
          {
            buzz();
            y = y + 1;
          }          
        }
        pwm_set_counter = pwm_set_counter + 1;
        delay(1);
      }
      else
      {
        pwm_set_counter = 0;
      }     
    }//end of !PWM_RANGE_SET
  }

  /*Aralık PWM_IN_MIN+115us'un altındaysa kodu başlatılır*/
  else
  {
    OCR1A = beeping_PWM_VALUE;
    ESC_MODE_ON = true;
    
    buzz();
    for(int x =0; x < 3; x++)
    {
      buzz();
    }
  }
  
//Yeni aralığı EEPROM'a kaydet  
EEPROM_readAnything(PWM_IN_MIN_ADRESS, PWM_IN_MIN);
EEPROM_readAnything(PWM_IN_MAX_ADRESS, PWM_IN_MAX);
}





// Analog karşılaştırıcı için kesme vektörü





void buzz()
{
  digitalWrite(buzzer,HIGH);
    PORTD = 0x10;      //D4'ü HIGH ve diğerleri LOW     
    TCCR1A =  0x81;    //OC1A - D9,  aşağı sayma, PWM 8-bit eşleşmesi.
    
    delay(500);   
    digitalWrite(buzzer,LOW);
    PORTD = 0x00;      //D4'ü HIGH ve digerleri LOW
    TCCR1A =  0;       //OC1A - D9,  aşağı sayma PWM
    delay(200);  
  
  
}


//Motor komutasyonu için gereken 6 durumun fonksiyonu
void set_next_step(){        
  switch(sequence_step){
    case 0:
      AH_BL();
      BEMF_C_RISING();
      break;
    case 1:
      AH_CL();
      BEMF_B_FALLING();
      break;
    case 2:
      BH_CL();
      BEMF_A_RISING();
      break;
    case 3:
      BH_AL();
      BEMF_C_FALLING();
      break;
    case 4:
      CH_AL();
      BEMF_B_RISING();
      break;
    case 5:
      CH_BL();
      BEMF_A_FALLING();
      break;
  }
}








void loop() {

  /*PWM girişi PWM_IN_MIN + 115'den büyükse motor çalışır*/
  if(PWM_INPUT > (PWM_IN_MIN + 115) && ESC_MODE_ON)
  {

    MOTOR_SPINNING = true;    
    motor_off_counter = 0;
    digitalWrite(buzzer,LOW); 
  }
  while( PWM_INPUT <(PWM_IN_MIN +115)&& ESC_MODE_ON)
  {
    Kalibrasyon();
  }

  //////////////////////////Motor dönerken////////////////////////
  if(MOTOR_SPINNING=true)
  {  
    SET_PWM(PWM_value);     
    
    // Motor başlangıcı
    for(int i=2200; i > 100; i = i + 20) {
      delayMicroseconds(i);
      set_next_step();
      sequence_step++;
      sequence_step %= 6;
       //Serial.begin(57600);
      //Serial.print("Motor start");
    }
    motor_speed = PWM_value;
    ACSR |= 0x08;                    // Analog karşılaştırıcı kesmesini açmak için kullanılır.
    while(MOTOR_SPINNING=true) 
    {
      PWM_INPUT = constrain(PWM_INPUT,PWM_IN_MIN,PWM_IN_MAX);
      motor_speed = map(PWM_INPUT,PWM_IN_MIN,PWM_IN_MAX,PWM_min_value,PWM_max_value);
      SET_PWM(motor_speed);
      if(PWM_INPUT < (PWM_IN_MIN + 30))
      {
        while(motor_off_counter > 1000)
        {
          MOTOR_SPINNING = false;
          motor_off_counter = 0;
          PORTD = 0x00;      //Digital 4 LOW'a çekilir.        
          TCCR1A =  0x00;    //Digtal  9 LOW'a çekilir.
      
        }
        motor_off_counter = motor_off_counter + 1;       
      }
      //Serial.print(PWM_IN_MIN);Serial.print("    ");Serial.println(PWM_IN_MAX);
      //Serial.println(motor_speed);
     
    }
  }
  else if(MOTOR_SPINNING=false)
   {      
    unsigned long currentMillis = millis();
    if(currentMillis - previousMillis >= 2000){  
        
      digitalWrite(buzzer,HIGH); 
      ACSR   = 0x10;            // Analog karşılaştırıcı kesmesini devre dışı bırakılır.
      TCCR2A =  0x00;
      TCCR1A =  0x00;           // timer pinleri kapatılır.
      PORTD  = 0x00;            // 0'dan 7'ye kadar olan pinler LOW olarak ayarlandı.    
      PORTB  &= 0x31;           // D9, D10 ve D11 LOW olarak ayarlandı.
      OCR1A = beeping_PWM_VALUE; 
      PORTD = 0x10;      //D4'ü HIGH ve gerisini LOW olarak ayarlayın       
      TCCR1A =  0x81;         //OC1A - D9, ters çevirmeyen mod, aşağı sayma, PWM 8-bit eşleşmesini karşılaştır        
    }
    while(currentMillis >= 2100+previousMillis){  
      previousMillis += 2000;    
      digitalWrite(buzzer,LOW);   
      PORTD = 0x00;      //LOW olarak ayarlandı.
      TCCR1A =  0x00;    
    }      
  }


  //////////////////////////Motor'un durması////////////////////////
  
}










// Her adımda A,B,C'de zero cross'un yükselip veya azalacağı durumlar belirtilmiştir.
void BEMF_A_RISING(){  
  ADCSRA = 0x00;     // ADC modülünü kapatır.
  ADCSRB = 0x40;     // MUX - inputu karşılaştırıcı için seçer.
  ADMUX = 2;         // A2 MUX tarafından negatif input olarak seçilir.
  ACSR |= 0x03;      // Kesme yükselen kenarlı olarak seçilir.
}
void BEMF_A_FALLING(){
  ADCSRA = 0x00;      // ADC modülünü kapatır.
  ADCSRB = 0x40;      // MUX - inputu karşılaştırıcı için seçer.
  ADMUX = 2;          // A2 MUX tarafından negatif input olarak seçilir.
  ACSR &= ~0x01;      // Kesme düşen kenarlı olarak seçilir.
}
void BEMF_B_RISING(){
  ADCSRA = 0x00;      // ADC modülünü kapatır.
  ADCSRB = 0x40;      // MUX - inputu karşılaştırıcı için seçer.
  ADMUX = 1;          // A1 MUX tarafından negatif input olarak seçilir.
  ACSR |= 0x03;       // Kesme düşen kenarlı olarak seçilir.
}
void BEMF_B_FALLING(){
  ADCSRA = 0x00;      // ADC modülünü kapatır.
  ADCSRB = 0x40;      // MUX - inputu karşılaştırıcı için seçer.
  ADMUX = 1;          // A1 MUX tarafından negatif input olarak seçilir.
  ACSR &= ~0x01;      // Kesme düşen kenarlı olarak seçilir.
}
void BEMF_C_RISING(){
  ADCSRA = 0x00;      // ADC modülünü kapatır.
  ADCSRB = 0x40;      // MUX - inputu karşılaştırıcı için seçer.
  ADMUX = 0;          // A0 MUX tarafından negatif input olarak seçilir.
  ACSR |= 0x03;       // Kesme düşen kenarlı olarak seçilir.
}
void BEMF_C_FALLING(){
  ADCSRA = 0x00;      // ADC modülünü kapatır.
  ADCSRB = 0x40;      // MUX - inputu karşılaştırıcı için seçer.
  ADMUX = 0;          // A0 MUX tarafından negatif input olarak seçilir.
  ACSR &= ~0x01;      // Kesme düşen kenarlı olarak seçilir.
}










//Her adımda motorun fazlara göre kombinasyonları yapılarak dönüşü için gereken 6 durum oluşturulmuştur.
//D9 PWM and D3 HIGH.  
void AH_BL(){
  PORTD = 0x08;      //Digtal 3 aktif edlir.(BL)
  TCCR2A =  0x00;       //Digital 11 0'a çekilir.
  TCCR1A =  0x81;    //Digital 9'dan 8 bitlik PWM verilir. (AH)
}
//D9 PWM ve D2 HIGH
void AH_CL(){
  PORTD = 0x04;       //Digtal 2 aktif edlir.(CL)
  TCCR2A =  0x00;       //Digital 11 0'a çekilir.
  TCCR1A =  0x81;    //Digital 9'dan 8 bitlik PWM verilir. (AH)
}
//D10 PWM ve D2 HIGH
void BH_CL(){
  PORTD = 0x04;      //Digital 2 aktif edilir.(CL)
  TCCR2A =  0x00;       //Digital 11 0'a çekilir.
  TCCR1A =  0x21;    //Digital 10'dan 8 bitlik PWM verilir. (BH)
}
//D10 PWM ve D4 HIGH
void BH_AL(){  
  PORTD = 0x10;      //Digital 4 aktif edilir.(AL)
  TCCR2A =  0x00;       //Digital 11 0'a çekilir.
  TCCR1A =  0x21;    // Digital 10'dan 8 bitlik PWM verilir. (BH) 
}
//D11 PWM ve D4 HIGH
void CH_AL(){ 
  PORTD = 0x10;      //Digital 4 aktif edilir.(AL)
  TCCR1A =  0x00;    //Digital 10 0'a çekilir.
  TCCR2A =  0x81;    // Digital 11'dan 8 bitlik PWM verilir. (CH) 
}
//D11 PWM ve D3 HIGH
void CH_BL(){  
  PORTD = 0x08;      //Digital 3 aktif edilir.(BL)
  TCCR1A =  0x00;       //Digital 10 0'a çekilir.
  TCCR2A =  0x81;    // Digital 11'dan 8 bitlik PWM verilir. (CH)
}








/*Bu fonksiyon PWM değerlerini sadece pin D8'de okunan PWM tarafından verilen width_value göre değiştirecektir*/
void SET_PWM(byte width_value){
  //PWM aralığını min ve max arasında tutuyoruz (8 bit value)
  if(width_value < PWM_min_value)    width_value  = PWM_min_value;
  if(width_value > PWM_max_value)    width_value  = PWM_max_value;
  OCR1A  = width_value;                   //pin 9  PWM duty cycle değeri atama
  OCR1B  = width_value;                   //pin 10 PWM duty cycle değeri atama
  OCR2A  = width_value;                   //pin 11 PWM duty cycle değeri atama 
}



void Kalibrasyon(int DELAY)
{
  DELAY=motor_speed
  while (DELAY<999)
   
  {
    
  PWM_INPUT = constrain(PWM_INPUT,PWM_IN_MIN,PWM_IN_MAX);
  motor_speed = map(PWM_INPUT,PWM_IN_MIN,PWM_IN_MAX,PWM_min_value,PWM_max_value);
  SET_PWM(motor_speed);
  
  DELAY= motor_speed-PWM_INPUT;
  digitalWrite(buzzer,HIGH);
  delay(1000);
  digitalWrite(buzzer,LOW);
  
 
  }
 
  
  
  
}





/*Bu, PWM girişi olan dijital pin D8 için  pin değişiminde kesinti rutinidir.*/

ISR(PCINT0_vect){
  //İlk önce micros() fonksiyonunu kullanarak mevcut sayım değerini mikro saniye cinsinden alıyoruz.
  current_count = micros();
  
  if(PINB & 0x01){                             //Pin durum register'ı ile bir AND yapıyoruz, Pin 8'in HIGH olup olmadığını doğruluyoruz???
    while(last_PWM_state == 0){                //Son durum 0 ise, durum değişikliği gerçekleşir
      last_PWM_state = 1;                      //Bir sonraki döngü için mevcut durumu son duruma depolamak için
      counter_1 = current_count;               // Counter_1'i geçerli değere ayarlayın.
    }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
  }
  else if(last_PWM_state == 1){                      //Pin 8 LOW ve son durum HIGH ise, durum değişikliğimiz var demektir     
    last_PWM_state = 0;                              //Bir sonraki döngü için mevcut durumu son duruma depola
    PWM_INPUT = current_count - counter_1;           //Zaman farkı oluşturulur. PWM_INPUT, current_time - mikro saniye cinsinden bir counter_1'dir.
  }
}
