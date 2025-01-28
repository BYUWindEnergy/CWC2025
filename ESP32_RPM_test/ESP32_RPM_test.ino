#include <ESP32Encoder.h>

//At least one of these pins needs interrupt capability (pin 2)
#define ENCODER_PIN_1 5
#define ENCODER_PIN_2 6

//Encoder
ESP32Encoder encoder;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);  // Set baud rate to 9600 bit/s

  pinMode(ENCODER_PIN_1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_2, INPUT_PULLUP);
  
//  ESP32Encoder::useInternalWeakPullResistors=UP;
  encoder.attachFullQuad(ENCODER_PIN_1,ENCODER_PIN_2);
  encoder.setCount(0);

}

void loop() {
    int numSamples = 100;
    float sumRPM = 0.0;
    int count = 0;
    
    static long oldPosition = 0;
    static long newPosition = 0;
    static long oldTime = 0;
    static float rpm = 0;
    static long newTime = 0;
    static long dtheta = 0;
    static long dt = 0;
    static long oldoldposition = 0;
  
    for(int i=0; i<numSamples; i++) {
      newPosition = encoder.getCount();
    
      if(newPosition != oldPosition) {
        newTime = micros();
        dtheta = newPosition - oldPosition;
  //      if (abs(dtheta) > 2048/2) {
  //        dtheta = (dtheta > 0) ? dtheta - 2048 : dtheta + 2048;
  //      }
  //      if (dtheta < 0) {
  //        dtheta = dtheta + 500;
  //      }
        dt = newTime - oldTime;
        oldoldposition = oldPosition;
        oldPosition = newPosition;
        oldTime = newTime;
  
        //1000000*60 converts time from microseconds to minutes, 2048 converts dx to rotations
        rpm = (dtheta * 1000000 * 60) / (dt * 500 * 4);
        
        sumRPM += rpm;
      }
      //delay(1);
    }
    
    float averageRPM = sumRPM / numSamples;
    Serial.println(String(abs(averageRPM))+"   "+String(sumRPM)+"   "+String(rpm)+"   "+String(dtheta)+"   "+String(dt));
//    Serial.println(String(dtheta)+"   "+String(oldoldposition)+"   "+String(newPosition));

}
