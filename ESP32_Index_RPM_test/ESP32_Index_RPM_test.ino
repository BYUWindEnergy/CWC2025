const int channelA_Pin = 6; // GPIO pin for Channel A
const int index_Pin = 2;    // GPIO pin for Index

volatile unsigned long lastIndexTime = 0;
volatile unsigned long currentIndexTime = 0;
volatile bool newRevolution = false;

void indexISR() {
  lastIndexTime = currentIndexTime;
  currentIndexTime = micros();
  newRevolution = true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(channelA_Pin, INPUT);
  pinMode(index_Pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(index_Pin), indexISR, RISING);
}

void loop() {

  float rpm = 0;
  static float prevRPM = 0;
  
  if (newRevolution) {
    newRevolution = false;
    unsigned long timeInterval = currentIndexTime - lastIndexTime;
    if (timeInterval > 0) {
      rpm = (60.0 * 1000000.0) / timeInterval;
        Serial.println(rpm);
//      if (abs(rpm - prevRPM) > 1000){
//        rpm = prevRPM;
//        }
//        prevRPM = rpm;
    }
  }

}

//  static float sumRPM =0;
//  static float rpm = 0;
//  for (int i=0;i<101;i++){
//    if (newRevolution) {
//      i++;
//      newRevolution = false;
//      unsigned long timeInterval = currentIndexTime - lastIndexTime;
//      if (timeInterval > 0) {
//        rpm = (60.0 * 1000.0) / timeInterval;
//      }
//    }
//    if (i==100) {
//      sumRPM = 0;
//    }
//    sumRPM += rpm;
//  }
//  
//  float averageRPM = sumRPM/100;
//  Serial.print("RPM: ");
//  Serial.println(String(averageRPM)+"   "+String(sumRPM));
