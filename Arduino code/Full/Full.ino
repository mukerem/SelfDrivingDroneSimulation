  #include <math.h>  
  #include <TinyGPS.h>
  #include <LiquidCrystal.h>
  #include <SoftwareSerial.h>
  
  #define in1  9
  #define in2  6
  #define in3  5
  #define in4  3
  #define enb  4
  #define trg  A0
  #define ech  A1
  #define pin  A2
  #define bin  A3
  
  TinyGPS gps;
  SoftwareSerial gsmSerial(A5, A4);
  LiquidCrystal lcd(13, 12, 11, 10, 8, 7);
  
  bool idle;
  int m1, m2, m3, m4, disCM, mtr_spd;
  long duration, batteryLevel, pir_reading;
  float xpos, ypos, zpos;
  float x_current, y_current, z_current;
  int angle, dist, dir;
  bool landed = false, goal=false;

  float flat, flon;
  unsigned long age;
  
  bool PIR_read();
  void GPS_read();
  void send_signal();
  void checkBatteryLevel();
  int get_distanceCM(int , int);
  void rotate_left(int angle2);
  void rotate_right(int angle2);
  
  void setup() {
    
      pinMode(in1, OUTPUT);
      pinMode(in2, OUTPUT);
      pinMode(in3, OUTPUT);
      pinMode(in4, OUTPUT);
      pinMode(enb, OUTPUT);
      pinMode(trg, OUTPUT);
      pinMode(ech, INPUT);
      pinMode(pin, INPUT);
      pinMode(bin, INPUT);

      digitalWrite(enb, LOW);
      lcd.begin(20, 4);
      Serial.begin(9600);
      gsmSerial.begin(9600);
     
      gsmSerial.println("Drone Status: GSM Message");
      //while(1)GPS_read();delay(1000);
      lcd.setCursor(4,0);
      lcd.println("Drone Status");
      Serial.println("Drone Status");
      lcd.setCursor(4,1);
      lcd.println("============");
      Serial.println("============");
      lcd.setCursor(1,2);
      lcd.println("Start");
      Serial.println("Start");
      
      mtr_spd = m1 = m2 = m3 = m4 = 0;
      dir = 90;
      x_current = y_current = z_current = 0;
      zpos = 10;
      
      Serial.println("Enter latitude value:"); 
      xpos = Serial.readString().toInt(); 
      delay(10);
      Serial.println("Enter longitude value:"); 
      ypos = Serial.readString().toInt();
      delay(10);
      
      checkBatteryLevel(); 
      Serial.println("The battery level is "+ String(batteryLevel) + "%.");
      if(batteryLevel < 15){
        Serial.println("Unable to Takeoff due to Critically Low Battery!!!"); 
        gsmSerial.println("Unable to Takeoff due to Critically Low Battery!!!");
        while(batteryLevel < 15)checkBatteryLevel();
      }
      
      takeoff();
      adjust_direction();
      lcd.setCursor(0,2);
      lcd.println("(" + String(xpos) + ", " + String(ypos) + ", " + String(zpos) + ")");
      dist = int(sqrt((x_current - xpos) * (x_current - xpos) + (y_current - ypos) * (y_current - ypos)));
      
  }       

  void loop() {  
      do{
        checkBatteryLevel();
      }while(batteryLevel < 10 && landed);
      landed = false;
      lcd.setCursor(0,3);
      lcd.println("(" + String(x_current) + ", " + String(y_current) + ", " + String(z_current) + ")");

      int echo_in = get_distanceCM(A0, A1); 
      int pin_in = digitalRead(pin); 
      
      dist = int(sqrt((x_current - xpos) * (x_current - xpos) + (y_current - ypos) * (y_current - ypos)));
      checkBatteryLevel();
      
          
      if(batteryLevel < 15 && batteryLevel >= 10){
          Serial.println("Battery Low!!! Initiate Self-Landing");  
          gsmSerial.println("Battery Low!!! Initiate Self-Landing");  
      }

     if(batteryLevel < 10){
          Serial.println("Battery Low!!! Self-Landing Initiated"); 
          gsmSerial.println("Battery Low!!! Self-Landing Initiated"); 
          landing();
          
      }
      else if(pin_in){
        gsmSerial.println("Obstacle present");
        control('3');
        rotate_left(10);// rotate 10 degree to left
        control('5');
        z_current++;
        x_current += cos(dir * M_PI / 180.0);
        y_current += sin(dir * M_PI / 180.0);
        adjust_direction();
      }
       else if(!goal && z_current < zpos){
        control('3');
        z_current++;
      }
      else if(!goal && z_current > zpos){
        control('4');
        z_current--;
      }
      else if(dist > 0){
        control('5');
        x_current += cos(dir * M_PI / 180.0);
        y_current += sin(dir * M_PI / 180.0);
      }
      else if(!landed){
        landing();
        goal = true;
      }     
  }

  void control(char data){
    switch(data){
      case '1': 
        m1 = m4 = 100;
        m2 = m3 = 0;
        Serial.println("<-- Move  ft");
        break;
      case '2': 
        m2 = m3 = 100;
        m1 = m4 = 0;
        Serial.println("--> Move Right");
        break;
      case '3': 
        m1 = m2 = m3 = m4 = 0;
        mtr_spd += 50;
        Serial.println("^-- Move Up");
        break;
      case '4':
        m1 = m2 = m3 = m4 = 0;
        mtr_spd -= 50;
        Serial.println("v-- Move Down");
        break;
      case '5': 
        m1 = m2 = 100;
        m3 = m4 = 0;
        Serial.println(">>> Move Forward");
        break;
      case '6': 
        m3 = m4 = 100;
        m1 = m2 = 0;
        Serial.println("<<<  Move Backward");
        break;
      case '7': 
        m2 = m4 = 0;
        m1 = m3 = 100;
        Serial.print("<-0  Rotate Left ");
        break;
      case '8': 
        m1 = m3 = 0;
        m2 = m4 = 100;
        Serial.print("O->  Rotate Right ");
        break;
      case '9':        
        idle = true;
        mtr_spd -= 50;
        m1 = m2 = m3 = m4 = 0;
        send_signal();
        Serial.println("Initiate Self-Land");        
        break;
      case '0': 
         m1 = m2 = m3 = m4 = 0;
    }
    send_signal();
    m1 = m2 = m3 = m4 = 0;
    if(data != '7' && data != '8') delay(500),send_signal();//to make hover
  }

  
  void adjust_direction(){
    int theta_1 = dir;
    int theta_2 = atan2(ypos - y_current, xpos - x_current) * 180.0 / M_PI;
    int rot_angle_1 = (theta_2 - theta_1 + 360) % 360;
    int rot_angle_2 = (theta_1 - theta_2 + 360) % 360;
    // let assume each function call rotate 10 degrees
    if(rot_angle_1 < rot_angle_2)
      rotate_left(rot_angle_1);
   else
     rotate_right(rot_angle_2);
   /*
   Serial.println(xpos);
   Serial.println(ypos);
   Serial.println(theta_1);
   Serial.println(theta_2);
   Serial.println(rot_angle_1);
   Serial.println(rot_angle_2);
   */
    dir = theta_2;
  }
 
  void rotate_left(int angle2){
    control('7');
    Serial.println(" " + String(angle2) + " degree"); 
    delay(angle2 * 10); // 100 ms for 10 degree
    control('0');//to make hover
    dir = (dir + angle2)%360;
  }
  
  void rotate_right(int angle2){
    control('8');
    Serial.println(" " + String(angle2) + " degree");      
    delay(angle2 * 10); // 100 ms for 10 degree
    control('0');//to make hover
    dir = (dir - angle2 + 360)%360;   
  }
  
  long long latitude_to_meter(float lat1,float lon1,float lat2,float lon2){
    long long R = 6378.137; // Radius of earth in KM
    float dLat = lat2 * M_PI / 180.0 - lat1 * M_PI / 180.0;
    float dLon = lon2 * M_PI / 180.0 - lon1 * M_PI / 180.0;
    float a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * M_PI / 180) * cos(lat2 * M_PI / 180) * sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    long long d = R * c;
    return abs(d * 1000); // meters
  } 
  
  void takeoff(){
    digitalWrite(enb, HIGH);    
    mtr_spd = 0;
    lcd.println("Initiate Self-Takeoff");
    Serial.println("Initiate Self-Takeoff");  
    for(int i=0; i<5; i++){
      mtr_spd += 100;
      send_signal();
      z_current++;
    }
    delay(500);
  }
  
   void landing(){          
    Serial.println("Initiate Self-Land");
    control('0');//hover
    delay(2000);
    int ground_distance = get_distanceCM(A0, A1); 
    while(ground_distance > 5){
      mtr_spd -= 10;
      Serial.println(String(ground_distance) + " meter from ground");
      send_signal();
      delay(500);
      ground_distance = get_distanceCM(A0, A1); 
    }
    digitalWrite(enb, LOW);
    Serial.println("Succefully Landed"); 
    gsmSerial.println("Succefully Landed at ");
    gsmSerial.println("     Latitude:  "+ String(x_current));  
    gsmSerial.println("     Longitude: "+ String(y_current));  
    landed = true;
    z_current = 0;
    mtr_spd = 0;
  }
  
  void send_signal(){  
      analogWrite(in1, mtr_spd - m1);
      analogWrite(in2, mtr_spd - m2);
      analogWrite(in3, mtr_spd - m3);
      analogWrite(in4, mtr_spd - m4);
  }

  int get_distanceCM(int trig, int echo) {
      digitalWrite(trig, LOW);  
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);  
      delayMicroseconds(10);
      digitalWrite(trig, LOW);  
      duration = pulseIn(echo, HIGH);
      return duration * 0.034/2; 
  }
  void checkBatteryLevel(){
      batteryLevel = analogRead(bin)/10;
      if(batteryLevel > 100)
          batteryLevel = 100; 
  }
 
  void GPS_read(){
      bool newData = false;
      unsigned long chars;
      unsigned short sentences, failed;
    
      for (unsigned long start = millis(); millis() - start < 1000;){
          while (Serial.available()){
            char c = Serial.read();
            if (gps.encode(c)) {
              newData = true;Serial.println(c);}
          }
      }   
      if (newData){
          gps.f_get_position(&flat, &flon, &age);
          Serial.print("LAT= ");
          Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
          Serial.print(" LON= ");
          Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      }
      
      gps.stats(&chars, &sentences, &failed);
      Serial.print(" CHARS= ");
      Serial.println(chars);
      if (chars == 0)
        Serial.println("** No characters received from GPS: check wiring **");
        
  }
