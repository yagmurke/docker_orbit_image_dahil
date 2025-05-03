const int start_btn = 2;
const int continue_btn = 3;
const int reboot_btn = 4; 
const int R = 9;
const int G = 10;
const int B = 11;

unsigned long previous_millis = 0;
unsigned long current_millis = 0;
const long interval = 600;
int led_state = 0;

String in_string = "";

void setup(){
  Serial.begin(9600);
  pinMode(start_btn, INPUT);
  pinMode(continue_btn, INPUT);
  pinMode(reboot_btn, INPUT);
  pinMode(R,OUTPUT);
  pinMode(G,OUTPUT);
  pinMode(B,OUTPUT);
  analogWrite(R, 255);
  analogWrite(G, 255);
  analogWrite(B, 255);
}

void loop(){
  unsigned long current_millis = millis();
  if (Serial.available() > 0){
    in_string = Serial.readString();
    in_string.trim();
  }
  if (current_millis - previous_millis >= interval){
    previous_millis = current_millis;
    light_led(in_string);
  }
  print_btn(); 
}

void print_btn(){
  Serial.print("{");
  Serial.print("\"emmergency_btn\": ");
  Serial.print(digitalRead(start_btn));
  Serial.print(", ");
  Serial.print("\"start_btn\": ");
  Serial.print(digitalRead(continue_btn));
  Serial.print(", ");
  Serial.print("\"reboot_btn\": ");
  Serial.print(digitalRead(reboot_btn));
  Serial.println("}");
}

void light_led(String led){
  if (led_state == 0){
    led_state = 255;
  } else {
    led_state = 0;
  }
  if (led == "brown"){
    analogWrite(R, 0);
    analogWrite(G, 255);
    analogWrite(B,255);
  }
  else if (led == "green"){
    analogWrite(R, 255);
    analogWrite(G, 0);
    analogWrite(B, 255);
  }
  else if (led == "blue"){
    analogWrite(R, 255);
    analogWrite(G, 255);
    analogWrite(B, 0);
  }
}