
const int motor_a = 9;
const int motor_b = 10;
const int fan_a = 5;
const int fan_b = 6; 
const int heat_relay = 8;
const int button_pos = 11;
const int button_neg = 12;
const int button_key = 4;

my_serial MySerial = my_serial();

int current_state = 0;

void motor_run(int dir)
{
  if(dir == 1)
  {
    digitalWrite(motor_a, HIGH);
    digitalWrite(motor_b, LOW);
  }
  else if(dir == -1)
  {
    digitalWrite(motor_a, LOW);
    digitalWrite(motor_b, HIGH);
  }
  else if(dir == 0)
  {
    digitalWrite(motor_a, LOW);
    digitalWrite(motor_b, LOW);
  }
}

void fan_run(int bin)
{
  if(bin == 1)
  {
    digitalWrite(fan_a, HIGH);
    digitalWrite(fan_b, LOW);
  }
  else if(bin == 0)
  {
    digitalWrite(fan_a, LOW);
    digitalWrite(fan_b, LOW);
  }
}

void heat_ctrl(int bin)
{
  if(bin == 1)
  {
     digitalWrite(heat_relay, LOW);
  }
  else if(bin == 0)
  {
     digitalWrite(heat_relay, HIGH);
  }
}



void setup() 
{
  pinMode(motor_a, OUTPUT);
  pinMode(motor_b, OUTPUT);
  pinMode(fan_a, OUTPUT);
  pinMode(fan_b, OUTPUT);
  pinMode(heat_relay, OUTPUT);
  pinMode(button_pos, INPUT_PULLUP);
  pinMode(button_neg, INPUT_PULLUP);
  pinMode(button_key, INPUT_PULLUP);
  
  digitalWrite(motor_a, LOW);
  digitalWrite(motor_b, LOW);
  digitalWrite(fan_a, LOW);
  digitalWrite(fan_b, LOW);
  digitalWrite(heat_relay, LOW);

  current_state = 0;
}


void loop() 
{
  //Serial.println(current_state);
  current_state = MySerial.rev_data();
  switch(current_state)
  {
    // 按下开关
    case 0:
    {
      motor_run(0);
      fan_run(0);
      heat_ctrl(0);
      /*if(digitalRead(button_key) == 0)
      {
        delay(20);
        if(digitalRead(button_key) == 0)
        {
          current_state = 1; 
        }
      }*/
      break;
    }
   // 电机转到到一半停顿
   case 1:
   {
      motor_run(1);
      delay(5000);
      motor_run(0);
      current_state = 2;
      delay(1000);
      break;
   }
   // 电机继续转动直到碰到开关
   case 2:
   {
      motor_run(1);
      if(digitalRead(button_pos) == 0)
      {
        current_state = 3;
      }
      break;
   }
   // 加热
   case 3:
   {
      motor_run(0);
      //加热
      heat_ctrl(1);
      delay(10000);
      heat_ctrl(0);
      current_state = 4;
      break;
   }
   // 电机回转直到碰到开关
   case 4:
   {
      motor_run(-1);
      if(digitalRead(button_neg) == 0)
      {
        current_state = 5;
      }
      break;
   }
   // 风机抽气
   case 5:
   {
      motor_run(0);
      fan_run(1);
      delay(5000);
      current_state = 0;
      break;
   }
  }
}
