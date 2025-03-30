void led_pin_init(){
  pinMode(IO4_PIN, OUTPUT);
  pinMode(IO5_PIN, OUTPUT);

  ledcAttach(IO4_PIN, FREQ, ANALOG_WRITE_BITS);
  ledcAttach(IO4_PIN, FREQ, ANALOG_WRITE_BITS);
}

void led_pwm_ctrl(int io4Input, int io5Input) {
  ledcWrite(IO4_CH, constrain(io4Input, 0, 255));
  ledcWrite(IO5_CH, constrain(io5Input, 0, 255));
}