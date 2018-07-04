void setup() {
  // put your setup code here, to run once:
DDRB |= (1 << DDB6) | (1 << DDB7);
  DDRD |= (1 << DDD6) | (1 << DDD7);
}

void loop() {
  // put your main code here, to run repeatedly:
PORTB =  (0<<DDB7)|(1<<DDB6);
 PORTD = (1<<DDD4)|(0<<DDD3);
}
