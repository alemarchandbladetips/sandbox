long t0,t1;
float x, y, quat[4],rpy[3];
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void quat2rpy(float q[4], float rpy[3])
{
  rpy[0] = atan2(2*q[1]*q[3] + 2*q[2]*q[0], 1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
  rpy[1] = asin(2*q[2]*q[3] - 2*q[1]*q[0]);
  rpy[2] = atan2(2*q[1]*q[2] + 2*q[3]*q[0], 1 - 2*q[1]*q[1] - 2*q[3]*q[3]);
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  for(i=0;i<4;i++)
  {
    quat[i] = (float)random(500)/500.0;
  }
  t0 = micros();
  y = constrain(x,0,100);
  t1 = micros();
  Serial.println(t1-t0);
  x++;
}
