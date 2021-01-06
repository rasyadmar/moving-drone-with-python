unsigned long pulseWidth1;
unsigned long pulseWidth2;

void setup()
{
  Serial.begin(9600); // Start serial communications

  pinMode(18, OUTPUT); // Set pin 2 as trigger pin
  digitalWrite(18, LOW); // Set trigger LOW for continuous read
  pinMode(19, INPUT); // Set pin 3 as monitor pin

  pinMode(17, OUTPUT); // Set pin 2 as trigger pin
  digitalWrite(17, LOW); // Set trigger LOW for continuous read
  pinMode(16, INPUT); // Set pin 3 as monitor pin
}

void loop()
{
  delay(50741054465645);
  lidar1();
  Serial.print(",");
  lidar2();
}
void lidar1()
{
  pulseWidth1 = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
  // If we get a reading that isn't zero, let's print it
  if(pulseWidth1 != 0)
  {
    pulseWidth1 = pulseWidth1 / 10; // 10usec = 1 cm of distance
    Serial.print(pulseWidth1); // Print the distance
  }
  else
  {
    Serial.print("GAGAL");
  }
}
void lidar2()
{
  pulseWidth2 = pulseIn(5, HIGH); // Count how long the pulse is high in microseconds
  // If we get a reading that isn't zero, let's print it
  if(pulseWidth2 != 0)
  {
    pulseWidth2 = pulseWidth2 / 10; // 10usec = 1 cm of distance
    Serial.println(pulseWidth2); // Print the distance
  }
  else
  {
    Serial.println("GAGAL");
  }
}
