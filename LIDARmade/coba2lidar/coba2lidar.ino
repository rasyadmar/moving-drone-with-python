unsigned long pulseWidth1;
unsigned long pulseWidth2;
String lidar1, lidar2;

void setup()
{
  Serial.begin(57600); // Start serial communications

  pinMode(2, OUTPUT); // Set pin 2 as trigger pin
  digitalWrite(2, LOW); // Set trigger LOW for continuous read
  pinMode(3, INPUT); // Set pin 3 as monitor pin

  pinMode(4, OUTPUT); // Set pin 2 as trigger pin
  digitalWrite(4, LOW); // Set trigger LOW for continuous read
  pinMode(5, INPUT); // Set pin 3 as monitor pin
}

void loop()
{
  pulseWidth1 = pulseIn(3, HIGH); // Count how long the pulse is high in microseconds
  // If we get a reading that isn't zero, let's print it
  if(pulseWidth1 != 0)
  {
    pulseWidth1 = pulseWidth1 / 10; // 10usec = 1 cm of distance
    lidar1 = String(pulseWidth1); // Print the distance
  }
  else
  {
    lidar1 = "GAGAL";
  }
  pulseWidth2 = pulseIn(5, HIGH); // Count how long the pulse is high in microseconds
  // If we get a reading that isn't zero, let's print it
  if(pulseWidth2 != 0)
  {
    pulseWidth2 = pulseWidth2 / 10; // 10usec = 1 cm of distance
    lidar2 = String(pulseWidth2); // Print the distance
  }
  else
  {
    lidar2 = "GAGAL";
  }
  Serial.println(lidar1+","+lidar2);
 
}
