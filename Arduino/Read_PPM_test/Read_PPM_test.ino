#define channels 4 
int channel[channels]; //store Channel values
int PPMin = 2;
 
void setup()
{
  Serial.begin(9600); //Start serial
  pinMode(PPMin, INPUT); // Pin 4 as input
}
 
void loop()
{
  //waits ultil synchronize arrives &gt; 4 miliseconds
  if(pulseIn(PPMin , HIGH) > 4000) //If pulse > 4 miliseconds, continues
  {
    for(int i = 1; i <= channels; i++) //Read the pulses of the remaining channels
    {
 channel[i-1]=pulseIn(PPMin, HIGH);
    }
    for(int i = 1; i <= channels; i++) //Prints all the values read
    {
     Serial.print("CH"); //Channel
     Serial.print(i); //Channel number
     Serial.print(": "); 
     Serial.println(channel[i-1]); //Print the value
    }
    delay(10);//Give time to print values.
  }
}
