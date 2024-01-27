#include <AccelStepper.h>
#include <MultiStepper.h>

#define DIR_1 3
#define DIR_2 4
#define DIR_3 5
#define STEP_1 6
#define STEP_2 7
#define STEP_3 8
#define EN 9
#define arduinoLED 13   // Arduino LED on board

AccelStepper stepper_1(1,STEP_1,DIR_1);
AccelStepper stepper_2(1,STEP_2,DIR_2);
AccelStepper stepper_3(1,STEP_3,DIR_3);

String commands[]={"M A1033.5601 B627.3443 C1439.8159",
"M A1860.9124 B1272.9105 C1566.9136",
"M A2506.4515 B2100.2554 C1694.0331",
"M A2633.5601 B2927.6151 C2339.5646",
"M A2760.6687 B3573.1271 C3166.9005",
"M A3406.2078 B3700.2302 C3994.2692",
"M A4233.5601 B3827.3443 C4639.8159"};

/*String commands[]={"M A1033.5601 B627.3443 C1439.8159","M A1158.8081 B673.8966 C1473.1277","M A1286.5533 B741.4963 C1498.4017",
"M A1419.18 B838.4488 C1518.7866",
"M A1558.7132 B966.395 C1536.2144",
"M A1706.2603 B1115.8295 C1551.9502",
"M A1860.9124 B1272.9105 C1566.9136",
"M A2017.993 B1427.5613 C1581.8775",
"M A2167.4242 B1575.1067 C1597.6147",
"M A2295.364 B1714.638 C1615.0452",
"M A2392.3093 B1847.2634 C1635.434",
"M A2459.903 B1975.0077 C1660.7138",
"M A2506.4515 B2100.2554 C1694.0331",
"M A2539.767 B2225.5037 C1740.5862",
"M A2565.044 B2353.2498 C1808.1841",
"M A2585.4308 B2485.8779 C1905.1305",
"M A2602.86 B2625.4128 C2033.0666",
"M A2618.5964 B2772.9618 C2182.491",,
"M A2633.5601 B2927.6151 C2339.5646",
"M A2648.5238 B3084.6954 C2494.2109",
"M A2664.2603 B3234.1233 C2641.7537",
"M A2681.6894 B3362.0567 C2781.2839",
"M A2702.0763 B3458.9946 C2913.9087",
"M A2727.3532 B3526.5824 C3041.6528",
"M A2760.6687 B3573.1271 C3166.9005",
"M A2807.2172 B3606.4403 C3292.1488",
"M A2874.811 B3631.7158 C3419.8951",
"M A2971.7562 B3652.1018 C3552.5237",
"M A3099.696 B3669.5304 C3692.0598",
"M A3249.1272 B3685.2666 C3839.6113",
"M A3406.2078 B3700.2302 C3994.2692",
"M A3560.8599 B3715.1939 C4151.3569",
"M A3708.4071 B3730.9307 C4300.7949",
"M A3847.9402 B3748.3603 C4428.7383",
"M A3980.5669 B3768.7481 C4525.6824",
"M A4108.3121 B3794.0264 C4593.272",
"M A4233.5601 B3827.3443 C4639.8159"};*/

/*long steps[3][7]={
  {1033.5601,	1860.9124,	2506.4515,	2633.5601,	2760.6687,	3406.2078,	4233.5601},
  {627.3442,	1272.9105,	2100.2554,	2927.6151,	3573.1271,	3700.2301,	3827.3442},
  {1439.8159,	1566.9135,	1694.0331,	2339.5645,	3166.9005, 3994.2692,	4639.8159}};*/

long steps[3][37]={
  {1033.56010000000,	1158.80810000000,	1286.55330000000,	1419.18000000000,	1558.71320000000,	1706.26030000000,	1860.91240000000,	2017.99300000000,	2167.42420000000,	2295.36400000000,	2392.30930000000,	2459.90300000000,	2506.45150000000,	2539.76700000000,	2565.04400000000,	2585.43080000000,	2602.86000000000,	2618.59640000000,	2633.56010000000,	2648.52380000000,	2664.26030000000,	2681.68940000000,	2702.07630000000,	2727.35320000000,	2760.66870000000,	2807.21720000000,	2874.81100000000,	2971.75620000000,	3099.69600000000,	3249.12720000000,	3406.20780000000,	3560.85990000000,	3708.40710000000,	3847.94020000000,	3980.56690000000,	4108.31210000000,	4233.56010000000},
  {627.344300000000,	673.896600000000,	741.496300000000,	838.448800000000,	966.395000000000,	1115.82950000000,	1272.91050000000,	1427.56130000000,	1575.10670000000,	1714.63800000000,	1847.26340000000,	1975.00770000000,	2100.25540000000,	2225.50370000000,	2353.24980000000,	2485.87790000000,	2625.41280000000,	2772.96180000000,	2927.61510000000,	3084.69540000000,	3234.12330000000,	3362.05670000000,	3458.99460000000,	3526.58240000000,	3573.12710000000,	3606.44030000000,	3631.71580000000,	3652.10180000000,	3669.53040000000,	3685.26660000000,	3700.23020000000,	3715.19390000000,	3730.93070000000,	3748.36030000000,	3768.74810000000,	3794.02640000000,	3827.34430000000},
  {1439.81590000000,	1473.12770000000,	1498.40170000000,	1518.78660000000,	1536.21440000000,	1551.95020000000,	1566.91360000000,	1581.87750000000,	1597.61470000000,	1615.04520000000,	1635.43400000000,	1660.71380000000,	1694.03310000000,	1740.58620000000,	1808.18410000000,	1905.13050000000,	2033.06660000000,	2182.49100000000,	2339.56460000000,	2494.21090000000,	2641.75370000000,	2781.28390000000,	2913.90870000000,	3041.65280000000,	3166.90050000000,	3292.14880000000,	3419.89510000000,	3552.52370000000,	3692.05980000000,	3839.61130000000,	3994.26920000000,	4151.35690000000,	4300.79490000000,	4428.73830000000,	4525.68240000000,	4593.27200000000,	4639.81590000000}
};

void setup() {
  pinMode(arduinoLED, OUTPUT);      // Configure the onboard LED for output
  digitalWrite(arduinoLED, LOW);    // default to LED off

  Serial.begin(115200);

  stepper_1.setMaxSpeed(6000); //Set maximum speed for the motor 1 (steps/s)
  stepper_1.setAcceleration(6000); //Set acceleration for the motor 1 (steps/s^2)
  stepper_1.setCurrentPosition(0);

  stepper_2.setMaxSpeed(6000); //Set maximum speed for the motor 1 (steps/s)
  stepper_2.setAcceleration(6000); //Set acceleration for the motor 1 (steps/s^2)
  stepper_2.setCurrentPosition(0);

  stepper_3.setMaxSpeed(6000); //Set maximum speed for the motor 3 (steps/s)
  stepper_3.setAcceleration(6000); //Set acceleration for the motor 3 (steps/s^2)
  stepper_3.setCurrentPosition(0);
  pinMode(EN,OUTPUT);
  digitalWrite(EN,LOW);

  // for (int i=0;i<=36;i++) {
  //   //Serial.println(commands[i]);
  //   //MoveToPosition(commands[i]);
  //   MoveMotors(steps[0][i],steps[1][i],steps[2][i]);    
  //   /*Serial.print("theta1:");
  //   Serial.print(steps[0][i]);
  //   Serial.print(" theta2:");
  //   Serial.print(steps[1][i]);
  //   Serial.print(" theta3:");
  //   Serial.println(steps[2][i]);*/
  //   delay(5);

  // }
  /*Serial.println(sizeof(commands)/sizeof(commands[0]));
  MoveToPosition(commands[1]);*/


}

void loop() {
  //sCmd.readSerial();     // We don't do much, just process serial commands
  while (Serial.available()>0) {
    String command = Serial.readStringUntil('\n');
    MoveToPosition(command);
    MoveTrajectory(command);
    //Serial.println(command);
  }  
}


void MoveToPosition(String command){
  String STEP1="";
  String STEP2="";
  String STEP3="";
  long steps1=0;
  long steps2=0;
  long steps3=0;

  if (command.startsWith("M")){
    //Serial.print("Move to ");
    steps1=ExtractValue(command,'A');
    steps2=ExtractValue(command,'B');
    steps3=ExtractValue(command,'C');
    MoveMotors(steps1,steps2,steps3);
    /*Serial.print("theta1:");
    Serial.print(steps1);
    Serial.print(" theta2:");
    Serial.print(steps2);
    Serial.print(" theta3:");
    Serial.println(steps3);*/
  }
 
}

void MoveTrajectory(String command){
  if (command.startsWith("I")){
    int commaIndex = command.indexOf(',');
    while (commaIndex != -1) {
      // Extract and print the substring
      String substring = command.substring(0, commaIndex);
      //Serial.print("Substring: ");
      //Serial.println(substring);
      MoveToPosition(substring);
      delay(5);

      // Remove the processed substring and the comma from the original string
      command = command.substring(commaIndex + 1);

      // Find the next comma
      commaIndex = command.indexOf(',');
    }
    //Serial.print("Last Substring: ");
    //Serial.println(command);
  }    
}

void MoveMotors(float steps_1,float steps_2,float steps_3){
  stepper_1.moveTo(steps_1);
  stepper_2.moveTo(steps_2);
  stepper_3.moveTo(steps_3);
  while (stepper_1.currentPosition()!=steps_1 || stepper_2.currentPosition()!=steps_2 || stepper_3.currentPosition()!=steps_3) {
    stepper_1.run();
    stepper_2.run();
    stepper_3.run();
  }

}

float ExtractValue(String linea, char eje) {
  int index = linea.indexOf(eje);
  if (index == -1) {
    return 0;  // Retorna 0 si no se encuentra el eje en el comando
  }
  return linea.substring(index + 1).toFloat();  // Obtén el valor del eje como un número flotante
}


void TurnLED(String linea) {  
  // Ejemplo: si el comando es "G1 X100 Y200":
  if (linea.startsWith("ON")) {
    Serial.println(linea);
    digitalWrite(arduinoLED, HIGH);
  }
  if (linea.startsWith("OFF")) {
    Serial.println(linea);
    digitalWrite(arduinoLED, LOW);
  }

}
