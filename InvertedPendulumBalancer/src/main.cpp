#include <Arduino.h>

//-----------Inverted Pendulum Stabilizer----------//

//--hardware--//

  //const
double pendulumLength;
double axisLength;
  //changing physical values
double pendulumAngle();

//--PID Controller--//

  //need to be tuned
double kP;
double kI;
double kD;
double kFF; //feedforward friction
double kG; //feedforward gravity
  //non tunable values
double error;
double lastError;
double integralSum;
double derivative;

//const for time
long initTime = 0;
long finalTime = 0;

//return total time for loop to run
long dT(){
  return finalTime - initTime;
}

double gravityFF(){
  return pendulumLength * sin(pendulumAngle());
}

//proof of consept PID controller
double pidController(double target, double actual){

  error = target - actual;

  integralSum += error * dT();
  derivative = error - lastError * dT();

  lastError = error;

  return (kP * error) + (integralSum * kI)  + (derivative * kD) + kFF + gravityFF();
}


void setup() {
  // put your setup code here, to run once

}

void loop() {
  // put your main code here, to run repeatedly

  //put at start of loop
  initTime = millis();


  pidController(2,2);


  //put at end of loop
  finalTime = millis();

}


