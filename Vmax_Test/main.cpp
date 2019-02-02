#include <iostream>
#include <math.h>
using namespace std;

double getMax(double distance, double acceleration, double totalTime){
double testPoint = -.5 * sqrt(acceleration);
double testPoint2 = sqrt(acceleration*(totalTime*totalTime)  -4* distance);
double testPoint3 = .5 * acceleration * totalTime;
return testPoint*testPoint2 + testPoint3;
}
double getVofT(double VelocityMax, double accelerationMax, double TimeTotal, double TimeCurrent) {
if(TimeCurrent >= 0 && TimeCurrent < TimeTotal*.25 ){
    return accelerationMax * TimeCurrent;
}
else if(TimeCurrent >= TimeTotal*.25 && TimeCurrent < TimeTotal*.75){
    return VelocityMax;
}
else if(TimeCurrent >= TimeTotal* .75 && TimeCurrent <= TimeTotal){
    return -accelerationMax*TimeCurrent + accelerationMax*TimeTotal;

}
else if (TimeCurrent > TimeTotal){
    return 0;
}

}

int main()
{

    double distance = 0;
    double acceleration = 0;
    double time = 0;
    double evaluationTime1 = 0;
    double evaluationTime2 = 0;
    double evaluationTime3 = 0;
cout << "Enter a distance" << endl;
cin >> distance;
cout << "Enter an acceleration" << endl;
cin >> acceleration;
cout << "Enter a time" << endl;
cin >> time;
cout << "VelocityMax = " << getMax(distance, acceleration, time) << endl;
cout << " Enter Evaluation times" << endl;
cin >> evaluationTime1;
cin >> evaluationTime2;
cin >> evaluationTime3;
cout << "V(t) at time "<< evaluationTime1 << "seconds = " << getVofT(getMax(distance, acceleration, time), acceleration, time, evaluationTime1) << endl;
cout << "V(t) at time " << evaluationTime2 << "seconds = "<< getVofT(getMax(distance, acceleration, time), acceleration, time, evaluationTime2) << endl;
cout << "V(t) at time " << evaluationTime3 << "seconds = " << getVofT(getMax(distance, acceleration, time), acceleration, time, evaluationTime3) << endl;

for(double curTime = 0; curTime<20; curTime = curTime+.1){
    cout << "V of T at time" << curTime << " = " << getVofT(getMax(distance, acceleration, time), acceleration, time, curTime) << endl;
}

}
