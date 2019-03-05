#include <iostream>

using namespace std;
double prevTime = 0;
double getPos(double speed, double myTime, double max, double min, double curPos){
double curTime = myTime;
    //double dt = curTime - prevTime;
    double dt = .02;
    double targetPos = curPos + speed*dt;
    if(targetPos > max){
        targetPos = max;
    }
    else if(targetPos < min){
        targetPos = min;
    }
   // oldPos = targetPos;
    prevTime = curTime;
    return targetPos; }

int main()
{
    cout << getPos(1, 2, 80, 2, 40)<< endl;
    return 0;
}
