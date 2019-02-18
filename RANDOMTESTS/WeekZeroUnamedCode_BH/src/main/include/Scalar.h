#ifndef SRC_NESCALER_H_

#define SRC_NESCALER_H_
#include <math.h> 


class neScaler {

public:

	float exponent = 1;

	float slope = 1;

	float deadZone = 0.1;



	neScaler();

	virtual ~neScaler();

	float scaleOutput(float x, float exponent, float slope);

	void init(float newExponent, float newSlope, float newDeadZone);

	float scaleOutput(float x);



};



#endif /* SRC_NESCALER_H_ */