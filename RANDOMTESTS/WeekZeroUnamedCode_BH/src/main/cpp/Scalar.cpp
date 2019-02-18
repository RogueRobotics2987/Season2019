
#include "Scalar.h" 
neScaler::neScaler() {

	// TODO Auto-generated constructor stub



}



neScaler::~neScaler() {

	// TODO Auto-generated destructor stub

}



float neScaler::scaleOutput(float x, float exponent, float slope) {

	float output = 0;

	if (x>0){

		output = slope * (pow(fabs(x), exponent));

		if (output>=1){output = 1;}

	}

	if (x<0){

		output = slope * (-1 * pow(fabs(x), exponent));

		if (output <=-1){output = -1;}

	}

	if ((x<deadZone) && (x>-deadZone)){

		output = 0;

	}

	return output;

}



void neScaler::init(float newExponent, float newSlope, float newDeadZone){

	exponent = newExponent;

	slope = newSlope;

	deadZone = newDeadZone;

}



float neScaler::scaleOutput(float x){

	return scaleOutput(x, exponent, slope);

}