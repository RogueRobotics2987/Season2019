#ifndef SRC_RRCONSTANTS_V001_H_
#define SRC_RRCONSTANTS_V001_H_

	/*
	 *value for Proportional control
	 */

double kPVal = 0.015;
	/*
	 * value for Integral control
	 */

double kIVal = 0.01;

enum Constants {
	/*
	 * Which PID slot to pull gains from.  Starting 2018, you can choose
	 * from 0,1,2 or 3.  Only the first two (0,1) are visible in web-based configuration.
	 */
	kSlotIdx = 0,

	/*
	 *  Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops.
	 */
	kPIDLoopIdx = 0,

	/*
	 * set to zero to skip waiting for confirmation, set to nonzero to wait
	 * and report to DS if action fails.
	 */
	kTimeoutMs = 10,

};

#endif
