/*
 * rrPixy.hpp
 *
 *  Created on: Jan 28, 2017
 *      Author: davidj
 */

#ifndef SRC_RRPIXY_HPP_
#define SRC_RRPIXY_HPP_

#include "WPILib.h"

//Default address of Pixy Camera. You can change the address of the Pixy in Pixymon under setting-> Interface
#define PIXY_I2C_DEFAULT_ADDR           0x54

enum BlockType
{
	NORMAL_BLOCK, //normal color recognition
	CC_BLOCK	  //color-code(chnage in angle) recognition
};

struct Block
{  // print block structure!
  uint16_t signature; //Identification number for your object - you could set it in the pixymon
  uint16_t x; //0 - 320
  uint16_t y; //0 - 200
  uint16_t width;
  uint16_t height;
  uint16_t angle;
};

class rrPixy {
	I2C* i2c_Pixy;
	BlockType blockType;// it is the enum on the top
	bool  skipStart;	//skips to check 0xaa55, which is byte that tells pixy it is start of new frame
	uint16_t blockCount; //How many signatured objects are there?
	uint16_t blockArraySize; //not used in the code
	Block blocks[100]; //array that stores blockCount array

public:
	void init(int addy);
	uint16_t rrPixyGetBlocks(uint16_t maxBlocks);
	Block rrReturnBlock0(void);
	Block rrReturnBlock1(void);
	void printBlock(Block block);
	void i2cReset();
	bool getStart();
	uint8_t getByte();
	uint16_t getWord();

};

// Communication/misc parameters
#define PIXY_INITIAL_ARRAYSIZE      30
#define PIXY_MAXIMUM_ARRAYSIZE      130
#define PIXY_START_WORD             0xaa55 //for regular color recognition
#define PIXY_START_WORD_CC          0xaa56 //for color code - angle rotation recognition
#define PIXY_START_WORDX            0x55aa //regular color another way around
#define PIXY_MAX_SIGNATURE          7
#define PIXY_DEFAULT_ARGVAL         0xffff

// Pixy x-y position values
#define PIXY_MIN_X                  0L	//x: 0~319 pixels, y:0~199 pixels. (0,0) starts at bottom left
#define PIXY_MAX_X                  319L
#define PIXY_MIN_Y                  0L
#define PIXY_MAX_Y                  199L

// RC-servo values - not needed unless you want to use servo to face the goal instead of moving the whole robot
#define PIXY_RCS_MIN_POS            0L
#define PIXY_RCS_MAX_POS            1000L
#define PIXY_RCS_CENTER_POS         ((PIXY_RCS_MAX_POS-PIXY_RCS_MIN_POS)/2)


#endif /* SRC_RRPIXY_HPP_ */
