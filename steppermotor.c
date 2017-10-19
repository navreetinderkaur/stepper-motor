/**
  ***********************************************************************************************************
  * File Name          : stepper.c
  * Description        : Working with stepper motor and drivers.
  * Modified on	       : Feb 19, 2017
  ***********************************************************************************************************
*/
//NOTE: AS WE ARE USING MD1 & MD2 TO BE HIGH; WE ARE SETTING THEM TO BE IN 2W1-2 PHASE EXCITATION MODE (FOR MICRO-STEPPING).
//	SO 1 STEP CORRESPONDS TO 1.8/8 DEGREES.
//	THEREOFRE ONE WHOLE ROTATION IS OF 1600 STEPS.

//include files
#include <stdint.h>
#include <stdio.h>
#include "stm32f3xx_hal.h"
#include "common.h"

//Declaring global variables
static DAC_HandleTypeDef hdac;
static TIM_HandleTypeDef htim17;
GPIO_InitTypeDef GPIO_InitStruct; 
//global variables for timer interrrupt
uint32_t delayTimer;
static uint32_t delayflag;
int32_t stepTimer;
static int32_t stepCount;
static volatile uint32_t counter;

// FUNCTION      : stepperInit
// DESCRIPTION   : This function initializes Motor drivers.
// PARAMETERS    : 
//	mode	: checks if the command is passed through in vcp
// RETURNS       : NOTHING
void stepperInit(int mode)
{
	int rc;
	
	//checks if such command exists
	if(mode != CMD_INTERACTIVE) {
    	return;
  	}
	//Turn on clocks to I/O
	__GPIOA_CLK_ENABLE();
	__GPIOD_CLK_ENABLE(); 
	__GPIOE_CLK_ENABLE();
	__GPIOF_CLK_ENABLE();
	__DAC1_CLK_ENABLE();
	__TIM17_CLK_ENABLE();
	
	//Configure GPIO pins for DAC
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStruct.Alternate = 0;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	DAC_ChannelConfTypeDef DacConfig;

	//Initialize DAC
	hdac.Instance=DAC1;
	rc = HAL_DAC_Init(&hdac);
	if(rc != HAL_OK) {
		printf("Unable to initialize DAC, rc=%d\n",rc);
		return;
	}
	DacConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	DacConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	rc = HAL_DAC_ConfigChannel(&hdac, &DacConfig,DAC_CHANNEL_1);
	if(rc != HAL_OK) {
		printf("Unable to configure DAC channel 1, rc=%d\n",rc);
		return;
	}

	//Configure GPIO pins for all inputs to LV8712T (PD11, PD14, PF10, PE6, PE7)
	GPIO_InitStruct.Pin = (GPIO_PIN_11 | GPIO_PIN_14) ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;	
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);	

	GPIO_InitStruct.Pin = (GPIO_PIN_10) ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;	
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);	

	GPIO_InitStruct.Pin = (GPIO_PIN_6 | GPIO_PIN_7) ;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;	
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);	

	//Configure Timer17
	htim17.Instance = TIM17;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Prescaler = 95;
  	htim17.Init.Period = 499;
 	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  	htim17.Init.RepetitionCounter = 0;
  	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		return;
	}

    	//Peripheral interrupt priority setting and enabling
    	HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM17_IRQn, 0, 1);
    	HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);


}

// FUNCTION      : stepperEnable
// DESCRIPTION   : This function enables all the peripherals of the stepper motor driver-LV8712T.
// PARAMETERS    : 
//	mode	: checks if the command is passed through in vcp
// RETURNS       : NOTHING
void stepperEnable(int mode)
{
	int rc;
	uint32_t state;

	//checks if such command exists
	if(mode != CMD_INTERACTIVE) {
    	return;
  	}

	//fetch user inputs
  	rc = fetch_uint32_arg(&state);
  	if(rc) {
    		printf("Missing state value\n");
    		return;
  	}

	//Set the peripherals for LV8712T
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11 , 1);		//PS
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7 , 1);		//RESET
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6 , (!state));	//OE

	//Enable the DAC output 
	__HAL_DAC_ENABLE(&hdac,DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac,DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1,DAC_ALIGN_12B_R,4095);  //DAC
}

// FUNCTION      : stepper
// DESCRIPTION   : This function runs the motor for specified time and step.
// PARAMETERS    : 
//	mode	: checks if the command is passed through in vcp
// RETURNS       : NOTHING
void stepper(int mode)
{
	int rc;
	uint32_t delay;
	int32_t step;

	//checks if such command exists
	if(mode != CMD_INTERACTIVE) {
    	return;
  	}
	
	//fetch user inputs
  	rc = fetch_int32_arg(&step);
  	if(rc) {
    		printf("Missing step value\n");
    		return;
  	}

	//fetch user inputs
  	rc = fetch_uint32_arg(&delay);
  	if(rc) {
    		printf("Missing delay value\n");
    		return;
  	}

	//checks if stepper driver is enabled
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)) {
		return;
	}

	//check if motor runs clockwise or counter-clockwise
	if(step > 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);		//FR----clockwise
	}
	else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);		//FR----counter-clockwise
		step = step * (-1);
	}
	
	//toggle step value for each iteration to generate square wave
	for(int i = 0; i < (2*step); i++) {
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);		//STEP
		HAL_Delay(delay);
	}
	
}

// FUNCTION      : TIM17_IRQHandler
// DESCRIPTION   : This function is an interrupt service routine for timer 17.
// PARAMETERS    : NOTHING
// RETURNS       : NOTHING
void TIM17_IRQHandler(void)
{
	
	//calls interrupt handler
	HAL_TIM_IRQHandler(&htim17);
  
	//toggle step value 
	if(stepCount < stepTimer && delayflag == 0) {
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_10);
		stepCount++;
	}
	delayflag++;
	//reset delay flag
	if(delayflag == delayTimer) {
		delayflag = 0;
	}
}

// FUNCTION      : stepperInit
// DESCRIPTION   : This function initializes Motor drivers.
// PARAMETERS    : 
//	mode	: checks if the command is passed through in vcp
// RETURNS       : NOTHING
void stepperTimer(int mode)
{
	int rc;
	
	//checks if such command exists
	if(mode != CMD_INTERACTIVE) {
    	return;
  	}
	
	//fetch user inputs
  	rc = fetch_int32_arg(&stepTimer);
  	if(rc) {
    		printf("Missing step value\n");
    		return;
  	}

	//fetch user inputs
  	rc = fetch_uint32_arg(&delayTimer);
  	if(rc) {
    		printf("Missing delay value\n");
    		return;
  	}

	//checks if stepper driver is enabled
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)) {
		return;
	}

	//check if motor runs clockwise or counter-clockwise
	if(stepTimer > 0) {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);		//FR----clockwise
	}
	else {
		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);		//FR----counter-clockwise
		stepTimer = stepTimer * (-1);
	}
	//reset flags
	stepCount = 0;
	delayflag = 0;

	//starts timer interrupt
	HAL_TIM_Base_Start_IT(&htim17);

}

// FUNCTION      : stepperTrapeze
// DESCRIPTION   : This function runs the motor in a trapezoidal motion.
// PARAMETERS    : 
//	mode	: checks if the command is passed through in vcp
// RETURNS       : NOTHING
void stepperTrapeze(int mode)
{
	//variables to set acceleration 
	uint32_t counter = 0;
	uint32_t delay = 10;	
	uint32_t flag = 1;

	//checks if such command exists
	if(mode != CMD_INTERACTIVE) {
    	return;
  	}

	//checks if stepper driver is enabled
	if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)) {
		return;
	}

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);				//FR---set to be clockwise
	
	while(counter < 3200) {
		if(counter < 200) {						//ACCELERATING
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, 0);		//STEP
			HAL_Delay(delay);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, 1);		//STEP
			HAL_Delay(delay);
			flag++;
			if(flag % 20 == 0)					//decreases delay value after every 20 iteration
				delay--;
		}	
		else if(counter > 200 && counter < 3000) {			//CONSTANT
			delay = 1;
			flag = 0;
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, 0);		//STEP
			HAL_Delay(delay);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, 1);		//STEP
			HAL_Delay(delay);
		}
		else {								//DEACCELERATING
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, 0);		//STEP
			HAL_Delay(delay);
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, 1);		//STEP
			HAL_Delay(delay);
			flag++;
			if(flag % 20 == 0)					//increases delay value after every 20 iteration
				delay++;
		}
		counter++;		
	}
	
}

//add commands to virtual communication port
ADD_CMD("stepinit",stepperInit,"		Initializes ")	
ADD_CMD("stepenable",stepperEnable," <0/1>		Enables ")	
ADD_CMD("step",stepper," <step> <delay>	Steps ")	
ADD_CMD("steptrapeze",stepperTrapeze," 			Step in trapezoidal manner ")	
ADD_CMD("steptim",stepperTimer," 			Step with timer ")
