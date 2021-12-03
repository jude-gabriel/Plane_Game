/* --COPYRIGHT--,BSD
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//****************************************************************************
//
// main.c - MSP-EXP432P401R + Educational Boosterpack MkII - Joystick
//
//          Displays raw 14-bit ADC measurements for X/Y axis of Joystick
//
//****************************************************************************

#include <ti/devices/msp432p4xx/inc/msp.h>
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include <ti/grlib/grlib.h>
#include "LcdDriver/Crystalfontz128x128_ST7735.h"
#include <stdio.h>
#include <math.h>

/* Graphic library context */
Graphics_Context g_sContext;

/* ADC results buffer */
static uint16_t resultsBuffer[2];

/* Location of plane variables */
int planeXLeft;
int planeXRight;
int planeYTop;
int planeYBottom;

/* Instance variable used for drawing the plane, slows down planes movement */
int slowPlane;

/* Delay Variable */
uint32_t delay_time_us;

//generate random number for the coordinates.

/* Asteroid Locations */
int astXLeft;
int astXRight;
int astYTop;
int astYBottom;
int astXLeft1;
int astXRight1;
int astYTop1;
int astYBottom1;
/* Instance variable to slow the asteroid */
int slowAst;
int countAstSpeed;
int astModSpeed;

/* Instance Variable for if the plane makes a collision */
bool didPlaneCollide;

/* Variables for number of lives the player has and the string that displays it */
#define lives_length 50
char numLivesString[lives_length];
int numLives;

/* Variables for the score */
int score;
int scoreAdd;
#define score_length 50
char scoreString[score_length];

/* Variables for laser system */
#define MAX_LASERS 8
int laserX[MAX_LASERS];
int laserY[MAX_LASERS];
int laserSpeed = 2;

int didFire;



/**
 * Erases the plane
 *
 * @author Jude Gabriel
 */
void eraseRect(Graphics_Context g_sContext, int xMin, int yMin, int xMax, int yMax){
    //Change the foreground color to the background color
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

    //Create a rectangle
    Graphics_Rectangle myRectangle = {xMin, yMin, xMax, yMax};
    Graphics_drawRectangle(&g_sContext, &myRectangle);
    Graphics_fillRectangle(&g_sContext, &myRectangle);

    //Set foreground color back to original
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
}


/**
 * Draws the plane
 *
 * @author Jude Gabriel
 */
void drawRect(Graphics_Context g_sContext, int xMin, int yMin, int xMax, int yMax){
   //Change the foreground color to initial foreground color
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);

    //Create and draw a rectangle
    Graphics_Rectangle myRectangle = {xMin, yMin, xMax, yMax};
    Graphics_drawRectangle(&g_sContext, &myRectangle);
    Graphics_fillRectangle(&g_sContext, &myRectangle);
}

/**
 * TIMER/DELAY initialization
 *
 * Initializes the timer AND delay
 *
 * @author Jude Gabriel
 */
void timer_delay_init(void){
    //Initialize the timer
    Timer32_initModule(TIMER32_0_BASE, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_PERIODIC_MODE);
    Timer32_disableInterrupt(TIMER32_0_BASE);
}


/**
 * delay
 *
 * Allows the user to delay the specified amount of time
 *
 * @author Jude Gabriel
 */
void delay(uint32_t duration_us){
    //Set the timer
    Timer32_haltTimer(TIMER32_0_BASE);
    Timer32_setCount(TIMER32_0_BASE, 3 * duration_us);
    Timer32_startTimer(TIMER32_0_BASE, true);

    //Have the timer count down
    while(Timer32_getValue(TIMER32_0_BASE) > 0);
}


/**
 * Draws a string
 */
void drawString(Graphics_Context g_sContext, int8_t* theString, int i, int j){
    //Change foreground color to the color of the text we want
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);

    //Draw the string at the specified location
    Graphics_drawString(&g_sContext, theString, -1, i, j, true);
}


/**
 * Erases a string
 */
void eraseString(Graphics_Context g_sContext, int8_t* theString, int i, int j){
    //Change the foreground color to the background color to cover the text
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);

    //Draw the cover-up string
    Graphics_drawString(&g_sContext, theString, -1, i, j, true);

    //Set the foreground color back to yellow
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_YELLOW);
}



/*
 * Main function
 */
int main(void)
{
    /* Halting WDT and disabling master interrupts */
    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

    /* Set the core voltage level to VCORE1 */
    MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

    /* Set 2 flash wait states for Flash bank 0 and 1*/
    MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
    MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

    /* Initializes Clock System */
    MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
    MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
    MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Initializes display */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128, &g_sCrystalfontz128x128_funcs);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
    Graphics_clearDisplay(&g_sContext);


    /* Configures Pin 6.0 and 4.4 as ADC input */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN0, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Initializing ADC (ADCOSC/64/8) */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8, 0);

    /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM1 (A15, A9)  with repeat)
         * with internal 2.5v reference */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM1, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A15, ADC_NONDIFFERENTIAL_INPUTS);

    MAP_ADC14_configureConversionMemory(ADC_MEM1,
            ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A9, ADC_NONDIFFERENTIAL_INPUTS);

    /* Enabling the interrupt when a conversion on channel 1 (end of sequence)
     *  is complete and enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT1);

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
     * convert.
     */
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();


    //Set the background to black to simulate space and the foreground to red for our plane
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
    Graphics_clearDisplay(&g_sContext);
    Graphics_setFont(&g_sContext, &g_sFontCm12);

    //Draw the place and give it coordinates (Plane is a square for now)
    planeXLeft = 10;
    planeXRight = 20;
    planeYTop = 55;
    planeYBottom = 65;

    //Draw the initial plane
    drawRect(g_sContext, planeXLeft, planeYTop, planeXRight, planeYBottom);

    //Set variable to slow plane and slow asteroid
    slowPlane = 0;
    slowAst = 0;
    countAstSpeed = 0;
    astModSpeed = 40;

    //Initialize the timer and delay
    timer_delay_init();

    //Create the delay time
    delay_time_us = 20000;

    //Draw a random asteroid
    /*
    astXLeft = 122;
    astXRight = 127;
    astYTop = 50;
    astYBottom = 60;
    */
    int a = (rand() % (107 - 20 + 1)) + 20;//first asteroid
    int b = (rand() % (107 - 20 + 1)) + 20;//second

    /* Asteroid Locations */
    astXLeft = 122;
    astXRight = 127;
    astYTop = a;
    astYBottom = a + 10;
    drawRect(g_sContext, astXLeft, astYTop, astXRight, astYBottom);
    //second asteroid
    astXLeft1 = 122;
    astXRight1 = 127;
    astYTop1 = b;
    astYBottom1 = b + 10;
    drawRect(g_sContext, astXLeft1, astYTop1, astXRight1, astYBottom1);

    //Initialize didPlaneCollide
    didPlaneCollide = false;

    //Initialize lasers
    int i = 0;
    for (i = 0; i < 8; i++) {
        laserX[i] = -1;
        laserY[i] = -1;
    }

    didFire = 0;

    //Initialize the number of lives
     numLives = 3;
     snprintf(numLivesString, lives_length, "Lives Left: %d", numLives);
     drawString(g_sContext, (int8_t*) numLivesString, 50, 5);

     //Initialize the score
     score = 0;
     scoreAdd = 0;
     snprintf(scoreString, score_length, "Score: %d", score);
     drawString(g_sContext, (int8_t*) scoreString, 50, 115);

    while(1)
    {
        //Exit the while loop if there was a collision
        if(didPlaneCollide == true)
        {
            Graphics_clearDisplay(&g_sContext);
            break;
        }
        MAP_PCM_gotoLPM0();

    }

    //If we hit here there was a collision. Clear all screen and display game over
    eraseRect(g_sContext, planeXLeft, planeYTop, planeXRight, planeYBottom);
    eraseRect(g_sContext, astXLeft, astYTop, astXRight, astYBottom);
    eraseRect(g_sContext, astXLeft1, astYTop1, astXRight1, astYBottom1);
    Graphics_clearDisplay(&g_sContext);
    drawString(g_sContext, (int8_t*) "GAME OVER.", 5, 50);
    return 1;

}


/* This interrupt is fired whenever a conversion is completed and placed in
 * ADC_MEM1. This signals the end of conversion and the results array is
 * grabbed and placed in resultsBuffer */
void ADC14_IRQHandler(void)
{
    uint64_t status;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);


    if(didPlaneCollide == false){
        /* ADC_MEM1 conversion completed */
        if(status & ADC_INT1)
        {
            /* Store ADC14 conversion results */
            resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
            resultsBuffer[1] = ADC14_getResult(ADC_MEM1);


            /********* Updat the Plane's coordinates *************/

            //Case 1: Joystick is being moved up, move plane up
            if(resultsBuffer[1] > 9500)
            {
                //Update slowPlane
                slowPlane++;

                if(slowPlane % 10 == 0)
                {
                    //Erase the most current plane
                    eraseRect(g_sContext, planeXLeft, planeYTop, planeXRight, planeYBottom);

                    //Update the coordinates
                    planeYTop--;
                    planeYBottom--;

                    //Error check the coordinates
                    if(planeYTop < 20){
                        planeYTop++;
                        planeYBottom++;
                    }

                    //Draw the plane in the new coordinates
                    drawRect(g_sContext, planeXLeft, planeYTop, planeXRight, planeYBottom);

                    //update slowPlane
                    slowPlane = 0;
                }
            }



            //Case 2: Joystick is being moved down, move plane down
            if(resultsBuffer[1] < 7500)
            {
                //Update slowPlane
                slowPlane++;

                if(slowPlane % 10 == 0)
                {
                    //Erase the most current plane
                    eraseRect(g_sContext, planeXLeft, planeYTop, planeXRight, planeYBottom);

                    //Update the coordinates
                    planeYTop++;
                    planeYBottom++;

                    //Error check the coordinates
                    if(planeYBottom  > 117){
                        planeYTop--;
                        planeYBottom--;
                    }

                    //Draw the plane in the new coordinates
                    drawRect(g_sContext, planeXLeft, planeYTop, planeXRight, planeYBottom);

                    //Update slowPlane
                    slowPlane = 0;
                }
            }

        }


       //Increase the slow down variable
       slowAst++;
       countAstSpeed++;

       //Check if mod value is true
       if(slowAst % astModSpeed == 0)
       {
           //Case 1: We are still on screen, move asteroid left
           if(astXLeft > 0 || astXLeft1>0)
           {
               eraseRect(g_sContext, astXLeft, astYTop, astXRight, astYBottom);
               eraseRect(g_sContext, astXLeft1, astYTop1, astXRight1, astYBottom1);
               delay(delay_time_us);
               astXLeft--;
               astXRight--;
               astXLeft1--;
               astXRight1--;
               drawRect(g_sContext, astXLeft, astYTop, astXRight, astYBottom);
               drawRect(g_sContext, astXLeft1, astYTop1, astXRight1, astYBottom1);
               slowAst = 0;
               if(countAstSpeed % 200 == 0){
                   astModSpeed--;
                   if(astModSpeed <= 5){
                       astModSpeed = 5;
                   }
               }
           }

           //Case 2: Asteroid is off screen. Erase it
           if(astXLeft == 0){
               eraseRect(g_sContext, astXLeft, astYTop, astXRight, astYBottom);
           }
           if(astXLeft1==0){
               eraseRect(g_sContext, astXLeft1, astYTop1, astXRight1, astYBottom1);
           }
       }

       //Once asteroid goes off screen, reset it's values
       if(astXLeft == 0){
           int a = (rand() % (107 - 20 + 1)) + 20;  //y

               /* Asteroid Locations */
               astXLeft = 122;
               astXRight = 127;
               astYTop = a;
               astYBottom =a + 10;
       }
       if(astXLeft1 == 0){
                  int b = (rand() % (107 - 20 + 1)) + 20;  //y

                      /* Asteroid Locations */
                      astXLeft1 = 122;
                      astXRight1 = 127;
                      astYTop1 = b;
                      astYBottom1 =b + 10;
              }

       //Check for a collision
       if((astXLeft >= planeXLeft && astXLeft <= planeXRight) && ((astYTop >= planeYTop && astYTop <= planeYBottom) || (astYBottom >= planeYTop && astYBottom <= planeYBottom)))
       {
           //Take the object off of the screen
           eraseRect(g_sContext, astXLeft, astYTop, astXRight, astYBottom);
           astXLeft = 0;
           astXRight = 0;

           //Update the number of lives
          numLives--;

          //Redraw the string
          eraseString(g_sContext, (int8_t*) numLivesString, 50, 5);
          snprintf(numLivesString, lives_length, "Lives Left: %d", numLives);
          drawString(g_sContext, (int8_t*) numLivesString, 50, 5);

          //Check if out of lives
          if(numLives == 0)
          {
              //If we are out of lives set to true so the game ends
              didPlaneCollide = true;
          }

       }

       /* Laser code */
       //Check if the button is pushed
       if(~P5IN & 0x02)
       {
           didFire = 1;
           // Check for available laser
           int i = 0;
           int laserId = -1;
           for (i = 0; i < MAX_LASERS; i ++) {
               if(laserX[i] == -1){
                   laserId = i;
                   break;
               }
           }

           // Spawn the laser
           if (laserId != -1) {
               laserX[laserId] = planeXRight + 6;
               laserY[laserId] = (planeYTop + planeYBottom) / 2;
           }


       }

       // Laser tick update
       if(didFire == 1){
           int i = 0;
           for (i = 0; i < MAX_LASERS; i ++) {
               // Move active lasers
               if (laserX[i] != -1) {
                   laserX[i] += laserSpeed;
               }
               // Delete lasers that go past the border
               if (laserX[i] > 127) {
                   eraseRect(g_sContext, laserX[i]-5-laserSpeed, laserY[i]-1, laserX[i]+5 - laserSpeed, laserY[i]+1);
                   laserX[i] = -1;
               }
               // Draw laser
               eraseRect(g_sContext, laserX[i]-5-laserSpeed, laserY[i]-1, laserX[i]+5 - laserSpeed, laserY[i]+1);
               drawRect(g_sContext, laserX[i]-5, laserY[i]-1, laserX[i], laserY[i]+1);
           }

           // Asteroid collision
           i = 0;
           for (i = 0; i < MAX_LASERS; i ++) {
               if (laserX[i] != -1) {
                   if (laserX[i] >= astXLeft && laserX[i] <= astXRight && laserY[i] >= astYTop && laserY[i] <= astYBottom){
                       eraseRect(g_sContext, astXLeft, astYTop, astXRight, astYBottom);
                       astXLeft = 0;
                       astXRight = 0;
                       astYTop = 0;
                       astYBottom = 0;
                       score++;
                       snprintf(scoreString, score_length, "Score: %d", score);
                       drawString(g_sContext, (int8_t*) scoreString, 50, 115);
                   }
               }
           }
       }
    }


   if(didPlaneCollide == true){
       return;
   }







}
