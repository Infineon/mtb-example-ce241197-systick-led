/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Systic timer
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
* Macros
********************************************************************************/
/* 40 kHz ILO clock with 40000 reload value to generate 1s interrupt */
#define SYSTICK_RELOAD_VAL   (40000UL)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void systick_handler(void);

/*******************************************************************************
* Global Variables
********************************************************************************/

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This main achieve the systick timer interrupt function. Toggle user led when generate
* the systick interrupt with 1s period.
*
* Return: int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the systick, set the 40 kHz ILO as clock source */
    Cy_SysTick_Init(CY_SYSTICK_CLOCK_SOURCE_CLK_LF, SYSTICK_RELOAD_VAL);

    /* Set Systick interrupt callback and enable Systick interrupt*/
    Cy_SysTick_SetCallback(0, systick_handler);

    /* Enable Systick */
    Cy_SysTick_Enable();

    for (;;)
    {
    }
}

/*******************************************************************************
* Function Name: toggle_led_on_systick_handler
********************************************************************************
*
*  Summary:
*  Systick interrupt handler
*
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
void systick_handler(void)
{
    /* toggle led */
    Cy_GPIO_Inv(CYBSP_USER_LED_PORT,CYBSP_USER_LED_NUM);

}

/* [] END OF FILE */
