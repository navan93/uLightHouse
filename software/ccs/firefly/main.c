/* --COPYRIGHT--,BSD_EX
 * Copyright (c) 2012, Texas Instruments Incorporated
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
 *
 *******************************************************************************
 *
 *                       MSP430 CODE EXAMPLE DISCLAIMER
 *
 * MSP430 code examples are self-contained low-level programs that typically
 * demonstrate a single peripheral function or device feature in a highly
 * concise manner. For this the code may rely on the device's power-on default
 * register values and settings such as the clock configuration and care must
 * be taken when combining code from several examples to avoid potential side
 * effects. Also see www.ti.com/grace for a GUI- and www.ti.com/msp430ware
 * for an API functional library-approach to peripheral configuration.
 *
 * --/COPYRIGHT--*/
//******************************************************************************
//  MSP430G2xx1 Demo - Basic Clock, LPM3 Using WDT ISR, VLO ACLK
//
//  Description: This program operates MSP430 normally in LPM3, pulsing P1.0
//  ~ 6 second intervals. WDT ISR used to wake-up system. All I/O configured
//  as low outputs to eliminate floating inputs. Current consumption does
//  increase when LED is powered on P1.0. Demo for measuring LPM3 current.
//  ACLK = VLO/2, MCLK = SMCLK = default DCO
//
//
//           MSP430G2xx1
//         ---------------
//     /|\|            XIN|-
//      | |               |
//      --|RST        XOUT|-
//        |               |
//        |           P1.0|-->LED
//
//  D. Dang
//  Texas Instruments Inc.
//  October 2010
//  Built with CCS Version 4.2.0 and IAR Embedded Workbench Version: 5.10
//******************************************************************************


#include <msp430.h>
#include <stdint.h>

#define T_ON            12
#define CTR_OFFSET      1
#define NUM_PULSES      20
#define T_ON_DELTA      (1)
#define DARKNESS_TH     400
#define T_ON_TIME       190

static int ton_timer = T_ON_TIME;

void main_ta_10(void)
{
    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
    P1DIR |= 0x02;                            // P1.1 output
    P1SEL |= 0x02;                            // P1.1 option select
    CCTL0 = OUTMOD_3;                         // CCR0 toggle mode
    CCR0 = 50-1;
    CCR1 = 10;
    TACTL = TASSEL_2 + MC_1;                  // SMCLK, upmode

    __bis_SR_register(CPUOFF);                // CPU off
}

void main_ta_19(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
  P1DIR |= BIT2;                            // P1.2 and P1.3 output
  P1OUT = 0;                                // All P1.x reset
  P1SEL |= BIT2;                            // P1.2 and P1.3 TA1/2 options
  CCR0 = CTR_OFFSET + (T_ON-1);                               // PWM Period/2
  CCTL1 = OUTMOD_3;                         // CCR1 set/reset
  CCR1 = CTR_OFFSET;                                // CCR1 PWM duty cycle

  CCTL0 = CCIE;                             // CCR0 interrupt enabled

  TACTL = TASSEL_2 + MC_2 + TACLR;                  // SMCLK, cont mode

  __bis_SR_register(LPM0_bits + GIE);             // Enter LPM0
}

int main_adc10_02(void)
{
  ADC10CTL0 = SREF_1 + ADC10SHT_2 + REFON + ADC10ON + ADC10IE;
  __enable_interrupt();                     // Enable interrupts.
  CCR0 = 30;                              // Delay to allow Ref to settle
  CCTL0 |= CCIE;                          // Compare-mode interrupt.
  TACTL = TASSEL_2 | MC_1;                  // TACLK = SMCLK, Up mode.
  LPM0;                                     // Wait for delay.
  CCTL0 &= ~CCIE;                         // Disable timer Interrupt
  __disable_interrupt();
  ADC10CTL1 = INCH_6;                       // input A6
  ADC10AE0 |= BIT6;                         // PA.1 ADC option select
  P1DIR |= 0x01;                            // Set P1.0 to output direction

  for (;;)
  {
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    __bis_SR_register(CPUOFF + GIE);        // LPM0, ADC10_ISR will force exi
    if (ADC10MEM < 0x88)                    // ADC10MEM = A1 > 0.2V?
      P1OUT &= ~0x01;                       // Clear P1.0 LED off
    else
      P1OUT |= 0x01;                        // Set P1.0 LED on
  }
}

void adc_init(void)
{
      ADC10CTL0 = SREF_1 + ADC10SHT_0 + REFON + ADC10ON + ADC10IE;
      ADC10CTL1 = INCH_6;                       // input A6
      __enable_interrupt();                     // Enable interrupts.
      CCR0 = 30;                              // Delay to allow Ref to settle
      CCTL0 |= CCIE;                          // Compare-mode interrupt.
      CCTL1 = 0;
      TACTL = TASSEL_2 | MC_1;                  // TACLK = SMCLK, Up mode.
      LPM0;                                     // Wait for delay.
      CCTL0 &= ~CCIE;                         // Disable timer Interrupt
      TACTL = MC_0 + TACLR;                       // Halt timer
      __disable_interrupt();
      ADC10AE0 |= BIT6;                         // PA.1 ADC option select
}

void adc_deinit(void)
{
//    ADC10AE0 = 0;
//    ADC10CTL1 = 0;
    ADC10CTL0 &= ~ENC;                        // ADC10 disabled
    ADC10CTL0 = 0;                            // ADC10, Vref disabled completely
}

int adc_read(void)
{
    ADC10CTL0 |= ENC + ADC10SC;             // Sampling and conversion start
    __bis_SR_register(LPM0_bits + GIE);        // LPM0, ADC10_ISR will force exit
    return ADC10MEM;
}

void pwm_init(void)
{
    CCTL1 = OUTMOD_3;                         // CCR0 set/reset mode
    CCTL0 = CCIE;                             // CCR0 interrupt enabled
}

void pulsar(int pulse_width, int num_pulses)
{
    CCR0 = CTR_OFFSET + (pulse_width-1);;              // CCR0 will drive P1.1 low
    CCR1 = CTR_OFFSET;                                 // CCR1 will drive P1.1 high
    do {
        TACTL = TASSEL_2 + MC_2 + TACLR;            // SMCLK, upmode
        __bis_SR_register(LPM0_bits + GIE);         // CPU off
    }while(--num_pulses);
}

void pulsar2(int pulse_width, int num_pulses)
{
    unsigned int glow = 0;
    CCR1 = CTR_OFFSET;                                 // CCR1 will drive P1.1 high
    do {
        if(glow < pulse_width)
            glow += T_ON_DELTA;
        else
            glow = pulse_width;
        CCR0 = CTR_OFFSET + (glow-1);;              // CCR0 will drive P1.1 low
        TACTL = TASSEL_2 + MC_2 + TACLR;            // SMCLK, upmode
        __bis_SR_register(LPM0_bits + GIE);         // CPU off
    }while(--num_pulses);
}

//Turn ON pwm for a specified amount of time measured by WDT
void pulsar3(void)
{
    CCR0 = CTR_OFFSET + (T_ON-1);              // CCR0 will drive P1.1 low
    CCR1 = CTR_OFFSET;                                 // CCR1 will drive P1.1 high
    CCTL1 = OUTMOD_3;                         // CCR0 set/reset mode
    TACTL = TASSEL_2 + MC_1 + TACLR;            // SMCLK, upmode
    __bis_SR_register(LPM0_bits + GIE);         // CPU off
}

uint8_t is_dark(void)
{
    volatile unsigned int adc_val;
    adc_init();
    adc_val = adc_read();
    adc_deinit();
    return (adc_val < DARKNESS_TH) ;
}

void firefly(void)
{
    volatile unsigned int adc_val;
    adc_init();
    adc_val = adc_read();
    adc_deinit();
    if(adc_val < DARKNESS_TH) {
        pwm_init();
        pulsar(T_ON, NUM_PULSES);
    }
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM3
}

void beacon(void)
{
//    WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer
    if(is_dark()) {
        BCSCTL1 ^= DIVA_2;                        // Switch to DIVA_0
        WDTCTL   = WDT_ADLY_1_9;

        ton_timer = T_ON_TIME;
        pwm_init();
        while(ton_timer > 0)
        {

            pulsar(T_ON, NUM_PULSES);
            __bis_SR_register(LPM3_bits + GIE);         // Enter LPM3
        }
        BCSCTL1 ^= DIVA_2;                        // Switch to DIVA_2
    }
    WDTCTL   = WDT_ADLY_250;
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM3
}

void led_debug(void)
{
    pwm_init();
    pulsar(T_ON, NUM_PULSES);
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM3
}

void adc_debug(void)
{
    volatile unsigned int adc_val;
    adc_init();
    adc_val = adc_read();
    adc_deinit();
    pwm_init();
    pulsar(adc_val + 10, 1);
    __bis_SR_register(LPM3_bits + GIE);         // Enter LPM3
}

int my_main(void)
{

  BCSCTL1 |= DIVA_2;                        // ACLK
  BCSCTL3 |= LFXT1S_2;                      // ACLK = VLO
  WDTCTL   = WDT_ADLY_250;                   // Interval timer _16: 100ms, _250:
  IE1     |= WDTIE;                         // Enable WDT interrupt
//  WDTCTL = WDTPW + WDTHOLD;                 // Stop watchdog timer

  P1DIR = 0xFF;                             // All P1.x outputs
  P1OUT = 0;                                // All P1.x reset
  P2DIR = 0xFF;                             // All P2.x outputs
  P2OUT = 0;                                // All P2.x reset

  P1SEL |= 0x0C;                            // P1.1 option select
//  P1OUT |= BIT7;

  while(1)
  {
//      firefly();
//      led_debug();
//      adc_debug();
      beacon();
  }
}

int main(void)
{
//    main_ta_19();
    my_main();
}

// Watchdog Timer interrupt service routine
#pragma vector=WDT_VECTOR
__interrupt void watchdog_timer (void)
{
  __bic_SR_register_on_exit(LPM3_bits);         // Clear LPM3 bits from 0(SR)
//  __bic_SR_register_on_exit(LPM0_bits);       // Clear LPM0 bits from 0(SR)
  if(ton_timer--);
}

// Timer A0 interrupt service routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
    __bic_SR_register_on_exit(LPM0_bits);       // Clear LPM0 bits from 0(SR)
    TACTL = MC_0 + TACLR;                       // Halt timer
}

// ADC10 interrupt service routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}
