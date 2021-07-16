/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



#include "msp430.h"
#include "hal_UCS.h"

//====================================================================
/**
 * Startup routine for  XT2
 *
*/
void XT2_Start(void)
{
  P5SEL |= 0x0C;              // XT2IN/OUT = on
  UCSCTL6 &= ~XT2OFF;         // enalbe XT2 even if not used

  //while (SFRIFG1 & OFIFG) {   // check OFIFG fault flag
  while (UCSCTL7 & (DCOFFG /*+XT1LFOFFG+XT1HFOFFG*/ +XT2OFFG))
  {
      UCSCTL7 &= ~(DCOFFG+XT1LFOFFG /*+XT1HFOFFG*/ +XT2OFFG); // Clear OSC flaut Flags fault flags
      SFRIFG1 &= ~OFIFG;        // Clear OFIFG fault flag
  }

}

//====================================================================
/**
  * Initializes FLL of the UCS
  *
  * \param fsystem  required system frequency (MCLK) in kHz
  * \param ratio       ratio between fsystem and FLLREFCLK
  */
void Init_FLL(const unsigned int fsystem, const unsigned int ratio)
{
  unsigned int d, dco_div_bits;
  //  /\  Prevent variables from being "optimized".

  d = ratio;

  dco_div_bits = FLLD__2;      // Have at least a divider of 2

  UCSCTL0 = 0x000;             // Set DCO to lowest Tap

  UCSCTL2 &= ~(0x3FF);         // Reset FN bits
  UCSCTL2= dco_div_bits | (d - 1);

    UCSCTL1= DCORSEL_5 ;

    UCSCTL4 = (UCSCTL4 & ~(SELM_7 + SELS_7)) | (SELM__DCOCLKDIV + SELS__DCOCLKDIV); // selcet DCODIVCLK

} // End of fll_init()
