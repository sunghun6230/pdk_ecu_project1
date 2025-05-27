/****************************************************************************
*
* Copyright © 2015-2019 STMicroelectronics - All Rights Reserved
*
* This software is licensed under SLA0098 terms that can be found in the
* DM00779817_1_0.pdf file in the licenses directory of this software product.
* 
* THIS SOFTWARE IS DISTRIBUTED "AS IS," AND ALL WARRANTIES ARE DISCLAIMED,
* INCLUDING MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
*****************************************************************************/
/**
 * @file    pit_lld.c
 * @brief   SPC5xx PIT low level driver code.
 *
 * @addtogroup PIT
 * @{
 */

#include "pit_lld.h"

#if (LLD_USE_PIT == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   PIT0 driver identifier.
 */
#if (SPC5_PIT_USE_PIT0 == TRUE) || defined(__DOXYGEN__)
PITDriver PITD1;
#endif

/**
 * @brief   PIT1 driver identifier.
 */
#if (SPC5_PIT_USE_PIT1 == TRUE) || defined(__DOXYGEN__)
PITDriver PITD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
	
/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/
#if (SPC5_PIT_USE_PIT0 == TRUE) || defined(__DOXYGEN__)
#if (PIT0_CHANNEL_CH0_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 0 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT0_CH0_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD1.pit_ch_config[0].callback != NULL){
    PITD1.pit_ch_config[0].callback();
  }
  
  PITD1.pit_tagp->CH[0].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT0_CHANNEL_CH1_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 1 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT0_CH1_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD1.pit_ch_config[1].callback != NULL){
    PITD1.pit_ch_config[1].callback();
  }

  PITD1.pit_tagp->CH[1].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT0_CHANNEL_CH2_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 2 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT0_CH2_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD1.pit_ch_config[2].callback != NULL){
    PITD1.pit_ch_config[2].callback();
  }
  
  PITD1.pit_tagp->CH[2].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT0_CHANNEL_CH3_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 3 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT0_CH3_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD1.pit_ch_config[3].callback != NULL){
    PITD1.pit_ch_config[3].callback();
  }
  
  PITD1.pit_tagp->CH[3].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT0_CHANNEL_CH4_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 4 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT0_CH4_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD1.pit_ch_config[4].callback != NULL){
    PITD1.pit_ch_config[4].callback();
  }
  
  PITD1.pit_tagp->CH[4].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT0_CHANNEL_CH5_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 5 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT0_CH5_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD1.pit_ch_config[5].callback != NULL){
    PITD1.pit_ch_config[5].callback();
  }
  
  PITD1.pit_tagp->CH[5].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT0_CHANNEL_CH6_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 6 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT0_CH6_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD1.pit_ch_config[6].callback != NULL){
    PITD1.pit_ch_config[6].callback();
  }
  
  PITD1.pit_tagp->CH[6].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT0_CHANNEL_CH7_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 7 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT0_CH7_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD1.pit_ch_config[7].callback != NULL){
    PITD1.pit_ch_config[7].callback();
  }
  
  PITD1.pit_tagp->CH[7].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif
#endif

#if (SPC5_PIT_USE_PIT1 == TRUE) || defined(__DOXYGEN__)
#if (PIT1_CHANNEL_CH0_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 0 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT1_CH0_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD2.pit_ch_config[0].callback != NULL){
    PITD2.pit_ch_config[0].callback();
  }
  
  PITD2.pit_tagp->CH[0].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT1_CHANNEL_CH1_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 1 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT1_CH1_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD2.pit_ch_config[1].callback != NULL){
    PITD2.pit_ch_config[1].callback();
  }
  
  PITD2.pit_tagp->CH[1].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT1_CHANNEL_CH2_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 2 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT1_CH2_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD2.pit_ch_config[2].callback != NULL){
    PITD2.pit_ch_config[2].callback();
  }
  
  PITD2.pit_tagp->CH[2].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT1_CHANNEL_CH3_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 3 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT1_CH3_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD2.pit_ch_config[3].callback != NULL){
    PITD2.pit_ch_config[3].callback();
  }
  
  PITD2.pit_tagp->CH[3].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT1_CHANNEL_CH4_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 4 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT1_CH4_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD2.pit_ch_config[4].callback != NULL){
    PITD2.pit_ch_config[4].callback();
  }
  
  PITD2.pit_tagp->CH[4].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT1_CHANNEL_CH5_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 5 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT1_CH5_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD2.pit_ch_config[5].callback != NULL){
    PITD2.pit_ch_config[5].callback();
  }
  
  PITD2.pit_tagp->CH[5].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT1_CHANNEL_CH6_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 6 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT1_CH6_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD2.pit_ch_config[6].callback != NULL){
    PITD2.pit_ch_config[6].callback();
  }
  
  PITD2.pit_tagp->CH[6].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif

#if (PIT1_CHANNEL_CH7_ENABLED == TRUE) || defined(__DOXYGEN__)
/**
 * @brief   PIT CHANNEL 7 interrupt handler.
 *
 * @isr
 */
IRQ_HANDLER(SPC5_PIT1_CH7_HANDLER) {

  IRQ_PROLOGUE();

  /* call the related callback */
  if (PITD2.pit_ch_config[7].callback != NULL){
    PITD2.pit_ch_config[7].callback();
  }
  
  PITD2.pit_tagp->CH[7].TFLG.R = 1;

  IRQ_EPILOGUE();
}
#endif
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level pit driver initialization.
 *
 * @init
 */
void pit_lld_init(void) {

#if (SPC5_PIT_USE_PIT0 == TRUE) || defined(__DOXYGEN__)
  PITD1.pit_tagp = &SPC5_PIT0;
#if (PIT0_CHANNEL_CH0_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT0_CH0_INT_NUMBER) = SPC5_PIT0_CH0_IRQ_PRIO;
#endif
#if (PIT0_CHANNEL_CH1_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT0_CH1_INT_NUMBER) = SPC5_PIT0_CH1_IRQ_PRIO;
#endif
#if (PIT0_CHANNEL_CH2_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT0_CH2_INT_NUMBER) = SPC5_PIT0_CH2_IRQ_PRIO;
#endif
#if (PIT0_CHANNEL_CH3_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT0_CH3_INT_NUMBER) = SPC5_PIT0_CH3_IRQ_PRIO;
#endif
#if (PIT0_CHANNEL_CH4_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT0_CH4_INT_NUMBER) = SPC5_PIT0_CH4_IRQ_PRIO;
#endif
#if (PIT0_CHANNEL_CH5_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT0_CH5_INT_NUMBER) = SPC5_PIT0_CH5_IRQ_PRIO;
#endif
#if (PIT0_CHANNEL_CH6_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT0_CH6_INT_NUMBER) = SPC5_PIT0_CH6_IRQ_PRIO;
#endif
#if (PIT0_CHANNEL_CH7_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT0_CH7_INT_NUMBER) = SPC5_PIT0_CH7_IRQ_PRIO;
#endif
#endif
#if (SPC5_PIT_USE_PIT1 == TRUE) || defined(__DOXYGEN__)
  PITD2.pit_tagp = &SPC5_PIT1;
#if (PIT1_CHANNEL_CH0_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT1_CH0_INT_NUMBER) = SPC5_PIT1_CH0_IRQ_PRIO;
#endif
#if (PIT1_CHANNEL_CH1_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT1_CH1_INT_NUMBER) = SPC5_PIT1_CH1_IRQ_PRIO;
#endif
#if (PIT1_CHANNEL_CH2_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT1_CH2_INT_NUMBER) = SPC5_PIT1_CH2_IRQ_PRIO;
#endif
#if (PIT1_CHANNEL_CH3_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT1_CH3_INT_NUMBER) = SPC5_PIT1_CH3_IRQ_PRIO;
#endif
#if (PIT1_CHANNEL_CH4_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT1_CH4_INT_NUMBER) = SPC5_PIT1_CH4_IRQ_PRIO;
#endif
#if (PIT1_CHANNEL_CH5_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT1_CH5_INT_NUMBER) = SPC5_PIT1_CH5_IRQ_PRIO;
#endif
#if (PIT1_CHANNEL_CH6_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT1_CH6_INT_NUMBER) = SPC5_PIT1_CH6_IRQ_PRIO;
#endif
#if (PIT1_CHANNEL_CH7_ENABLED == TRUE)
  INTC_PSR(SPC5_PIT1_CH7_INT_NUMBER) = SPC5_PIT1_CH7_IRQ_PRIO;
#endif
#endif
}

/**
 * @brief   Activate and configure PIT peripheral
 *
 * @param[in] pitd      pointer to the @p PITDriver object
 * @param[in] config	pointer to the  @p PIT_CH_Config object
 *
 * @api
 */
void pit_lld_start (PITDriver *pitd, PIT_CH_Config *config){

  pitd->pit_ch_config = config;      /* set Driver configuration                 */
/* The following code has been removed because the PIT0 is already enabled in 
   the OSAL component for the system time*/
#if 1
#if (SPC5_PIT_USE_PIT0 == TRUE)
  if (&PITD1 == pitd) {
    SPC5_PIT0_ENABLE_CLOCK();        /* start the peripheral clock               */
    pitd->pit_tagp->PITMCR.R = 1;    /* PIT clock enabled, stop while debugging. */
  }
#endif
#endif
#if (SPC5_PIT_USE_PIT1 == TRUE)
  if (&PITD2 == pitd) {
    SPC5_PIT1_ENABLE_CLOCK();        /* start the peripheral clock               */
    pitd->pit_tagp->PITMCR.R = 1;    /* PIT clock enabled, stop while debugging. */
  }
#endif  

}

/**
 * @brief   start a PIT Channel with frequency and callback defined in the configurator
 *
 * @param[in] pitd      pointer to the @p PITDriver object
 * @param[in] channel	number of channel to activate
 *
 * @api
 */
void pit_lld_channel_start (PITDriver *pitd, uint8_t channel){

  uint32_t reg = 0;

  if (pitd->pit_ch_config[channel].enabled == TRUE){
    /* Check if already started as Chain Mode */
    if ((pitd->pit_tagp->CH[channel].TCTRL.R & 4U) == 0U) {
#if (SPC5_PIT_USE_PIT0 == TRUE)
      if (&PITD1 == pitd) {
        reg = (SPCGetPeripheralClock(SPC5_PERIPHERAL_PIT0) / pitd->pit_ch_config[channel].frequency) - 1U;
      }
#endif
#if (SPC5_PIT_USE_PIT1 == TRUE)
      if (&PITD2 == pitd) {
        reg = (SPCGetPeripheralClock(SPC5_PERIPHERAL_PIT1) / pitd->pit_ch_config[channel].frequency) - 1U;
      }
#endif
      pitd->pit_tagp->CH[channel].LDVAL.R = reg;
      pitd->pit_tagp->CH[channel].CVAL.R  = reg;
      pitd->pit_tagp->CH[channel].TFLG.R  = 1U;    	/* Interrupt flag cleared. */

      /* Enable interrupt if a callback was provided */
      if (pitd->pit_ch_config[channel].callback != NULL) {
        pitd->pit_tagp->CH[channel].TCTRL.R = 3U;	/* Timer active, interrupt enabled. */
      } else {
        pitd->pit_tagp->CH[channel].TCTRL.R = 1U;	/* Timer active. */	
      }
    }
  }
}

/**
 * @brief   Change PIT Channel frequency 
 *
 * @param[in] pitd       pointer to the @p PITDriver object
 * @param[in] channel	 number of channel to activate
 * @param[in] frequency  new frequency in Hertz
 *
 * @api
 */
void pit_lld_channel_set_freq (PITDriver *pitd, uint8_t channel,uint32_t frequency){
  
  uint32_t reg = 0UL;
   
  if (pitd->pit_ch_config[channel].enabled == TRUE){
	/* Stop timer */	
    pitd->pit_tagp->CH[channel].TCTRL.R = 0U; 
    /* Set new frequency and restart timer */
    pitd->pit_ch_config[channel].frequency = frequency;    
#if (SPC5_PIT_USE_PIT0 == TRUE)
    if (&PITD1 == pitd) {
      reg = (SPCGetPeripheralClock(SPC5_PERIPHERAL_PIT0) / pitd->pit_ch_config[channel].frequency) - 1U;
    }
#endif
#if (SPC5_PIT_USE_PIT1 == TRUE)
    if (&PITD2 == pitd) {
      reg = (SPCGetPeripheralClock(SPC5_PERIPHERAL_PIT1) / pitd->pit_ch_config[channel].frequency) - 1U;
    }
#endif
    pitd->pit_tagp->CH[channel].LDVAL.R = reg;
    pitd->pit_tagp->CH[channel].CVAL.R  = reg;
    pitd->pit_tagp->CH[channel].TFLG.R  = 1U;    /* Interrupt flag cleared.                  */
    /* Enable interrupt if a callback was provided */
    if (pitd->pit_ch_config[channel].callback != NULL) {
      pitd->pit_tagp->CH[channel].TCTRL.R = 3U;	/* Timer active, interrupt enabled. */
    } else {
      pitd->pit_tagp->CH[channel].TCTRL.R = 1U;	/* Timer active. */	
    }
  }
}

/**
 * @brief   STOP a PIT Channel 
 *
 * @param[in] pitd      pointer to the @p PITDriver object
 * @param[in] channel	number of channel to deactivate
 *
 * @api
 */
void pit_lld_channel_stop (PITDriver *pitd, uint8_t channel){

  if (pitd->pit_ch_config[channel].enabled == TRUE){
    /* Check if enabled as Normal Mode */
    if ((pitd->pit_tagp->CH[channel].TCTRL.R & 4U) == 0U) {
      pitd->pit_tagp->CH[channel].TCTRL.R = 0U;    /* Timer Inactive, interrupt disabled. */
    }
  }
}

/**
 * @brief   get timer in microseconds
 *
 * @param[in] pitd      pointer to the @p PITDriver object
 * @param[in] channel	number of channel to probe
 *
 * @return				the current time value.
 *
 * @api
 */
uint32_t pit_lld_channel_get(PITDriver *pitd, uint8_t channel) {

  uint32_t tctrl = pitd->pit_tagp->CH[channel].TCTRL.R;
  uint32_t udiv = 1UL;
  uint32_t usec = 0UL;

#if (SPC5_PIT_USE_PIT0 == TRUE)
  if (&PITD1 == pitd) {
    udiv = SPCGetPeripheralClock(SPC5_PERIPHERAL_PIT0) / 1000000U;
  }
#endif
#if (SPC5_PIT_USE_PIT1 == TRUE)
  if (&PITD2 == pitd) {
    udiv = SPCGetPeripheralClock(SPC5_PERIPHERAL_PIT1) / 1000000U;
  }
#endif

  if ((pitd->pit_ch_config[channel].enabled == TRUE) &&
      (pitd->pit_ch_config[channel].callback == NULL) &&
     ((tctrl & 1U) == 1U) &&	((tctrl & 4U) == 0U)) {
      usec = pitd->pit_tagp->CH[channel].CVAL.R / udiv;
  }
  return usec;
}

/**
 * @brief   start a PIT enabling the Lifetime feature
 *
 * @param[in] pitd      pointer to the PIT1 @p PITDriver object
 *
 * @api
 */
void pit_lld_lifetime_start(PITDriver *pitd) {

  uint32_t tctrl0 = pitd->pit_tagp->CH[0].TCTRL.R;
  uint32_t tctrl1 = pitd->pit_tagp->CH[1].TCTRL.R;

  /* Check if already enabled as normal PIT */
  if (((tctrl0 & 1U) == 0U) && ((tctrl1 & 1U) == 0U)) {
    pitd->pit_tagp->CH[1].LDVAL.R = 0xffffffffU; /* setup timer 1 for maximum counting period */
    pitd->pit_tagp->CH[1].TCTRL.R = 0U;  /* disable timer interrupts */
    pitd->pit_tagp->CH[1].TCTRL.R |= 4U; /* chain timer 1 to timer 0 */
    pitd->pit_tagp->CH[1].TCTRL.R |= 1U; /* start timer 1 */

    pitd->pit_tagp->CH[0].LDVAL.R = 0xffffffffU; /* setup timer 0 for maximum counting period */
    pitd->pit_tagp->CH[0].TCTRL.R = 0U;  /* disable timer interrupts */
    pitd->pit_tagp->CH[0].TCTRL.R |= 4U; /* chain setup */
    pitd->pit_tagp->CH[0].TCTRL.R |= 1U; /* start timer 0 */
  }
}

/**
 * @brief   stop a PIT disabling the Lifetime feature
 *
 * @param[in] pitd      pointer to the PIT1 @p PITDriver object
 *
 * @api
 */
void pit_lld_lifetime_stop(PITDriver *pitd) {

  uint32_t tctrl0 = pitd->pit_tagp->CH[0].TCTRL.R;
  uint32_t tctrl1 = pitd->pit_tagp->CH[1].TCTRL.R;

  /* Check if enabled in Chain Mode */
  if (((tctrl0 & 4U) != 0U) && (tctrl1 & 4U) != 0U) {
    pitd->pit_tagp->CH[0].TCTRL.R = 0U; /* stop timer 0 */
    pitd->pit_tagp->CH[1].TCTRL.R = 0U; /* stop timer 1 */
  }
}

/**
 * @brief   get Lifetime timer in microseconds
 *
 * @param[in] pitd      pointer to the PIT1 @p PITDriver object
 *
 * @return				the current time value.
 *
 * @api
 */
uint64_t pit_lld_lifetime_get(PITDriver *pitd) {

  uint32_t udiv = 1UL;
  uint64_t usec = 0UL;

  if ((pitd->pit_tagp->CH[1].TCTRL.R & 4U) != 0U) {
	udiv = SPCGetPeripheralClock(SPC5_PERIPHERAL_PIT1) / 1000000U;
    usec = pitd->pit_tagp->LMTRH.R;
    usec <<= 32U;
    usec |= pitd->pit_tagp->LMTRL.R;
    usec /= udiv;
  }
  return usec;
}

/**
 * @brief   STOP the PIT peripheral
 *
 * @param[in] pitd      pointer to the @p PITDriver object
 * @note 				PIT0 cannot disabled because it is used as System Timer
 * @api
 */
void pit_lld_stop (PITDriver *pitd){

  pitd->pit_ch_config =  NULL;    /* reset Driver configuration   */
#if (SPC5_PIT_USE_PIT0 == TRUE)
	/* we cannot stop it because used as System Timer */
#endif
#if (SPC5_PIT_USE_PIT1 == TRUE)
  if (&PITD2 == pitd) {
    pitd->pit_tagp->PITMCR.R = 0x6;   /* PIT clock Disabled.        */
    SPC5_PIT1_DISABLE_CLOCK();       /* stop the peripheral clock  */
  }
#endif  
  
}
#endif /* LLD_USE_PIT */

/** @} */
