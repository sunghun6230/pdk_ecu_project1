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
 * @file    can_lld_cfg.c
 * @brief   CAN Driver configuration code.
 *
 * @addtogroup CAN
 * @{
 */
 
#include "can_lld_cfg.h"

#if (LLD_USE_CAN == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   Structure defining the CAN configuration "mcanconf".
 */
CANConfig can_config_mcanconf = {
  /* loopback */
  CAN_NO_LOOPBACK,
  /* endianness */
  CAN_BIG_ENDIAN,
  /* sync jump width */ 
  15U,  
  /* time segment before sample point */
  62U,
  /* time segment after sample point */
  15U,
  /* clock prescaler */
  1U,
  
  /* Canfd configuration */
  TRUE,
  /* canfd bit rate switch */
  TRUE,
   /* sync jump width */ 
  3U,  
  /* time segment before sample point */
  14U,
  /* time segment after sample point */
  3U,
  /* clock prescaler */
  1U,
  /* transceiver delay compensation enabled */
  TRUE,
  /* transmitter delay compensation value */
  10U,
  /* transmitter delay compensation offset */
  7U,
  /* transmitter delay compensation filter window length */
  0U,

  /*can rx interrupt line*/
  CAN_RX_INT_DISABLE,
  /*can rx interrupt call back */
  NULL,
  /* FIFO 0 is not used */
  CAN_FIFO0_INT_DISABLE,0,NULL,0,NULL,0,NULL,0,NULL,
  /* FIFO 1 is not used */
  CAN_FIFO1_INT_DISABLE,0,NULL,0,NULL,0,NULL,0,NULL,
  /* rx standard buffer filters */
  {
  {0x2UL,0UL,0U,7U}, /* standard filter 0*/
  {0x34UL,1UL,0U,7U}, /* standard filter 1*/
  {0x36UL,2UL,0U,7U}, /* standard filter 2*/
  {0x423UL,3UL,0U,7U}, /* standard filter 3*/
  {0x700UL,8UL,0U,7U}, /* standard filter 4*/
  {0xfffUL,14UL,0U,7U}, /* standard filter 5*/
  {0xfffUL,15UL,0U,7U}, /* standard filter 6*/
  {0xfffUL,16UL,0U,7U}, /* standard filter 7*/
  {0x8ffUL,17UL,0U,7U}, /* standard filter 8*/
  {0x8ffUL,19UL,0U,7U}, /* standard filter 9*/
  {0UL,0UL,0U,0U},  /* standard_filter unused 0 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 1 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 2 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 3 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 4 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 5 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 6 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 7 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 8 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 9 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 10 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 11 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 12 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 13 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 14 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 15 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 16 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 17 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 18 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 19 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 20 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 21 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 22 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 23 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 24 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 25 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 26 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 27 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 28 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 29 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 30 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 31 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 32 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 33 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 34 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 35 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 36 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 37 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 38 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 39 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 40 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 41 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 42 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 43 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 44 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 45 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 46 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 47 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 48 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 49 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 50 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 51 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 52 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 53 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 54 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 55 (MISRA Compliance) */
  },
    /* rx extended buffer filters */
  {
  {0x218a006UL,4UL,0U,7U}, /* Extended filter 0*/
  {0x4394000UL,5UL,0U,7U}, /* Extended filter 1*/
  {0x18febf0bUL,6UL,0U,7U}, /* Extended filter 2*/
  {0x18fef100UL,7UL,0U,7U}, /* Extended filter 3*/
  {0x18ef4a1fUL,9UL,0U,7U}, /* Extended filter 4*/
  {0x18f00503UL,10UL,0U,7U}, /* Extended filter 5*/
  {0x18f0090bUL,11UL,0U,7U}, /* Extended filter 6*/
  {0x18febf0bUL,12UL,0U,7U}, /* Extended filter 7*/
  {0xcfdcc27UL,13UL,0U,7U}, /* Extended filter 8*/
  {0x8ffUL,17UL,0U,7U}, /* Extended filter 9*/
  {0x8ffUL,20UL,0U,7U}, /* Extended filter 10*/
  {0UL,0UL,0U,0U},  /* extended_filter unused 0 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 1 (MISRA Compliance) */
  },
/* number of standard filters */
  10U,
/* number of extended filters */
  11U,
  /* number of Rx Buffers */
  20U,
/* number of FIFO 0 Buffers */
  0U,
/* number of FIFO 1 Buffers */
  0U,
/* Transmission Mode */
  CAN_DEDICATED_TXBUFFER,
/* number of Tx Buffers */
  16U,
/* number of FIFO or QUEUE Tx Buffers */
  16U,
/* Error Callback*/
  NULL,
};

/**
 * @brief   Structure defining the CAN configuration "Lcanfdconf".
 */
CANConfig can_config_Lcanfdconf = {
  /* loopback */
  CAN_NO_LOOPBACK,
  /* endianness */
  CAN_BIG_ENDIAN,
  /* sync jump width */ 
  1U,  
  /* time segment before sample point */
  13U,
  /* time segment after sample point */
  4U,
  /* clock prescaler */
  2U,
  
  /* Canfd configuration */
  TRUE,
  /* canfd bit rate switch */
  TRUE,
   /* sync jump width */ 
  1U,  
  /* time segment before sample point */
  4U,
  /* time segment after sample point */
  1U,
  /* clock prescaler */
  1U,
  /* transceiver delay compensation enabled */
  TRUE,
  /* transmitter delay compensation value */
  10U,
  /* transmitter delay compensation offset */
  6U,
  /* transmitter delay compensation filter window length */
  0U,

  /*can rx interrupt line*/
  CAN_LINE0_INT,
  /*can rx interrupt call back */
   Lcanfdconf_Lrxreceive,
  /* FIFO 0 is not used */
  CAN_FIFO0_INT_DISABLE,0,NULL,0,NULL,0,NULL,0,NULL,
  /* FIFO 1 is not used */
  CAN_FIFO1_INT_DISABLE,0,NULL,0,NULL,0,NULL,0,NULL,
  /* rx standard buffer filters */
  {
  {0x100UL,0UL,0U,7U}, /* standard filter 0*/
  {0x110UL,1UL,0U,7U}, /* standard filter 1*/
  {0x111UL,2UL,0U,7U}, /* standard filter 2*/
  {0x120UL,3UL,0U,7U}, /* standard filter 3*/
  {0x121UL,4UL,0U,7U}, /* standard filter 4*/
  {0x130UL,5UL,0U,7U}, /* standard filter 5*/
  {0x131UL,6UL,0U,7U}, /* standard filter 6*/
  {0x200UL,7UL,0U,7U}, /* standard filter 7*/
  {0x201UL,8UL,0U,7U}, /* standard filter 8*/
  {0x202UL,9UL,0U,7U}, /* standard filter 9*/
  {0x203UL,10UL,0U,7U}, /* standard filter 10*/
  {0x204UL,11UL,0U,7U}, /* standard filter 11*/
  {0x205UL,12UL,0U,7U}, /* standard filter 12*/
  {0x206UL,13UL,0U,7U}, /* standard filter 13*/
  {0x207UL,14UL,0U,7U}, /* standard filter 14*/
  {0x208UL,15UL,0U,7U}, /* standard filter 15*/
  {0x209UL,16UL,0U,7U}, /* standard filter 16*/
  {0x20aUL,17UL,0U,7U}, /* standard filter 17*/
  {0x300UL,18UL,0U,7U}, /* standard filter 18*/
  {0x301UL,19UL,0U,7U}, /* standard filter 19*/
  {0x302UL,20UL,0U,7U}, /* standard filter 20*/
  {0x303UL,21UL,0U,7U}, /* standard filter 21*/
  {0x304UL,22UL,0U,7U}, /* standard filter 22*/
  {0x305UL,23UL,0U,7U}, /* standard filter 23*/
  {0x306UL,24UL,0U,7U}, /* standard filter 24*/
  {0x307UL,25UL,0U,7U}, /* standard filter 25*/
  {0x308UL,26UL,0U,7U}, /* standard filter 26*/
  {0x309UL,27UL,0U,7U}, /* standard filter 27*/
  {0x30aUL,28UL,0U,7U}, /* standard filter 28*/
  {0x400UL,29UL,0U,7U}, /* standard filter 29*/
  {0x401UL,30UL,0U,7U}, /* standard filter 30*/
  {0x402UL,31UL,0U,7U}, /* standard filter 31*/
  {0x403UL,32UL,0U,7U}, /* standard filter 32*/
  {0x404UL,33UL,0U,7U}, /* standard filter 33*/
  {0x405UL,34UL,0U,7U}, /* standard filter 34*/
  {0x406UL,35UL,0U,7U}, /* standard filter 35*/
  {0x407UL,36UL,0U,7U}, /* standard filter 36*/
  {0x408UL,37UL,0U,7U}, /* standard filter 37*/
  {0x409UL,38UL,0U,7U}, /* standard filter 38*/
  {0x40aUL,39UL,0U,7U}, /* standard filter 39*/
  {0x423UL,40UL,0U,7U}, /* standard filter 40*/
  {0x423UL,41UL,0U,7U}, /* standard filter 41*/
  {0x423UL,42UL,0U,7U}, /* standard filter 42*/
  {0x423UL,43UL,0U,7U}, /* standard filter 43*/
  {0x423UL,44UL,0U,7U}, /* standard filter 44*/
  {0x423UL,45UL,0U,7U}, /* standard filter 45*/
  {0x423UL,46UL,0U,7U}, /* standard filter 46*/
  {0x423UL,47UL,0U,7U}, /* standard filter 47*/
  {0x423UL,48UL,0U,7U}, /* standard filter 48*/
  {0x423UL,49UL,0U,7U}, /* standard filter 49*/
  {0x423UL,50UL,0U,7U}, /* standard filter 50*/
  {0UL,0UL,0U,0U},  /* standard_filter unused 0 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 1 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 2 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 3 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 4 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 5 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 6 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 7 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 8 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 9 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 10 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 11 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 12 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 13 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 14 (MISRA Compliance) */
  },
    /* rx extended buffer filters */
  {
  {0UL,0UL,0U,0U},  /* extended_filter unused 0 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 1 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 2 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 3 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 4 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 5 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 6 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 7 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 8 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 9 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 10 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 11 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 12 (MISRA Compliance) */
  },
/* number of standard filters */
  51U,
/* number of extended filters */
  0U,
  /* number of Rx Buffers */
  50U,
/* number of FIFO 0 Buffers */
  0U,
/* number of FIFO 1 Buffers */
  0U,
/* Transmission Mode */
  CAN_DEDICATED_TXBUFFER,
/* number of Tx Buffers */
  16U,
/* number of FIFO or QUEUE Tx Buffers */
  16U,
/* Error Callback*/
  NULL,
};

/**
 * @brief   Structure defining the CAN configuration "Rcanfdconf".
 */
CANConfig can_config_Rcanfdconf = {
  /* loopback */
  CAN_NO_LOOPBACK,
  /* endianness */
  CAN_BIG_ENDIAN,
  /* sync jump width */ 
  1U,  
  /* time segment before sample point */
  13U,
  /* time segment after sample point */
  4U,
  /* clock prescaler */
  2U,
  
  /* Canfd configuration */
  TRUE,
  /* canfd bit rate switch */
  TRUE,
   /* sync jump width */ 
  1U,  
  /* time segment before sample point */
  4U,
  /* time segment after sample point */
  1U,
  /* clock prescaler */
  1U,
  /* transceiver delay compensation enabled */
  TRUE,
  /* transmitter delay compensation value */
  10U,
  /* transmitter delay compensation offset */
  6U,
  /* transmitter delay compensation filter window length */
  0U,

  /*can rx interrupt line*/
  CAN_LINE1_INT,
  /*can rx interrupt call back */
   Rcanfdconf_Rrxreceive,
  /* FIFO 0 is not used */
  CAN_FIFO0_INT_DISABLE,0,NULL,0,NULL,0,NULL,0,NULL,
  /* FIFO 1 is not used */
  CAN_FIFO1_INT_DISABLE,0,NULL,0,NULL,0,NULL,0,NULL,
  /* rx standard buffer filters */
  {
  {0x100UL,0UL,0U,7U}, /* standard filter 0*/
  {0x110UL,1UL,0U,7U}, /* standard filter 1*/
  {0x111UL,2UL,0U,7U}, /* standard filter 2*/
  {0x120UL,3UL,0U,7U}, /* standard filter 3*/
  {0x121UL,4UL,0U,7U}, /* standard filter 4*/
  {0x130UL,5UL,0U,7U}, /* standard filter 5*/
  {0x131UL,6UL,0U,7U}, /* standard filter 6*/
  {0x200UL,7UL,0U,7U}, /* standard filter 7*/
  {0x201UL,8UL,0U,7U}, /* standard filter 8*/
  {0x202UL,9UL,0U,7U}, /* standard filter 9*/
  {0x203UL,10UL,0U,7U}, /* standard filter 10*/
  {0x204UL,11UL,0U,7U}, /* standard filter 11*/
  {0x205UL,12UL,0U,7U}, /* standard filter 12*/
  {0x206UL,13UL,0U,7U}, /* standard filter 13*/
  {0x207UL,14UL,0U,7U}, /* standard filter 14*/
  {0x208UL,15UL,0U,7U}, /* standard filter 15*/
  {0x209UL,16UL,0U,7U}, /* standard filter 16*/
  {0x20aUL,17UL,0U,7U}, /* standard filter 17*/
  {0x20bUL,18UL,0U,7U}, /* standard filter 18*/
  {0x300UL,19UL,0U,7U}, /* standard filter 19*/
  {0x301UL,20UL,0U,7U}, /* standard filter 20*/
  {0x302UL,21UL,0U,7U}, /* standard filter 21*/
  {0x303UL,22UL,0U,7U}, /* standard filter 22*/
  {0x304UL,23UL,0U,7U}, /* standard filter 23*/
  {0x305UL,24UL,0U,7U}, /* standard filter 24*/
  {0x306UL,25UL,0U,7U}, /* standard filter 25*/
  {0x307UL,26UL,0U,7U}, /* standard filter 26*/
  {0x400UL,27UL,0U,7U}, /* standard filter 27*/
  {0x401UL,28UL,0U,7U}, /* standard filter 28*/
  {0x402UL,29UL,0U,7U}, /* standard filter 29*/
  {0x403UL,30UL,0U,7U}, /* standard filter 30*/
  {0x404UL,31UL,0U,7U}, /* standard filter 31*/
  {0x405UL,32UL,0U,7U}, /* standard filter 32*/
  {0x406UL,33UL,0U,7U}, /* standard filter 33*/
  {0x407UL,34UL,0U,7U}, /* standard filter 34*/
  {0x500UL,35UL,0U,7U}, /* standard filter 35*/
  {0x501UL,36UL,0U,7U}, /* standard filter 36*/
  {0x502UL,37UL,0U,7U}, /* standard filter 37*/
  {0x503UL,38UL,0U,7U}, /* standard filter 38*/
  {0x504UL,39UL,0U,7U}, /* standard filter 39*/
  {0x505UL,40UL,0U,7U}, /* standard filter 40*/
  {0x506UL,41UL,0U,7U}, /* standard filter 41*/
  {0x507UL,42UL,0U,7U}, /* standard filter 42*/
  {0x508UL,43UL,0U,7U}, /* standard filter 43*/
  {0x509UL,44UL,0U,7U}, /* standard filter 44*/
  {0x50aUL,45UL,0U,7U}, /* standard filter 45*/
  {0x50bUL,46UL,0U,7U}, /* standard filter 46*/
  {0x600UL,47UL,0U,7U}, /* standard filter 47*/
  {0x601UL,48UL,0U,7U}, /* standard filter 48*/
  {0x602UL,49UL,0U,7U}, /* standard filter 49*/
  {0x603UL,50UL,0U,7U}, /* standard filter 50*/
  {0x604UL,51UL,0U,7U}, /* standard filter 51*/
  {0x605UL,52UL,0U,7U}, /* standard filter 52*/
  {0x606UL,53UL,0U,7U}, /* standard filter 53*/
  {0x607UL,54UL,0U,7U}, /* standard filter 54*/
  {0x700UL,55UL,0U,7U}, /* standard filter 55*/
  {0x701UL,56UL,0U,7U}, /* standard filter 56*/
  {0x702UL,57UL,0U,7U}, /* standard filter 57*/
  {0x703UL,58UL,0U,7U}, /* standard filter 58*/
  {0x704UL,59UL,0U,7U}, /* standard filter 59*/
  {0x705UL,60UL,0U,7U}, /* standard filter 60*/
  {0x706UL,61UL,0U,7U}, /* standard filter 61*/
  {0x707UL,62UL,0U,7U}, /* standard filter 62*/
  {0x708UL,63UL,0U,7U}, /* standard filter 63*/
  {0UL,0UL,0U,0U},  /* standard_filter unused 0 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* standard_filter unused 1 (MISRA Compliance) */
  },
    /* rx extended buffer filters */
  {
  {0UL,0UL,0U,0U},  /* extended_filter unused 0 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 1 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 2 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 3 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 4 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 5 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 6 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 7 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 8 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 9 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 10 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 11 (MISRA Compliance) */
  {0UL,0UL,0U,0U},  /* extended_filter unused 12 (MISRA Compliance) */
  },
/* number of standard filters */
  64U,
/* number of extended filters */
  0U,
  /* number of Rx Buffers */
  64U,
/* number of FIFO 0 Buffers */
  0U,
/* number of FIFO 1 Buffers */
  0U,
/* Transmission Mode */
  CAN_DEDICATED_TXBUFFER,
/* number of Tx Buffers */
  16U,
/* number of FIFO or QUEUE Tx Buffers */
  16U,
/* Error Callback*/
  NULL,
};

/*===========================================================================*/
/* Driver local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

#endif /* LLD_USE_CAN */

/** @} */
