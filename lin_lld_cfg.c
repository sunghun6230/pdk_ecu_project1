/****************************************************************************
*
* Copyright © 2018-2019 STMicroelectronics - All Rights Reserved
*
* This software is licensed under SLA0098 terms that can be found in the
* DM00779817_1_0.pdf file in the licenses directory of this software product.
* 
* THIS SOFTWARE IS DISTRIBUTED "AS IS," AND ALL WARRANTIES ARE DISCLAIMED,
* INCLUDING MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
*
*****************************************************************************/
/**
 * @file    lin_lld_cfg.c
 * @brief   LIN Driver configuration code.
 *
 * @addtogroup LIN
 * @{
 */

#include <lin_lld.h>

#if (LLD_USE_LIN == TRUE) || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/



/**
 * @brief   Structure defining the LIN configuration "lin_config_Pas".
 */
LinConfig lin_config_lin_config_Pas = {
  19200,
  LIN_MODE_MASTER,
  SPC5_LIN_API_MODE_ASYNCHRONOUS,
  NULL,
  NULL,
  NULL,
  0,
  FALSE,
  NULL
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

#endif /* LLD_USE_LIN */

/** @} */
