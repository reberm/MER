/*
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2017 STMicroelectronics</center></h2>
  *
  * Licensed under ADG License Agreement, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://goo.gl/28pHKW
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
*/

/*
*   @file    ICS_PWMnCurrFdbkClass.c
*   @version BETA 0.9.1
*   @brief   This file contains private impelementation of derived class for 
*            single motor ICS current reading.
*          
*   @details PWM generation and Current sensing using FlexPWM, ADC, DMA and CTU module.
*
*/

/** @addtogroup SPC5_PMSM_MC_Library
  * @{
*/

/** @addtogroup PWMnCurrFdbk_ICS
  * @{
*/

/**
*   @defgroup ICS ICS class Description
*   @brief   ICS PWMnCurrFdbk class implementation
*   @details ICS current sensing carried out through isolated current sensors or on shunt resistors. 
*            Also the shunt resistors positioning must be configured: on phases or on legs of inverter. 
*            The sampling of the currents can be performed every where in the PWM period. It has been chosen 
*            to sample simultaneously the current flows when the bottom transistor of the respective inverter 
*            leg is switched on.
*            \image html ICS_figure1.png "Current Sensing on phases"
*            \image html ICS_figure3.png "Current Sensing on legs"
*            In standard motor operation, where the supplied voltage is generated using the space vector
*            modulation, the sampling instant of phase current takes place in the middle of the PWM period
*            in which all bottom transistors are switched on.\n\n
*            The three currents I1, I2, and I3 flowing through a three-phase system follow the mathematical
*            relationship:\n\n
*            I1 + I2 + I3 = 0\n\n
*            Therefore, to reconstruct the currents flowing through a generic three-phase load, it is 
*            sufficient to sample only two out of the three currents while the third one can be computed by 
*            using the above relationship.\n\n
*            For correct calculation of the FOC algorithm, the phase current measurements must be
*            executed at the same time. The flexibility of the ADC converter makes it possible the sampling
*            of the phases at the same time using Dual Conversion Mode. Moreover, also the SINGLE ADC conversion
*            can be used when only one ADC module is available. The sequence of these two
*            measurements is handled by CTU triggers T0CR. The result of each conversion is stored in
*            one of the four available FIFOs of CTU module. DMA transfer is used to access to CTU FIFO.\n\n
*            \image html ICS_figure2.png "Current Sensing"
*            This class generate duty cycles phase A, phase B, and phase C of different PWM periods.
*            These phase voltage waveforms correspond to a center-aligned PWM.\n\n
*            Moreover, the class can be used to execute a regular conversion for other purpose (for
*            example, for Bus voltage, temperature sensor, etc).\n\n
*            The CICS_PWMC derived class uses the following hardware peripherals:\n\n
*            \par FLEXPWM
*            - The class uses six PWM channels, each of which is configured to control a single
*              half-bridge power stage
*            - PWM is configured in complementary mode to drive a 3-phase DC/AC inverter
*            - FlexPWM module is configured in PWM Center-Aligned mode and in signed-mode: INIT
*              register is configured with negative value (2’s complement) of VAL1
*            - FlexPWM modules are synchronized using the Master Reload Signal (MRS). The
*              FlexPWM_0 submodule #0 is configured to run as a master and to generate MRS and
*              counter synchronization signal (master sync) for other submodules. 
*            - The MRS signal is generated every second opportunity of submodule #0, VAL1 compare,
*              that is, full cycle reload. All double buffered registers, including compare registers VAL2,
*              VAL3 are updated on the occurrence of MRS, therefore new PWM duty cycles are updated
*              each PWM periods (configurable).
*            \par CTU
*            - The MRS signal generated from the FlexPWM module is internally routed to the Cross
*              Triggering Unit (CTU) module
*            - The MRS signal is generated for each PWM periods (configurable). The CTU counter can count up 
*              to value equal to PWM period
*            - T0CR trigger compare registers is used to generate ADC command event: The value of
*              T0CR is configured to zero. In this way, when a T0CR trigger events occurs, it generates
*              the ADC start of conversion request at the beginning of each PWM reload cycle. So, the
*              phase current is measured when the bottom transistor are swithed on.
*            \par ADC
*            - ADC channels are configured in Injection mode to perform Current Reading Calibration
*            - ADC channels are configured in Dual Conversion mode to sampling of the phases at the
*              same time
*            - ADC can be configured in Single Conversion mode to execute User regular conversion.
*            - ADC can be configured in Single Conversion mode to execute Voltage Bus conversion.
*            \par DMA
*            - DMA module is used to transfer the phases converted values from CTU FIFO.
*/

/**
  * @}
  */
  
/**
  * @}
  */


/*
* @page misra_violations MISRA-C:2004 violations
*
* @section ICS_PWMnCurrFdbkClass_REF_1
* Violates MISRA 2004 Required Rule 11.1, Cast from unsigned int to pointer.
* This rule can not be avoided because it appears when addressing memory mapped registers or 
* other hardware specific feature
*
* @section ICS_PWMnCurrFdbkClass_REF_2
* Violates MISRA 2004 Required Rule 1.2, Place reliance on undefined or unspecified behaviour.
* Violation is needed to implement Data Hiding. It is used to preventing the unintended  changes on private 
* parameters and functions.
*
* @section ICS_PWMnCurrFdbkClass_REF_3
* Violates MISRA 2004 Advisory Rule 11.4, Cast from pointer to pointer. 
* Violation is needed to implement Data Hiding. It is used to preventing the unintended  changes on private 
* parameters and functions.
*
* @section ICS_PWMnCurrFdbkClass_REF_4
* Violates MISRA 2004 Advisory Rule 16.7,
* A pointer parameter in a function prototype should be declared as
* pointer to constif the pointer is not used to modify the addressed
* object. This rule can not be avoided because this pointer is used also to modify the addressed object.
*
* @section ICS_PWMnCurrFdbkClass_REF_5
* Violates MISRA 2004 Required Rule 10.1, cast from integer type to different type or
* a wider integer type
* This is used for the Speed optimization of the memory access.
*
* @section ICS_PWMnCurrFdbkClass_REF_6
* Violates MISRA 2004 Required Rule 10.5, if the  bitwise operator ~ and << are applied to an operand of
* underlying type unsigned char or unsigned short the result shall be immediately cast to the undrlying type
* of the operand
*
* @section ICS_PWMnCurrFdbkClass_REF_7
* Violates MISRA 2004 Required Rule 10.1,
* Cast from integer type to different type or a wider integer type
* This is used for the Speed optimization of the memory access. 
*
* @section ICS_PWMnCurrFdbkClass_REF_8
* Violates MISRA 2004 Required Rule 12.4, side effects on right hand of logical operator: '&&'
* This is used for the code complexity.
*
*/

/*===============================================================================================
*                                         INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
===============================================================================================*/
#include "PWMnCurrFdbkClass.h"
#include "Control stage parameters.h"
#include "PWMnCurrFdbkPrivate.h"
#include "ICS_PWMnCurrFdbkClass.h"
#include "ICS_PWMnCurrFdbkPrivate.h"
#include "MCLibraryConf.h"
#include "Reg_Macros.h"
#include "Reg_eSys_FlexPwm.h"
#include "MC_type.h"
#include "Reg_eSys_ADCDig.h"
#include "Dmamux_LLD.h"
#include "Dma_LLD.h"
#include "Reg_eSys_CTU2.h"
#include "pal.h"
#include "pal_lld.h"
#include "MC_Lib_Setup.h"
#include "Drive parameters.h"
#include "Parameters conversion.h"

/*===============================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
===============================================================================================*/

/*===============================================================================================
*                                       CONSTANTS
===============================================================================================*/
#define NB_CONVERSIONS 16u

#define FLEXPWM_NB        0x2U
#define FLEXPWM_SUBMOD_NB 0x4U

#define DCLASS_PARAMS (((_DCICS_PWMC)(((_CPWMC) this)->DerivedClass))->pDParams_str)
#define DCLASS_VARS  (((_DCICS_PWMC)(((_CPWMC) this)->DerivedClass))->DVars_str)

#ifdef MC_CLASS_DYNAMIC
#include "stdlib.h" /* Used for dynamic allocation */
#endif

/*===============================================================================================
*                 TO BE MOVED IN CONFIGURATION FILE (Fix to support FlexPWM SUB_MOD 0
===============================================================================================*/

/*===============================================================================================
*                                       LOCAL VARIABLES
===============================================================================================*/

/*===============================================================================================
*                                       GLOBAL CONSTANTS
===============================================================================================*/
#ifdef ADC_USER_ENABLE
extern const ICS_ADC_User_ch ICS_ADC_Curr[];
#endif
/*===============================================================================================
*                                       GLOBAL VARIABLES
===============================================================================================*/

/*===============================================================================================
*                                   LOCAL FUNCTION PROTOTYPES
===============================================================================================*/
/*#define DEBUG*/
#ifdef DEBUG
static volatile _CPWMC oPWMCdbg;
static volatile _DCICS_PWMC oICSdbg;
#endif

static void ICS_Init(CPWMC this);
static void ICS_CurrentReadingCalibration(CPWMC this);
static void ICS_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents);
static void ICS_TurnOnLowSides(CPWMC this);
static void ICS_SwitchOnPWM(CPWMC this);
static void ICS_SwitchOffPWM(CPWMC this);
static uint16 ICS_WriteFlexPWMRegisters(CPWMC this);
static void ICS_ADC_SetSamplingTime(ADConv_t ADConv_struct);
static uint16 ICS_ExecRegularConv(CPWMC this, uint8 bChannel, uint8 ADC_Unit);
static uint16 ICS_IsOverCurrentOccurred(CPWMC this);
static void ICS_FlexPWMInit(CPWMC this);
static void ICS_SetIdleStatePWM(uint8 submodule, uint16 channel, uint8 channel_mask);
static void ICS_ADC_Init(uint8 Adc_Unit, uint8 PwrDownDelay, uint32 SamplingTime);
static void ICS_CTU_Init(CPWMC this);
static void ICS_CTU_Start(CPWMC this);
#if ((TEMPERATURE_READING == DISABLE) && (BUS_VOLTAGE_READING == DISABLE) && !defined (ADC_USER_ENABLE))
static void ICS_CTU_Stop(void);
#endif
static uint16 ICS_GetMultipleRegularConv(CPWMC this, uint8 bChannel);

/**
* @brief Current CTU buffers
*/
static uint32 CurrentCTUFifoBuffer[2];

#if defined ADC_USER_ENABLE
uint32 RegularCTUFifoBuffer[1];
#ifdef ADC_MULTIPLE_CONV_ENABLED
uint16_t ADC_User_multiple_buffer[ADC_USER_CONV_MAX];
#endif /* ADC_MULTIPLE_CONV_ENABLED */
#endif /* ADC_USER_ENABLE */

#if defined BUS_VOLTAGE_MEASUREMENT
static uint32 VBusCTUFifoBuffer[1];
#endif /* BUS_VOLTAGE_MEASUREMENT */


/**
* @brief FlexPWM Base address
*/
static const uint32 FlexPWM_Regs[FLEXPWM_NB] = {
    FLEXPWM0_BASEADDR_CTRL,  /* first FlexPWM module */
#ifdef FLEXPWM_Module_1
    FLEXPWM1_BASEADDR_CTRL   /* second FlexPWM module */
#endif
};

/**
* @brief FlexPWM Submodules Base address
*/
static const uint32 FlexPWM_SubFlexPwm_Regs[FLEXPWM_NB*FLEXPWM_SUBMOD_NB] = {
    FLEXPWM0_BASEADDR_SUB0, /* first FlexPWM module */
    FLEXPWM0_BASEADDR_SUB1,
    FLEXPWM0_BASEADDR_SUB2,
    FLEXPWM0_BASEADDR_SUB3
#ifdef FLEXPWM_Module_1
    ,FLEXPWM1_BASEADDR_SUB0 /* second FlexPWM module */
    ,FLEXPWM1_BASEADDR_SUB1
    ,FLEXPWM1_BASEADDR_SUB2
    ,FLEXPWM1_BASEADDR_SUB3
#endif
};

/**
* @brief base address array for ADCDig
*/
const uint32 ADCDIG_BASE_ADDR[] = {
   ADC0_BASEADDR
#ifndef _SPC560PXX_SMALL_
   ,ADC1_BASEADDR
#endif   
};

/*===============================================================================================
*                                             FUNCTIONS
===============================================================================================*/

/**
* @brief        Creates an object of the class ICS
*
* @param[in]    pPWMnCurrFdbkParams pointer to an PWMnCurrFdbk parameters structure
* @param[in]    pICS_Params pointer to an ICS parameters structure
*
* @return       CICS_PWMC new instance of ICS object
*
* @api
*
*/
CICS_PWMC ICS_NewObject(pPWMnCurrFdbkParams_t pPWMnCurrFdbkParams, 
                                    pICS_Params_t pICS_Params)
{
  _CPWMC oPWMnCurrFdbk;
  _DCICS_PWMC oICS;

#ifndef MC_CLASS_DYNAMIC
   static _DCICS_PWMC_t ICS_PWMCpool[MAX_DRV_PWMC_NUM];
   static uint8 ICS_PWMC_Allocated = 0u;
#endif

  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  oPWMnCurrFdbk = (_CPWMC)PWMC_NewObject(pPWMnCurrFdbkParams);
  
#ifdef MC_CLASS_DYNAMIC
  oICS = (_DCICS_PWMC)calloc(1u,sizeof(_DCICS_PWMC_t));
#else
  if (ICS_PWMC_Allocated  < MAX_DRV_PWMC_NUM)
  {
    oICS = &ICS_PWMCpool[ICS_PWMC_Allocated];
    ICS_PWMC_Allocated = ICS_PWMC_Allocated + 1U;
  }
  else
  {
    oICS = (_DCICS_PWMC)MC_NULL;
  }
#endif
  
  oICS->pDParams_str = pICS_Params;
  oPWMnCurrFdbk->DerivedClass = (void*)oICS;

  oPWMnCurrFdbk->Methods_str.pPWMC_Init = &ICS_Init;
  oPWMnCurrFdbk->Methods_str.pPWMC_GetPhaseCurrents = &ICS_GetPhaseCurrents;
  oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOffPWM = &ICS_SwitchOffPWM;
  oPWMnCurrFdbk->Methods_str.pPWMC_SwitchOnPWM = &ICS_SwitchOnPWM;        
  oPWMnCurrFdbk->Methods_str.pPWMC_CurrentReadingCalibr = &ICS_CurrentReadingCalibration;         
  oPWMnCurrFdbk->Methods_str.pPWMC_TurnOnLowSides = &ICS_TurnOnLowSides;         
  oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect1 = &ICS_WriteFlexPWMRegisters;        
  oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect2 = &ICS_WriteFlexPWMRegisters; 
  oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect3 = &ICS_WriteFlexPWMRegisters;        
  oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect4 = &ICS_WriteFlexPWMRegisters;         
  oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect5 = &ICS_WriteFlexPWMRegisters;        
  oPWMnCurrFdbk->Methods_str.pPWMC_SetADCSampPointSect6 = &ICS_WriteFlexPWMRegisters; 
  oPWMnCurrFdbk->Methods_str.pPWMC_ExecRegularConv= &ICS_ExecRegularConv;
  oPWMnCurrFdbk->Methods_str.pPWMC_GetMultipleRegularConv = &ICS_GetMultipleRegularConv;
  oPWMnCurrFdbk->Methods_str.pPWMC_SetSamplingTime= &ICS_ADC_SetSamplingTime;
  oPWMnCurrFdbk->Methods_str.pPWMC_IsOverCurrentOccurred= &ICS_IsOverCurrentOccurred;

  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  return ((CICS_PWMC)oPWMnCurrFdbk);
}

/**
* @brief        It initializes ADC, CTU, DMA and FlexPWM for current reading 
*               and PWM generation.
*
* @param[in]    this related object of class CICS_PWMC
*
* @return       none
*
* @api
*
*/
static void ICS_Init(CPWMC this)
{
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS;  

  /* Initialiaze ADC Unit 0 */
  ICS_ADC_Init(pLocalDParams->hIaChannel.Adc_Unit, pLocalDParams->AdcPwrDownDelay, 
               (uint32)pLocalDParams->hIaChannel.hSamplingTime);
#ifndef _SPC560PXX_SMALL_
  /* Initialiaze ADC Unit 1 */
  ICS_ADC_Init(pLocalDParams->hIbChannel.Adc_Unit, pLocalDParams->AdcPwrDownDelay, 
                (uint32)pLocalDParams->hIbChannel.hSamplingTime);
#endif  
  /* Initialiaze CTU hardware */
  ICS_CTU_Init(this);

  /* FlexPWM Configuration */
  ICS_FlexPWMInit(this);
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}

/**
* @brief        It initializes  ADC and CTU for current reading.
*
* @param[in]    Adc_Unit ADC Unit (ADC_UNIT_0/ADC_UNIT_1)
* @param[in]    PwrDownDelay Power Down Exit Delay for ADC Units
* @param[in]    SamplingTime It is the value of Conversion timing (CTR register)
*
* @return       none
*
* @api
*
*/
static void ICS_ADC_Init(uint8 Adc_Unit, uint8 PwrDownDelay, uint32 SamplingTime)
{
  uint16 Timeout = 0xFFFFU;

  /* configure sampling time for ADC Unit 0 and 1 */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE32(ADC_CTR0_REG(Adc_Unit),(uint32)SamplingTime);
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE32(ADC_CTR0_REG(Adc_Unit),(uint32)SamplingTime);
  
  /* enter into power-down mode */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE32(ADC_MCR_REG(Adc_Unit),ADC_POWER_DOWN_EN);
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  while((((REG_READ32(ADC_MSR_REG(Adc_Unit)))&ADC_STATUS) != ADC_POWERDOWN_STATUS)&&
        (Timeout > (uint32)0U))
  {
      Timeout--;
  }
  /* If the ADC hardware is not entered in power down state: Error and return from function */
  if ( Timeout != (uint32)0U )
  {
      /* Write the configured power down exit delay value into PDEDR register */
      /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
      REG_WRITE32(ADC_PDEDR_REG(Adc_Unit),PwrDownDelay);
      /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
      REG_BIT_SET32(ADC_MCR_REG(Adc_Unit), 0x00000008);

      /* comes out from PowerDown */
      /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
      REG_BIT_CLEAR32(ADC_MCR_REG(Adc_Unit), ADC_POWER_DOWN);

      /* load the timeout counter */
      Timeout = 0xFFFFU;
      /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
      while((((REG_READ32(ADC_MSR_REG(Adc_Unit)))&ADC_STATUS) != ADC_IDLE_OFFSET_STATUS)&&
           (Timeout > (uint32)0U))
      {
         Timeout--;
      }
      /* If the ADC hardware is not entered in idle state: Error and return from function */
      if ( Timeout != (uint32)0U )
      {
          /* Main configuration register */
          /* ClkPrescaler, Auto Clock Off , Offset Refresh, Right align and overwrite */
          /* CTU control mode: disabled */
          REG_BIT_SET32(ADC_MCR_REG(Adc_Unit),(ADC_CLOCK_PRESCALER_DIV2 | ADC_AUTO_CLKOFF_DIS | 
                        ADC_OVERWRITE_EN | ADC_WRITE_LEFT_ALIGNED)
          /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
          );
      }
  }
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}

/**
* @brief  It initializes CTU hardware.
*
* @param[in]    this related object of class CICS_PWMC
*
* @return       none
*
* @api
*
*/
static void ICS_CTU_Init(CPWMC this)
{
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS; 

  /* Clear the CTUEFR_IFR flag */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE16(CTUV2_CTUEFR, REG_READ16(CTUV2_CTUEFR)); /* Clear the error flag */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE16(CTUV2_CTUIFR, REG_READ16(CTUV2_CTUIFR)); /* Clear the interrupt flag */

  /* TGS Counter reload value */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE16(CTUV2_TGSCRR, (uint16)0x0000);
  /* Stop TGS counter at this value */
  /* TGS Counter compare (Master reload occ.) = PWM freq  * PWMLoadFreq */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  REG_WRITE16(CTUV2_TGSCCR, (((((_CPWMC) this)->pParams_str->hPWMperiod)) * (ICS_CTU_PERIOD(pLocalDParams->PWMLoadFreq) + 1u))-1u);

  /* Write the Trigger Compare Registers in order to start the ADC command */
  /* T0CR - Trigger 0 compare register value 0 for ADC:  when a T0CR trigger events occurs, */
  /* it generates the ADC start of conversion request at the beginning of each PWM reload cycle. */
  /* So, the phase current is measured when the bottom transistor are swithed on. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_6 cast after bitwise operation */
  REG_WRITE16(CTUV2_TxCR(0U), 0x0001U);

#if defined ADC_USER_ENABLE
  /* Write the Trigger Compare Registers in order to start the ADC User command */
  /* T7CR - Trigger 7 compare register:  when a T7CR trigger events occurs, */
  /* it generates the ADC start of conversion request. */
  REG_WRITE16(CTUV2_TxCR(7U), (uint16)((REG_READ16(CTUV2_TGSCCR)*3U)/4U));

  /* Configure trigger handler: for ADC for regular conversion*/
  REG_WRITE32(CTUV2_THCR2, (uint32)(CTUV2_TRIGGERx_ADCE | CTUV2_TRIGGERx_E) << (uint32)0x18);
#endif /* ADC_USER_ENABLE */

#if defined BUS_VOLTAGE_MEASUREMENT
  /* Write the Trigger Compare Registers in order to start the ADC VBus command */
  /* T4CR - Trigger 5 compare register:  when a T5CR trigger events occurs, */
  /* it generates the ADC start of Vbus conversion. */
  REG_WRITE16(CTUV2_TxCR(5U), (uint16)((REG_READ16(CTUV2_TGSCCR))>>1U));

  /* Configure trigger handler: for ADC for VBus conversion */
  REG_BIT_SET32(CTUV2_THCR2, ((uint32)(CTUV2_TRIGGERx_ADCE | CTUV2_TRIGGERx_E) << (uint32)0x08));
#endif /* BUS_VOLTAGE_MEASUREMENT */

#if ( ((defined (AUX_STATE_OBSERVER_PLL)) || (defined (STATE_OBSERVER_PLL))) && \
      ((defined (RESOLVER)) || (defined (AUX_RESOLVER))) )
   if(pLocalDParams->PWMLoadFreq  > FLEXPWM_CTRL_LDFQ_EACH1)
   {
      /* Configure Trigger 4 to start ADC conversion with the period equal to PWM frequency */
      REG_WRITE16(CTUV2_TxCR(4U), ((((_CPWMC) this)->pParams_str->hPWMperiod))); 
   }
#endif 

  /* Configure trigger handler: for ADC for current sensing */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
#if defined _SPC56ELxx_
  REG_WRITE32(CTUV2_THCR1, (CTUV2_TRIGGERx_ADCE | 0x7F));
#endif
#if defined _SPC560Pxx_
  REG_WRITE32(CTUV2_THCR1, (CTUV2_TRIGGERx_ADCE | 0x02 |0x04| 0x10U));
#endif

  /* Program the command list control register */
  /* Address command register 1 (CLR1): Command List address 0 for current sensing */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE32(CTUV2_CLCR1, 0x0);

#if defined ADC_USER_ENABLE
  /* Address command register 10: Command List address for User ADC conversion */
  REG_WRITE32(CTUV2_CLCR2, CTUV2_CLCR2_T7_INDEX((uint32)9));
#endif /* ADC_USER_ENABLE */

#if defined BUS_VOLTAGE_MEASUREMENT
  /* Address command register 3 (CLR1): Command List address for User ADC conversion */
  REG_BIT_SET32(CTUV2_CLCR2, CTUV2_CLCR2_T5_INDEX((uint32)6));
#endif /* BUS_VOLTAGE_MEASUREMENT */

  /* Initialiaze DMA MUX */
#if defined _SPC56ELxx_
  /* DMA channel for CTU FIFO 1 - Current Sensing */
  DmaMux_SetChannelRouting(pLocalDParams->DmaChannel,(uint8)0x09,(uint8)STD_ON, (uint8)STD_OFF);
#if defined ADC_USER_ENABLE
  /* DMA channel for CTU FIFO 2 - ADC User conversion */
  DmaMux_SetChannelRouting(pLocalDParams->DmaUserChannel,(uint8)0x0A,(uint8)STD_ON, (uint8)STD_OFF);
#endif /* ADC_USER_ENABLE */
#if defined BUS_VOLTAGE_MEASUREMENT
  /* DMA channel for CTU FIFO 3 - ADC VBus conversion */
  DmaMux_SetChannelRouting(pLocalDParams->DmaVBusChannel,(uint8)0x0B,(uint8)STD_ON, (uint8)STD_OFF);
#endif /* BUS_VOLTAGE_MEASUREMENT */
#endif

#if defined _SPC560Pxx_
  /* DMA channel for CTU FIFO 1 - Current Sensing */
  DmaMux_SetChannelRouting(pLocalDParams->DmaChannel,(uint8)0x0B,(uint8)STD_ON, (uint8)STD_OFF);
#if defined ADC_USER_ENABLE
  /* DMA channel for CTU FIFO 2 - ADC User conversion */
  DmaMux_SetChannelRouting(pLocalDParams->DmaUserChannel,(uint8)0x0C,(uint8)STD_ON, (uint8)STD_OFF);
#endif /* ADC_USER_ENABLE */
#if defined BUS_VOLTAGE_MEASUREMENT
  /* DMA channel for CTU FIFO 3 - ADC VBus conversion */
  DmaMux_SetChannelRouting(pLocalDParams->DmaVBusChannel,(uint8)0x0D,(uint8)STD_ON, (uint8)STD_OFF);
#endif /* BUS_VOLTAGE_MEASUREMENT */
#endif /* _SPC56ELxx_ */

  /* Enable DMA for FIFO 1: Current Sensing is mandatory */
  REG_WRITE16(CTUV2_FDCR, 0x02);

#if defined ADC_USER_ENABLE
  /* Enable DMA for FIFO 2: User Conversion is optional */
  REG_BIT_SET16(CTUV2_FDCR, 0x04);
#endif /* ADC_USER_ENABLE */

#if defined BUS_VOLTAGE_MEASUREMENT
  /* Enable DMA for FIFO 3: VBus Conversion is optional */
  REG_BIT_SET16(CTUV2_FDCR, 0x08);
#endif /* BUS_VOLTAGE_MEASUREMENT */

  /* Specify which fifo has what threshold */
  /* Threshold value of the fifo. FIFOs generate an interrupt when                    */ 
  /* the number of elements in the FIFO exceeds the values present in                 */
  /* the threshold register. Threshold must have a maximum value of ((FIFO depth)-1). */
  /* FIFO 1 is configured as 2 depth, so threshold is configured: 1.                    */
  /* FIFO 2 is configured as 1 depth, so threshold is configured: 0.                    */
  /* FIFO 3 is configured as 1 depth, so threshold is configured: 0.                    */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE32(CTUV2_FTH, 0x00000100);

#ifndef _SPC560PXX_SMALL_
  /* Program the CTU ADC command register 0 in DUAL CONVERSION */
  /* - Command Interrupt Request: disabled,                    */
  /* - Not first command                                       */
  /* - Dual conversion mode                                    */
  /* - FIFO for ADC unit A/B: FIFO 1                           */ 
  /* - ADC unit A channel (hIaChannel) and ADC unit b channel (hIbChannel) */
  REG_WRITE16(CTUV2_CLRx(0), CTUV2_CLR_CIR_DISABLE | CTUV2_CLR_LC_NOT_LAST | CTUV2_CLR_CMS_DUAL |
              CTUV2_CLR_FIFO(1) | CTUV2_SU_CH_A(pLocalDParams->hIaChannel.hIChannel) | 
              CTUV2_SU_CH_B(pLocalDParams->hIbChannel.hIChannel)
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  );

  /* Program the CTU ADC command register Current Sensing             */
  /* for regular conversion                                           */
  /* - Command Interrupt Request: disabled,                           */
  /* - Last command                                                   */
  REG_WRITE16(CTUV2_CLRx(1), CTUV2_CLR_CIR_DISABLE | CTUV2_CLR_LC_LAST | CTUV2_CLR_CMS_SINGLE |
              CTUV2_CLR_FIFO(1));
#endif /* #ifndef _SPC560PXX_SMALL_ */

#ifdef _SPC560PXX_SMALL_
  /* Program the CTU ADC command register 0 in SINGLE CONVERSION   */
  /* - Command Interrupt Request: disabled,                        */
  /* - First command                                               */
  /* - Single conversion mode                                      */
  /* - FIFO for ADC: FIFO 1                                        */ 
  /* - ADC channel (hIaChannel)                                    */
  REG_WRITE16(CTUV2_CLRx(0), CTUV2_CLR_CIR_DISABLE | CTUV2_CLR_LC_NOT_LAST | CTUV2_CLR_CMS_SINGLE |
              CTUV2_CLR_FIFO(1) | CTUV2_SU_CH(pLocalDParams->hIaChannel.hIChannel) 
  );

  /* Program the CTU ADC command register 1 in SINGLE CONVERSION   */
  /* - Command Interrupt Request: disabled,                        */
  /* - First command                                               */
  /* - Single conversion mode                                      */
  /* - FIFO for ADC: FIFO 1                                        */ 
  /* - ADC channel (hIbChannel)                                    */
  REG_WRITE16(CTUV2_CLRx(1), CTUV2_CLR_CIR_DISABLE | CTUV2_CLR_LC_NOT_LAST | CTUV2_CLR_CMS_SINGLE |
              CTUV2_CLR_FIFO(1) | CTUV2_SU_CH(pLocalDParams->hIbChannel.hIChannel) 
  );

  REG_WRITE16(CTUV2_CLRx(2), CTUV2_CLR_CIR_DISABLE | CTUV2_CLR_LC_LAST | CTUV2_CLR_CMS_SINGLE |
              CTUV2_CLR_FIFO(1)
  );
#endif /* #ifdef _SPC560PXX_SMALL_ */
 
#if defined ADC_USER_ENABLE
  /* Program the CTU ADC command register USER REGULAR CONVERSION     */
  /* for regular conversion                                           */
  /* - Command Interrupt Request: disabled,                           */
  /* - First command                                                  */
  /* - Single conversion mode                                         */
  /* - FIFO for ADC: FIFO 2                                           */
  /* - ADC User channel                                               */
  REG_WRITE16(CTUV2_CLRx(9), CTUV2_CLR_CIR_DISABLE | CTUV2_CLR_LC_NOT_LAST | CTUV2_CLR_CMS_SINGLE |
              CTUV2_CLR_FIFO(2) | ADC_MOD_USER_CONV_0 | CTUV2_SU_CH(ADC_CH_USER_CONV_0)
  );

  /* Program the CTU ADC command register USER REGULAR CONVERSION     */
  /* for regular conversion                                           */
  /* - Command Interrupt Request: disabled,                           */
  /* - Last command                                                   */
  REG_WRITE16(CTUV2_CLRx(10), CTUV2_CLR_CIR_DISABLE | CTUV2_CLR_LC_LAST | CTUV2_CLR_CMS_SINGLE |
              CTUV2_CLR_FIFO(2));
#endif /* ADC_USER_ENABLE */

#if defined BUS_VOLTAGE_MEASUREMENT
  /* Program the CTU ADC command register VBUS CONVERSION             */
  /* for regular conversion                                           */
  /* - Command Interrupt Request: disabled,                           */
  /* - First command                                                  */
  /* - Single conversion mode                                         */
  /* - FIFO for ADC: FIFO 3                                           */
  /* - ADC User channel                                               */
  REG_WRITE16(CTUV2_CLRx(6), CTUV2_CLR_CIR_DISABLE | CTUV2_CLR_LC_NOT_LAST | CTUV2_CLR_CMS_SINGLE |
              CTUV2_CLR_FIFO(3) | VBUS_ADC_MOD | CTUV2_SU_CH(VBUS_ADC_CHANNEL)
  );

  /* Program the CTU ADC command register VBUS CONVERSION             */
  /* for regular conversion                                           */
  /* - Command Interrupt Request: disabled,                           */
  /* - Last command                                                   */
  REG_WRITE16(CTUV2_CLRx(7), CTUV2_CLR_CIR_DISABLE | CTUV2_CLR_LC_LAST | CTUV2_CLR_CMS_SINGLE |
              CTUV2_CLR_FIFO(3));
#endif /* BUS_VOLTAGE_MEASUREMENT */

#if ( ((defined (AUX_STATE_OBSERVER_PLL)) || (defined (STATE_OBSERVER_PLL))) && \
      ((defined (RESOLVER)) || (defined (AUX_RESOLVER))) )
   if(pLocalDParams->PWMLoadFreq  > FLEXPWM_CTRL_LDFQ_EACH1)
   {
    /* Configure Trigger 4 to start ADC conversion with the period equal to PWM frequency */
    REG_BIT_SET32(CTUV2_THCR2, ((uint32)(CTUV2_TRIGGERx_ADCE | 0x10U)));
   }
#endif

#if (!defined(RESOLVER) && !defined VIEW_RESOLVER_FEEDBACK)
 REG_WRITE16(CTUV2_CTUCR, CTUV2_CTUCR_GRE);
#endif

  /* Set-up DMA TCD descriptor */
  Dma_configure_channel(
  /* dma_channel */
  (uint8)pLocalDParams->DmaChannel,
  /* src_addr - FIFO 1 start addres (index) */
  (uint32 )CTUV2_FIFO_ADDR(1),
  /* src_transfer_size */
  DMA_DATA_TRANSFER_32_BIT,
  /* dest_transfer_size */
  DMA_DATA_TRANSFER_32_BIT,
  /* src_next_offset */
  (uint16)0,
  /* n_bytes_to_transfer */
  (uint16)((uint16)CTUV2_DMA_TRANSFER_WORD_SIZE*(2)),
  /* src_last_adj */
  (uint32)(0),
  /* dest_addr */
  (uint32)(CurrentCTUFifoBuffer),
  /* current_iteration_count */
  (uint16)1,
  /* dest_next_offset */
  (uint16)(CTUV2_DMA_TRANSFER_WORD_SIZE),
  /* dest_last_adj - adjustment will be done in the dma completion isr
     when all samples have been completed */
  (uint32)((sint32)-8),
  /* begin_iteration_count */
  (uint16)1,
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_7 cast from integer type to a other type or a wider integer type */  
  (uint8)DMA_TCD_INT_MAJOR);
 
#if defined ADC_USER_ENABLE
  /* Set-up DMA TCD descriptor */
  Dma_configure_channel(
  /* dma_channel */
  (uint8)pLocalDParams->DmaUserChannel,
  /* src_addr - FIFO 1 start addres (index) */
  (uint32 )CTUV2_FIFO_ADDR(2),
  /* src_transfer_size */
  DMA_DATA_TRANSFER_32_BIT,
  /* dest_transfer_size */
  DMA_DATA_TRANSFER_32_BIT,
  /* src_next_offset */
  (uint16)0,
  /* n_bytes_to_transfer */
  (uint16)((uint16)CTUV2_DMA_TRANSFER_WORD_SIZE),
  /* src_last_adj */
  (uint32)(0),
  /* dest_addr */
  (uint32)(RegularCTUFifoBuffer),
  /* current_iteration_count */
  (uint16)1,
  /* dest_next_offset */
  (uint16)(CTUV2_DMA_TRANSFER_WORD_SIZE),
  /* dest_last_adj - adjustment will be done in the dma completion isr
     when all samples have been completed */
  (uint32)((sint32)-4),
  /* begin_iteration_count */
  (uint16)1,
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_7 cast from integer type to a other type or a wider integer type */
#ifdef ADC_MULTIPLE_CONV_ENABLED
  (uint8)DMA_TCD_INT_MAJOR);
#else
  (uint8)0);
#endif
#endif /* ADC_USER_ENABLE */

#if defined BUS_VOLTAGE_MEASUREMENT
  /* Set-up DMA TCD descriptor */
  Dma_configure_channel(
  /* dma_channel */
  (uint8)pLocalDParams->DmaVBusChannel,
  /* src_addr - FIFO 1 start addres (index) */
  (uint32 )CTUV2_FIFO_ADDR(3),
  /* src_transfer_size */
  DMA_DATA_TRANSFER_32_BIT,
  /* dest_transfer_size */
  DMA_DATA_TRANSFER_32_BIT,
  /* src_next_offset */
  (uint16)0,
  /* n_bytes_to_transfer */
  (uint16)((uint16)CTUV2_DMA_TRANSFER_WORD_SIZE),
  /* src_last_adj */
  (uint32)(0),
  /* dest_addr */
  (uint32)(VBusCTUFifoBuffer),
  /* current_iteration_count */
  (uint16)1,
  /* dest_next_offset */
  (uint16)(CTUV2_DMA_TRANSFER_WORD_SIZE),
  /* dest_last_adj - adjustment will be done in the dma completion isr
     when all samples have been completed */
  (uint32)((sint32)-4),
  /* begin_iteration_count */
  (uint16)1,
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_7 cast from integer type to a other type or a wider integer type */
  (uint8)0);
#endif /* BUS_VOLTAGE_MEASUREMENT */

  /* Enable HW DMA request2 */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  LLM_Wr_DMA_SERQ(pLocalDParams->DmaChannel);

#if defined ADC_USER_ENABLE
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  LLM_Wr_DMA_SERQ(pLocalDParams->DmaUserChannel);
#endif

#if defined BUS_VOLTAGE_MEASUREMENT
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  LLM_Wr_DMA_SERQ(pLocalDParams->DmaVBusChannel);
#endif
}

/**
* @brief  Enable ADC units in CTU control mode. Enabled PWM Reload Rising edge Enable.
*
* @param[in]    this related object of class CICS_PWMC
*
* @return       none
*
* @api
*
*/
static void ICS_CTU_Start(CPWMC this)
{
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
   pDParams_t pLocalDParams = DCLASS_PARAMS;  
   VAR(uint32, AUTOMATIC) Timeout = 0xFFFFU;

   /* Configure ADC 0 in CTU control mode */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   REG_BIT_SET32(ADC_MCR_REG(pLocalDParams->hIaChannel.Adc_Unit), ADC_CTU_ENABLED);
#ifndef _SPC560PXX_SMALL_
   /* Configure ADC 1 in CTU control mode */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   REG_BIT_SET32(ADC_MCR_REG(pLocalDParams->hIbChannel.Adc_Unit), ADC_CTU_ENABLED);
#endif
   /* Program the accepted inputs of the CTUV2 module */
   /* Enabled PWM Reload Rising edge Enable */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   REG_WRITE32(CTUV2_TGISR, CTUV2_TGISER_RISING_EDGE_INPUT(0));

#if (defined(RESOLVER) || defined VIEW_RESOLVER_FEEDBACK)
   /* Update the double buffered registers */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   REG_WRITE16(CTUV2_CTUCR, CTUV2_CTUCR_GRE);
#endif

   /* Generating synchronization signal to copy double buffered input selection register */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   REG_WRITE16(CTUV2_CTUCR, CTUV2_CTUCR_TGSISR_RE);

   /* wait until the double buffered registers are updated */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   while(((REG_READ16(CTUV2_CTUCR)&CTUV2_CTUCR_TGSISR_RE) != (uint16)0) &&(Timeout > (uint32)0U))
   {
     Timeout--;
   }
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}

/**
* @brief  Disable CTU.
*
* @param[in]    this related object of class CICS_PWMC
*
* @return       none
*
* @api
*
*/
#if ((TEMPERATURE_READING == DISABLE) && (BUS_VOLTAGE_READING == DISABLE) && !defined (ADC_USER_ENABLE))
static void ICS_CTU_Stop(void)
{
   VAR(uint32, AUTOMATIC) Timeout = 0xFFFFU;

   /* Program the accepted inputs of the CTUV2 module */
   /* Enabled PWM Reload Rising edge Enable */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   REG_WRITE32(CTUV2_TGISR, 0x00000000UL);

   /* Generating synchronization signal to copy double buffered input selection register */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   REG_WRITE16(CTUV2_CTUCR, CTUV2_CTUCR_TGSISR_RE);

   /* wait until the double buffered registers are updated */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   while(((REG_READ16(CTUV2_CTUCR)&CTUV2_CTUCR_TGSISR_RE) != (uint16)0) &&(Timeout > (uint32)0U))
   {
     Timeout--;
   }
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}
#endif
/**
* @brief  It initializes FlexPWM peripheral for PWM generation
*
* @param[in]    this related object of class CICS_PWMC
*
* @return       none
*
* @api
*
*/
static void ICS_FlexPWMInit(CPWMC this)
{
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  uint8 submod_index,numChl;
  uint32 temp_reg;
  uint16 On_Value = (uint16)0x0000;
  uint16 Off_Value = (uint16)0x0000;

  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pLocalVars_Str->Half_PWMPeriod = ((((_CPWMC) this)->pParams_str->hPWMperiod) >> 0x1U);

  submod_index = (pLocalDParams->FlexPWMModule)<<FLEXPWM_NB;

  /* Disable output for PWM A/B/X */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_OUTEN, 0x0000U);

  /* Init SWCOUT register to reset value */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_SWCOUT,0x0000U);

  /* Init DTSRCSEL register to reset value */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_DTSRCSEL,0x0000U);

  /* Configure OCTRL polarity and Fault state bits for channel 1,2 3 */
  REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+(uint8)FLEXPWM_PHASE_U]+FLEXPWM_OCTRL), 
              ((pLocalDParams->hCh1Polarity) | (pLocalDParams->hCh1NPolarity))
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  );
  REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+(uint8)FLEXPWM_PHASE_V]+FLEXPWM_OCTRL), 
              (pLocalDParams->hCh2Polarity | pLocalDParams->hCh2NPolarity)
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  );
  REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+(uint8)FLEXPWM_PHASE_W]+FLEXPWM_OCTRL), 
              (pLocalDParams->hCh3Polarity | pLocalDParams->hCh3NPolarity)
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  );

  /* configure IDLE state (FAULT) for each channels */
  ICS_SetIdleStatePWM(submod_index+(uint8)FLEXPWM_PHASE_U,pLocalDParams->hCh1IdleState,(uint8)FLEX_PWMA);
  ICS_SetIdleStatePWM(submod_index+(uint8)FLEXPWM_PHASE_U,pLocalDParams->hCh1NIdleState,(uint8)FLEX_PWMB);
  ICS_SetIdleStatePWM(submod_index+(uint8)FLEXPWM_PHASE_V,pLocalDParams->hCh1IdleState,(uint8)FLEX_PWMA);
  ICS_SetIdleStatePWM(submod_index+(uint8)FLEXPWM_PHASE_V,pLocalDParams->hCh1NIdleState,(uint8)FLEX_PWMB);
  ICS_SetIdleStatePWM(submod_index+(uint8)FLEXPWM_PHASE_W,pLocalDParams->hCh1IdleState,(uint8)FLEX_PWMA);
  ICS_SetIdleStatePWM(submod_index+(uint8)FLEXPWM_PHASE_W,pLocalDParams->hCh1NIdleState,(uint8)FLEX_PWMB);

  /* Compute OFF and ON value (Duty cycle) */
  Off_Value = (uint16)(pLocalVars_Str->Half_PWMPeriod >> 0x1 );
  /*Off_Value = (uint16)2100;*/
  On_Value = (((uint16)0xFFFF-Off_Value) + (uint16)0x01);

  /* Clear LDOK bit for all Submodules */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_SET16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_MCTRL, (uint16)FLEXPWM_MCTRL_CLDOK_MASK);

  /* Configure FLEXPWM Sub-modules */
  for (numChl = (uint8)0; numChl < FLEXPWM_SUBMODULES_USED; numChl++)
  {
     /* Clear STS flag status register */
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_STS), (uint16)0x30FF); 

      /* Configure TCTRL (Output Trigger Control Register) */
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_TCTRL),(uint16)0x0000);
                
     if ((pLocalDParams->EmergencyStop)!= DISABLE)  
     {
        /* Configure DISMAP (all fields)  */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_DISMAP), (uint16)((uint32)0x11U << (pLocalDParams->FaultPIN)));
     }
     else
     {
        /* Configure DISMAP (all fields)  */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */ 
        REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_DISMAP), (uint16)0xF000);
     }
     /* Configure DeadTime (DTCNT0 register) */
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_DTCNT0),pLocalDParams->hDeadTime);

     /* Configure DeadTime (DTCNT1 register) */
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_DTCNT1), pLocalDParams->hDeadTime);

     /* Configure PWM period in Perfect Center aligned mode */
     /* set VAL1 to pLocalVars_Str->Half_PWMPeriod + half of deadtime for perfect center aligned */     
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_VAL1),
         (uint16)((uint32)(0xFFFFU)&((pLocalVars_Str->Half_PWMPeriod) + ((pLocalDParams->hDeadTime) >> 0x1U)))
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_5 cast from integer type to a other type or a wider integer type */
     );
     /* now compute the 2's complement for init */
     if(((pLocalVars_Str->Half_PWMPeriod) & (uint16)1) == (uint16)1)
     {
         /*odd period*/
         /* now compute the 2's complement for init */
         temp_reg = (~(uint32)pLocalVars_Str->Half_PWMPeriod) + (uint32)0x0001;
     }
     else
     {
         /*even period*/
         /* now compute the 2's complement for init and add 1 extra for period correction */
         temp_reg = (~(uint32)pLocalVars_Str->Half_PWMPeriod) + (uint32)0x0002;
     }
     temp_reg &=(uint32)0xFFFF;
     /* (add a half of deadtime for perfect center aligned) */
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_INIT), 
         (uint16)((uint16)temp_reg + (uint16)((pLocalDParams->hDeadTime) >> 0x1))
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     );
     /* set VAL0 to half of deadtime for perfect center aligned*/
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_VAL0), (uint16)((pLocalDParams->hDeadTime) >> 0x1));

     /* if submodule 0 (of FLEXPWM_0 and FLEXPWM_1) */
     if(((submod_index+numChl) == (uint8)0) || ((submod_index+numChl) == (uint8)4))
     {
         /* Configure CTRL2 register */
         REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_CTRL2), 
                      FLEXPWM_CTRL2_CLK_SEL_IPBUS | FLEXPWM_CTRL2_RELOAD_SEL_LOCAL_RELOAD | 
                      FLEXPWM_CTRL2_FORCE_SEL_FORCE | FLEXPWM_CTRL2_INIT_SEL_LOCAL_SYNC |
                      FLEXPWM_CTRL2_FRCEN_DIS |FLEXPWM_CTRL2_INDEP_COMPLEMENTARY |
                      FLEXPWM_CTRL2_WAITEN_DIS | FLEXPWM_CTRL2_DBGEN_DIS
         /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
         );
     }
     else /* submodule 1/2/3 */
     {
         /* Configure CTRL2 */
         REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_CTRL2), 
                      FLEXPWM_CTRL2_CLK_SEL_IPBUS | FLEXPWM_CTRL2_RELOAD_SEL_LOCAL_RELOAD |
                      FLEXPWM_CTRL2_FORCE_SEL_MASTER_FORCE | FLEXPWM_CTRL2_INIT_SEL_MASTER_SYNC |
                      FLEXPWM_CTRL2_FRCEN_DIS |FLEXPWM_CTRL2_INDEP_COMPLEMENTARY |
                      FLEXPWM_CTRL2_WAITEN_DIS | FLEXPWM_CTRL2_DBGEN_DIS
         /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
         );
     }
     /* Configure CTRL1 register */
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_CTRL1),
                  (pLocalDParams->FlexPWM_Clock_Divider) | FLEXPWM_CTRL_FULL_EN |
                  (pLocalDParams->PWMLoadFreq)
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     );

    /* Write On value to coresspondiong submodule register VAL2 */
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_VAL2), On_Value);
    /* Write Off value to coresponding submodule register VAL3 */
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_VAL3), Off_Value);
  }

  if ((pLocalDParams->EmergencyStop)!= DISABLE)  
  {
     /* Clear FCTRLregister, enable A logic 1 on the fault input FAUL0 Pin*/
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_FCTRL),(uint16)((uint32)(pLocalDParams->FaultPolarity) << (pLocalDParams->FaultPIN)));

     /* Clear FSTS.FFLAG flag status - w1c */
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_FSTS), (uint16)0x000F);
  }

  /* SET LDOK bit for all Submodules */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_SET16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_MCTRL, (uint16)FLEXPWM_MCTRL_LDOK_MASK);

  /* PWM generator enabled */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_SET16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_MCTRL, FLEXPWM_SUB_RUN_MASK);
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}

/**
* @brief  It configure the PWM channel idle state.
*
* @param[in]    submodule FlexPWM submodule
* @param[in]    channel PWM channel
* @param[in]    channel_mask PWM channel mask:channel A (FLEX_PWMA), channel B (FLEX_PWMB)
*
* @return       none
*
* @api
*
*/
static void ICS_SetIdleStatePWM(uint8 submodule, uint16 channel, uint8 channel_mask)
{
  uint16 idle,Pol,tmp1;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  Pol = REG_READ16(FlexPWM_SubFlexPwm_Regs[submodule]+FLEXPWM_OCTRL);
  tmp1 = (uint16)((uint32)FLEXPWM_OCTRL_POLX << (uint16)channel_mask);
  if( (uint16)0 == ((uint16)Pol & (uint16)tmp1)) {
     if (channel == (uint16)FLEXPWM_IDLE_STATE_HIGH)  {
        idle = 1U; }
     else {
        idle = 0U; }
  }else
  {
     if (channel == (uint16)FLEXPWM_IDLE_STATE_HIGH)  {
       idle = 0U; }
     else {
       idle = 1U; }
  }
  if(channel_mask == (uint8)FLEX_PWMA) {
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submodule]+FLEXPWM_OCTRL), (uint16)(Pol |((uint32)idle << 0x04) ));
  }else {
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submodule]+FLEXPWM_OCTRL), (uint16)(Pol |((uint32)idle << 0x02U) ));
  }
}

/**
* @brief  It stores into 'this' object variables the voltage present on Ia and 
*         Ib current feedback analog channels when no current is flowin into the
*         motor
*
* @param[in]    this related object of class CICS_PWMC
*
* @return       none
*
* @api
*
*/
static void ICS_CurrentReadingCalibration(CPWMC this)
{
  static uint8 passed = 0;
  
  if(passed == 0)
  {
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  DVars_t *pLocalVars_Str = &DCLASS_VARS;  
  uint8 bIndex;
  uint32 wPhaseAOffset=0u, wPhaseBOffset=0u; 

  /* Program the Injected Conversion Mask register */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_SET32(ADC_JCMR0_REG(pLocalDParams->hIaChannel.Adc_Unit), (uint32)0x1<<(pLocalDParams->hIaChannel.hIChannel));
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_SET32(ADC_JCMR0_REG(pLocalDParams->hIbChannel.Adc_Unit), (uint32)0x1<<(pLocalDParams->hIbChannel.hIChannel));

  /* ADC Channel used for current reading are read 
  in order to get zero currents ADC values*/ 
  for(bIndex = NB_CONVERSIONS; bIndex !=0u; bIndex--)
  {
    /* Clear the ADC JEOC pending flag */
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    REG_WRITE32(ADC_ISR_REG(pLocalDParams->hIaChannel.Adc_Unit), (ADC_ISR_END_CHAIN_INJ_CONV_CLEAN | ADC_ISR_END_CHANNEL_INJ_CONV_CLEAN));
#ifndef _SPC560PXX_SMALL_
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    REG_WRITE32(ADC_ISR_REG(pLocalDParams->hIbChannel.Adc_Unit), (ADC_ISR_END_CHAIN_INJ_CONV_CLEAN | ADC_ISR_END_CHANNEL_INJ_CONV_CLEAN));
#endif  
    /* Start Injected Convertion */
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    REG_BIT_SET32(ADC_MCR_REG(pLocalDParams->hIaChannel.Adc_Unit), ADC_INJ_START_CONV_EN);
#ifndef _SPC560PXX_SMALL_
    REG_BIT_SET32(ADC_MCR_REG(pLocalDParams->hIbChannel.Adc_Unit), ADC_INJ_START_CONV_EN);

    while((0U == REG_BIT_GET32(ADC_ISR_REG(pLocalDParams->hIaChannel.Adc_Unit),ADC_ISR_END_CHAIN_INJ_CONV)) &&  
          /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
          /* @violates @ref ICS_PWMnCurrFdbkClass_REF_8 side effects on right hand of logical operator: '&&' */
          (0U == REG_BIT_GET32(ADC_ISR_REG(pLocalDParams->hIbChannel.Adc_Unit),ADC_ISR_END_CHAIN_INJ_CONV))) 
    { }
#endif
#ifdef _SPC560PXX_SMALL_
    while(0U == REG_BIT_GET32(ADC_ISR_REG(pLocalDParams->hIaChannel.Adc_Unit),ADC_ISR_END_CHAIN_INJ_CONV))
    { }
#endif  
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    wPhaseAOffset += (uint16)(REG_READ32(ADC_CH_DATA_REG(pLocalDParams->hIaChannel.Adc_Unit,(pLocalDParams->hIaChannel.hIChannel))) );
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    wPhaseBOffset += (uint16)(REG_READ32(ADC_CH_DATA_REG(pLocalDParams->hIbChannel.Adc_Unit,(pLocalDParams->hIbChannel.hIChannel))) );
  }

  /* Clear the ADC JEOC pending flag */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE32(ADC_ISR_REG(pLocalDParams->hIaChannel.Adc_Unit), (ADC_ISR_END_CHAIN_INJ_CONV_CLEAN | ADC_ISR_END_CHANNEL_INJ_CONV_CLEAN));
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE32(ADC_ISR_REG(pLocalDParams->hIbChannel.Adc_Unit), (ADC_ISR_END_CHAIN_INJ_CONV_CLEAN | ADC_ISR_END_CHANNEL_INJ_CONV_CLEAN));

  /* Clear the Injected Conversion Mask register */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_CLEAR32(ADC_JCMR0_REG(pLocalDParams->hIaChannel.Adc_Unit), (uint32)0x1<<(pLocalDParams->hIaChannel.hIChannel));
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_CLEAR32(ADC_JCMR0_REG(pLocalDParams->hIbChannel.Adc_Unit), (uint32)0x1<<(pLocalDParams->hIbChannel.hIChannel));

  pLocalVars_Str->hPhaseAOffset = (uint16)(wPhaseAOffset>>4);
  pLocalVars_Str->hPhaseBOffset = (uint16)(wPhaseBOffset>>4);

  passed = 1;
  }

}

/**
* @brief  It computes and return latest converted motor phase currents motor
*
* @param[in]    this related object of class CICS_PWMC
*
* @return       Ia and Ib current in Curr_Components format
*
* @api
*
*/
static void ICS_GetPhaseCurrents(CPWMC this,Curr_Components* pStator_Currents)
{

  sint32 wAux;
  uint16 hReg;
  sint16 qI_Component_x;
  sint16 qI_Component_y;
  uint8 submod_index;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS; 
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  submod_index = (pLocalDParams->FlexPWMModule)<<FLEXPWM_NB;

  /* Clear Reload flag bit necessary to detect FOC duration SW error */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE16(FlexPWM_SubFlexPwm_Regs[submod_index+1]+FLEXPWM_STS, FLEXPWM_STS_RF);

  /* First Phase current = Fifo[0] - hPhaseAOffset */
  hReg = (uint16) ((CurrentCTUFifoBuffer[0])<<1);
  wAux = (sint32)(hReg)-(sint32)(pLocalVars_Str->hPhaseAOffset);
#ifdef INVERTER_LEGS_SENSING
  wAux =- wAux;
#endif
 
 /* Saturation of Ia */
  if (wAux < S16_MIN)
  {
    qI_Component_x= S16_MIN;
  }  
  else  if (wAux > S16_MAX)
  { 
    qI_Component_x= S16_MAX;
  }
  else
  {
    qI_Component_x = (sint16)wAux;
  }
  
  /* Second Phase current = Fifo[1] - hPhaseBOffset */  
  hReg = (uint16) ((CurrentCTUFifoBuffer[1])<<1);
  wAux = (sint32)(hReg)-(sint32)(pLocalVars_Str->hPhaseBOffset);
#ifdef INVERTER_LEGS_SENSING
  wAux =- wAux;
#endif

#ifdef CURR_MEASURED_ON_UW_PHASES
  pStator_Currents->qI_Component1 = qI_Component_x;
  wAux = - pStator_Currents->qI_Component1 - wAux;
#endif

#ifdef CURR_MEASURED_ON_VW_PHASES
  pStator_Currents->qI_Component2 = qI_Component_x;
  wAux = - pStator_Currents->qI_Component2 - wAux;
#endif

  /* Saturation of Ib */
  if (wAux < S16_MIN)
  {
    qI_Component_y= S16_MIN;
  }  
  else  if (wAux > S16_MAX)
  { 
    qI_Component_y= S16_MAX;
  }
  else
  {
    qI_Component_y = (sint16)wAux;
  }
  
#ifdef CURR_MEASURED_ON_UV_PHASES
  pStator_Currents->qI_Component1 = qI_Component_x;
  pStator_Currents->qI_Component2 = qI_Component_y;
#endif

#ifdef CURR_MEASURED_ON_VW_PHASES
  pStator_Currents->qI_Component1 = qI_Component_y;
#endif

#ifdef CURR_MEASURED_ON_UW_PHASES
  pStator_Currents->qI_Component2 = qI_Component_y;
#endif
}

/**
* @brief  It turns on low sides switches. This function is intended to be 
*         used for charging boot capacitors of driving section. It has to be 
*         called each motor start-up when using high voltage drivers
*
* @param[in]    this related object of class CICS_PWMC
*
* @return   none
*
* @api
*
*/
static void ICS_TurnOnLowSides(CPWMC this)
{    

  uint8 submod_index;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS;

  submod_index = (pLocalDParams->FlexPWMModule)<<FLEXPWM_NB;

  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  ((_CPWMC) this)->Vars_str.hCntPhA = 0u;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  ((_CPWMC) this)->Vars_str.hCntPhB = 0u;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  ((_CPWMC) this)->Vars_str.hCntPhC = 0u;
    

   /* Clear Reload flag bir necessary to detect FOC duration SW error */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
   REG_WRITE16(FlexPWM_SubFlexPwm_Regs[submod_index+1]+FLEXPWM_STS, FLEXPWM_STS_RF);

  if(ICS_WriteFlexPWMRegisters(this) == MC_NO_ERROR)
  {
      /* Clear Reload flag bir necessary to detect FOC duration SW error */
      /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
      REG_WRITE16(FlexPWM_SubFlexPwm_Regs[submod_index+1]+FLEXPWM_STS, FLEXPWM_STS_RF);

      /* Enable Output */
      if ((pLocalDParams->LowSideOutputs)== LS_PWM_TIMER)
      {
          /* Enable output for High side and Low side*/
          /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
          REG_WRITE16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_OUTEN, FLEXPWM_SUB_MOD_EN_OUT_ALL);
      }
      else
      {
          /* Enable output  only for high side */
          /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
          REG_WRITE16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_OUTEN, FLEXPWM_SUB_MOD_EN_OUT_LS);
      }

      if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
      {

        pal_lld_setpad(PORT_D, MC_PIN_PWM_H1);
        pal_lld_setpad(PORT_G, MC_PIN_PWM_H2);
        pal_lld_setpad(PORT_G, MC_PIN_PWM_H3);

      }
  }
  return;
}

/**
* @brief  It enables PWM generation on the proper FlexPWM peripheral
*
* @param[in]    this related object of class CICS_PWMC
*
* @return   none
*
* @api
*
*/
static void ICS_SwitchOnPWM(CPWMC this)
{

  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  
  /* restore from idle state*/
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_CLEAR16((FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_FSTS),(uint16)0x1000);

  /* Clear FSTS.FFLAG flag status - w1c */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_WRITE16((FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_FSTS), (uint16)0x000F);

  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    pal_lld_setpad(PORT_D, MC_PIN_PWM_H1);
    pal_lld_setpad(PORT_G, MC_PIN_PWM_H2);
    pal_lld_setpad(PORT_G, MC_PIN_PWM_H3);
  }

  /* Enable Output */
  if ((pLocalDParams->LowSideOutputs)== LS_PWM_TIMER)
  {
      /* Enable output for High side and Low side*/
      /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
      REG_WRITE16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_OUTEN, FLEXPWM_SUB_MOD_EN_OUT_ALL);
  }
  else
  {
      /* Enable output  only for high side */
      /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
      REG_WRITE16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_OUTEN, FLEXPWM_SUB_MOD_EN_OUT_LS);
  }

 
ICS_CTU_Start(this);

  return;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}

/**
* @brief  It disables PWM generation on the proper FlexPWM peripheral
*
* @param[in]    this related object of class CICS_PWMC
*
* @return   none
*
* @api
*
*/
static void ICS_SwitchOffPWM(CPWMC this)
{ 
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS;

#if ((TEMPERATURE_READING == DISABLE) && (BUS_VOLTAGE_READING == DISABLE) && !defined (ADC_USER_ENABLE))
  ICS_CTU_Stop();
#endif


  /* Set Idle state */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_SET16((FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_FSTS),(uint16)0x1000);
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_SET16((FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_FSTS),(uint16)0x100F);

  if ((pLocalDParams->LowSideOutputs)== ES_GPIO)
  {
    pal_lld_clearpad(PORT_D, MC_PIN_PWM_H1);
    pal_lld_clearpad(PORT_G, MC_PIN_PWM_H2);
    pal_lld_clearpad(PORT_G, MC_PIN_PWM_H3);
  }
  return; 
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}

/**
* @brief  Update PWM duty cycles
*
* @param[in]    this related object of class CICS_PWMC
*
* @return   FOC status (MC_NO_ERROR/MC_FOC_DURATION)
*
* @api
*
*/
static uint16 ICS_WriteFlexPWMRegisters(CPWMC this)
{
  uint8 submod_index,numChl;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  DVars_t *pLocalVars_Str = &DCLASS_VARS;
  uint16 hAux = MC_NO_ERROR;

  uint16 On_Value = (uint16)0x0000;
  uint16 Off_Value = (uint16)0x0000;
  uint16 LocalPhase[3];
  uint8 index_phase = 0;

  submod_index = (pLocalDParams->FlexPWMModule)<<FLEXPWM_NB;

  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  LocalPhase[0] = (uint16)(((_CPWMC) this)->Vars_str.hCntPhA);
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  LocalPhase[1] = (uint16)(((_CPWMC) this)->Vars_str.hCntPhB);
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  LocalPhase[2] = (uint16)(((_CPWMC) this)->Vars_str.hCntPhC);
  
  /* Configure FLEXPWM Sub-modules */
  for (numChl = (uint8)FLEXPWM_SUB_MOD_INDEX; numChl < FLEXPWM_SUBMODULES_USED; numChl++)
  { 
    /* if duty is 0% or 100% */ 
    if ( ((uint16)0x0 == (uint16)(LocalPhase[index_phase])) || 
         ((uint16)pLocalVars_Str->Half_PWMPeriod == (uint16)(LocalPhase[index_phase])))
    {
       /* pulse starts from INIT */
       /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
       On_Value = REG_READ16(FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_INIT);
       if ((uint16)0x0 == (uint16)(LocalPhase[index_phase]))
       {
           Off_Value = On_Value;
       }else{
           /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
           Off_Value = REG_READ16(FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_VAL1) + (uint16)0x0001U;
       }
    }
    else
    {
        /* Compute OFF and ON value (Duty cycle) */
        Off_Value = (uint16)LocalPhase[index_phase];
      On_Value = (((uint16)~Off_Value) + (uint16)0x01);
    }
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    if( (uint16)0x0 != (REG_READ16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_MCTRL) &
                       (uint16)(((uint32)FLEXPWM_MCTRL_SUBMOD_0_LDOK_MASK) << (uint32)(submod_index+(uint32)numChl))))
    {
        /* clear LDOK */
        REG_BIT_SET16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_MCTRL, 
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
                     (uint16)(((uint32)FLEXPWM_MCTRL_SUBMOD_0_CLDOK_MASK) <<(uint32)(submod_index+(uint32)numChl)));
    }

    /* PWM_VAL2 should never exceed PWM_INIT value (in two complements) */
    if (On_Value < REG_READ16(FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_INIT) )
    {
      On_Value = REG_READ16(FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_INIT);
      Off_Value = (((uint16)~On_Value) + (uint16)0x01);
    }
    /* Write On value to corresponding submodule register VAL2 */
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_VAL2), On_Value);
    /* Write Off value to corresponding submodule register VAL3 */
    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    REG_WRITE16((FlexPWM_SubFlexPwm_Regs[submod_index+numChl]+FLEXPWM_VAL3), Off_Value);
    
    /* Increase index */
    index_phase = index_phase + 1U;
  }

    /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
    if((REG_READ16(FlexPWM_SubFlexPwm_Regs[submod_index+1]+FLEXPWM_STS) & FLEXPWM_STS_RF) == FLEXPWM_STS_RF)
    {
       hAux = MC_FOC_DURATION;
    }

  /* SET LDOK bit for all Submodules */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  REG_BIT_SET16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_MCTRL, (uint16)FLEXPWM_MCTRL_LDOK_MASK);

  return hAux;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}

/**
* @brief  Execute a regular conversion using ADC. 
*         The function is not re-entrant (can't executed twice at the same time)
*         
* @param[in]    this related object of class CICS_PWMC
*               bChannel ADC channel
*               ADC_Unit ADC unit
*
* @return It returns the converted value if the function is called properly or 0xFFFFU in case of error
*
* @api
*
*/
static uint16 ICS_ExecRegularConv(CPWMC this, uint8 bChannel, uint8 ADC_Unit)
{
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
   pDVars_t pDVars_str = &DCLASS_VARS;
   uint16_t data_temp = 0xFFFFU;

#if defined BUS_VOLTAGE_MEASUREMENT || defined ADC_USER_ENABLE
   /* if the Motor is running (means that ADC is in CTU mode) we read the ADC result directly from the CTU FIFO */
   if ((REG_READ32(ADC_MCR_REG(ADC_Unit)) & ADC_CTU_ENABLED) == ADC_CTU_ENABLED)
   {
#if defined BUS_VOLTAGE_MEASUREMENT
       if ((bChannel == (uint8)VBUS_ADC_CHANNEL) && (ADC_Unit == (uint8)VBUS_ADC_MODULE))
       {
           data_temp =  (uint16)(VBusCTUFifoBuffer[0]);
           data_temp = data_temp << 1U;
       }
#endif /* BUS_VOLTAGE_MEASUREMENT */

#if defined ADC_USER_ENABLE
       if ((bChannel == (uint8)ADC_USER_CH) && (ADC_Unit == (uint8)ADC_USER_MODULE))
       {
           data_temp =  (uint16)(RegularCTUFifoBuffer[0]);
#if defined _SPC56ELxx_
           data_temp = data_temp >> 3U;
#endif
#if defined _SPC560Pxx_
           data_temp = data_temp >> 5U;
#endif
       }
#endif /* ADC_USER_ENABLE */
   }
   /* otherwise we start an injected ADC conversion */
   else
   {  
        /* Program the Injected Conversion Mask register */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_BIT_SET32(ADC_JCMR0_REG(ADC_Unit), (uint32)0x1<<(bChannel));

        /* Clear the ADC JEOC pending flag */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_WRITE32(ADC_ISR_REG(ADC_Unit), (ADC_ISR_END_CHAIN_INJ_CONV_CLEAN | ADC_ISR_END_CHANNEL_INJ_CONV_CLEAN));

        /* Start Injected Convertion */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_BIT_SET32(ADC_MCR_REG(ADC_Unit), ADC_INJ_START_CONV_EN);
        while(0U == REG_BIT_GET32(ADC_ISR_REG(ADC_Unit),ADC_ISR_END_CHAIN_INJ_CONV))
        { }

        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        data_temp = (uint16)(REG_READ32(ADC_CH_DATA_REG(ADC_Unit,(bChannel))));
        /* for BUS_VOLTAGE_MEASUREMENT no shift to do because the data is already left aligned */
    #if defined ADC_USER_ENABLE
        if ((bChannel == (uint8)ADC_USER_CH) && (ADC_Unit == (uint8)ADC_USER_MODULE))
        {
      #if defined _SPC56ELxx_
            data_temp = data_temp >> 4U;
      #endif
      #if defined _SPC560Pxx_
            data_temp = data_temp >> 6U;
      #endif
        }
    #endif /* ADC_USER_ENABLE */

        /* Clear the ADC JEOC pending flag */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_WRITE32(ADC_ISR_REG(ADC_Unit), (ADC_ISR_END_CHAIN_INJ_CONV_CLEAN | ADC_ISR_END_CHANNEL_INJ_CONV_CLEAN));
        /* Clear the Injected Conversion Mask register */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_BIT_CLEAR32(ADC_JCMR0_REG(ADC_Unit), (uint32)0x1<<(bChannel));
   }
#endif /* BUS_VOLTAGE_MEASUREMENT defined ADC_USER_ENABLE */

   /* save the data and return */
   pDVars_str->hRegConv = data_temp;
   return (uint16)(pDVars_str->hRegConv);
   /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}


/**
* @brief  Get the user ADC_channel regular conversion. The ADC conversion data 
*         are stored in buffer with size equal to Number of conversion. 
*         The ADC User conversion shall have a period of PWMfreq/Number of 
*          conversion.
*         
*         
* @param[in]    this related object of class CICS_PWMC
* @param[in]    ADC channel              
*               
*
* @return It returns the converted value if the function is called properly or 0xFFFFU in case of error
*
* @api
*
*/
static uint16 ICS_GetMultipleRegularConv(CPWMC this, uint8 bChannel)
{
   pDVars_t pDVars_str = &DCLASS_VARS;
   uint16_t data_temp = 0xFFFFU;

#ifdef ADC_USER_ENABLE
   uint8_t adc_circ_channel = 0U;
   uint8_t adc_circ_unit = 0U;

#ifdef _SPC560PXX_SMALL_
   if ((REG_READ32(ADC_MCR_REG(ADC_UNIT_0)) & ADC_CTU_ENABLED) == ADC_CTU_ENABLED)
#else
   if (((REG_READ32(ADC_MCR_REG(ADC_UNIT_0)) & ADC_CTU_ENABLED) == ADC_CTU_ENABLED) ||
       ((REG_READ32(ADC_MCR_REG(ADC_UNIT_1)) & ADC_CTU_ENABLED) == ADC_CTU_ENABLED))
#endif  
   {
#if defined ADC_MULTIPLE_CONV_ENABLED
       data_temp = ADC_User_multiple_buffer[bChannel];
#else
       data_temp =  (uint16)(RegularCTUFifoBuffer[0]);
#endif /* ADC_MULTIPLE_CONV_ENABLED */
#if defined _SPC56ELxx_
       data_temp = data_temp >> 3U;
#endif
#if defined _SPC560Pxx_
       data_temp = data_temp >> 5U;
#endif
   }
   /* otherwise we start an injected ADC conversion */
   else
   {
        adc_circ_unit = ((ICS_ADC_Curr[bChannel].Adc_Unit) >> 5U);
        adc_circ_channel= (ICS_ADC_Curr[bChannel].hIChannel);

        /* Program the Injected Conversion Mask register */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_BIT_SET32(ADC_JCMR0_REG(adc_circ_unit), (uint32)0x1<<(adc_circ_channel));

        /* Clear the ADC JEOC pending flag */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_WRITE32(ADC_ISR_REG(adc_circ_unit), (ADC_ISR_END_CHAIN_INJ_CONV_CLEAN | ADC_ISR_END_CHANNEL_INJ_CONV_CLEAN));

        /* Start Injected Convertion */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_BIT_SET32(ADC_MCR_REG(adc_circ_unit), ADC_INJ_START_CONV_EN);
        while(0U == REG_BIT_GET32(ADC_ISR_REG(adc_circ_unit),ADC_ISR_END_CHAIN_INJ_CONV))
        { }

        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        data_temp = (uint16)(REG_READ32(ADC_CH_DATA_REG(adc_circ_unit,(adc_circ_channel))));
        #if defined _SPC56ELxx_
        data_temp = data_temp >> 4U;
        #endif
        #if defined _SPC560Pxx_
        data_temp = data_temp >> 6U;
        #endif

        /* Clear the ADC JEOC pending flag */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_WRITE32(ADC_ISR_REG(adc_circ_unit), (ADC_ISR_END_CHAIN_INJ_CONV_CLEAN | ADC_ISR_END_CHANNEL_INJ_CONV_CLEAN));
        /* Clear the Injected Conversion Mask register */
        /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
        REG_BIT_CLEAR32(ADC_JCMR0_REG(adc_circ_unit), (uint32)0x1<<(adc_circ_channel));
    }
#endif
   return (uint16)(data_temp);
}

/**
* @brief  It sets the specified sampling time for the specified ADC channel:
*         - ADC channel 10 and 15 (ADC Unit 0 and ADC Unit 1)
*         - ADC chanell 0-8 and 11-14) (ADC Unit 0 and ADC Unit 1)
*
* @param[in]    this related object of class CICS_PWMC
* @param[in]    ADConv_struct ADC channel and sampling time
*
* @return   none
*
* @api
*
*/
static void ICS_ADC_SetSamplingTime(ADConv_t ADConv_struct)
{
}

/**
* @brief  It is used to check if an overcurrent occurred since last call. 
*
* @param[in]    this related object of class CICS_PWMC
*
* @return   It returns MC_BREAK_IN whether an overcurrent has been 
*           detected since last method call, MC_NO_FAULTS otherwise
*
* @api
*
*/
static uint16 ICS_IsOverCurrentOccurred(CPWMC this)
{
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_2 place reliance on undefined or unspecified behavior. */
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_3 Cast from pointer to pointer. */
  pDParams_t pLocalDParams = DCLASS_PARAMS;
  uint16 retVal = MC_NO_FAULTS;

  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
  if((REG_READ16(FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_FSTS) & 0x000FU) > 0x0U)
  {
     retVal = MC_BREAK_IN;
     /* Clear FSTS.FFLAG flag status - w1c */
     /* @violates @ref ICS_PWMnCurrFdbkClass_REF_1 cast from unsigned int to pointer */
     REG_WRITE16((FlexPWM_Regs[pLocalDParams->FlexPWMModule]+FLEXPWM_FSTS), (uint16)0x000F);
  }

  return retVal;
  /* @violates @ref ICS_PWMnCurrFdbkClass_REF_4 A pointer parameter in a function prototype 
   * should be declared as pointer to constif the pointer is not used to modify the addressed object */
}

