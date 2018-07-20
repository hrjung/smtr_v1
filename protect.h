/*
 * protect.h
 *
 *  Created on: 2017. 3. 17.
 *      Author: hrjung
 */

#ifndef PROTECT_H_
#define PROTECT_H_

//#if 1 //def TEST_MOTOR
//// for 220V power source
//#define DC_VOLTAGE_INIT_RELAY_OFF		(100.0)
//#define DC_VOLTAGE_INIT_RELAY_ON		(186.0)
//#define DC_VOLTAGE_OVER_TRIP_LEVEL		(404.5)
//
//#define DC_VOLTAGE_START_REGEN_LEVEL 	(373.4)
//#define DC_VOLTAGE_END_REGEN_LEVEL 		(357.8)
//#else
//// for 380 power source
//#define DC_VOLTAGE_INIT_RELAY_OFF		(100.0)
//#define DC_VOLTAGE_INIT_RELAY_ON		(322.4)
//#define DC_VOLTAGE_OVER_TRIP_LEVEL		(698.6)
//
//#define DC_VOLTAGE_START_REGEN_LEVEL 	(644.9)
//#define DC_VOLTAGE_END_REGEN_LEVEL 		(618.0)
//#endif

////////////////////////////////////////////////

extern int OVL_isOverloadTripEnabled(void);
extern void OVL_enbleOverloadTrip(uint16_t enable);

extern int OVL_setWarningLevel(uint16_t level);
extern int OVL_setTripLevel(uint16_t level);
extern int OVL_setWarningTime(uint16_t dur);
extern int OVL_setTripTime(uint16_t dur);

extern int REGEN_setRegenResistance(float_t resist);
extern int REGEN_setRegenResistancePower(uint16_t power);
extern int REGEN_setRegenThermal(float_t value);
extern int REGEN_setRegenVoltReduction(uint16_t value);

//extern int OSC_setDampImpact(int value);
//extern int OSC_setDampFilter(int value);

//extern int ETH_isElecThermalEnabled(void);
//extern void ETH_enbleElecThermal(void);
//extern void ETH_disableElecThermal(void);
//
//extern int ETH_setProtectMaxLimit(int level);
//extern int ETH_setProtectNormalLimit(int level);

//extern int ETH_processElectroThermalProtection(int cur_level);
//extern void OVL_processOverloadWarning(int cur_level);
extern int OVL_isOverloadTrip(float_t cur_level);

extern void PROT_init(int input);
extern int processProtection(void);
#endif /* PROTECT_H_ */
