/*
 * err_trip.h
 *
 *  Created on: 2017. 7. 27.
 *      Author: hrjung
 */

#ifndef ERR_TRIP_H_
#define ERR_TRIP_H_


/*******************************************************************************
 * CONSTANTS
 */

/*******************************************************************************
 * EXTERNS
 */
extern void ERR_setTripFlag(int cause);
extern int ERR_clearTripData(void);
extern int ERR_setTripInfo(int code);
extern int ERR_getCurrentErrCode(void);

#endif /* ERR_TRIP_H_ */
