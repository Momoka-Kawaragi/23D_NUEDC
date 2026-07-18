/**************************************************************************/
/*!
    @file     si5351_asserts.h
    @author   POLO_HU
    @section LICENSE

    BSD License
    
*/
/**************************************************************************/
#ifndef _ASSERTS_H_
#define _ASSERTS_H_

#include "si5351_errors.h"

/**************************************************************************/
/*!
    @brief Checks the condition, and returns the provided returnValue if assertion fails.
*/
/**************************************************************************/
#define ASSERT(condition, returnValue) \
        do{\
          if (!(condition)) {\
            return (returnValue);\
          }\
        }while(0)

/**************************************************************************/
/*!
    @brief Checks the provided err_t value (sts), returns sts if it's not equal to ERROR_NONE.
*/
/**************************************************************************/
#define ASSERT_STATUS(sts) \
        do{\
          err_t status = (sts);\
          if (ERROR_NONE != status) {\
            return status;\
          }\
        } while(0)

#endif