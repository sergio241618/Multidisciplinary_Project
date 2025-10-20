/*
 * File: rtwtypes.h

 * Code generated for Simulink model 'simulink_control'.
 * Model version                  : 1.19
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Sun Oct 19 18:04:12 2025

 * Purpose: This file defines standard data types used by the generated code to ensure
 * portability across different hardware platforms (e.g., from a 64-bit PC to a 32-bit ESP32).
 */

#ifndef RTWTYPES_H //header guard
#define RTWTYPES_H

/* --- Logical type definitions (true/false) --- */
// For C code that is not C++, these lines define 'false' as 0 and 'true' as 1,
// which is a standard convention.
#if (!defined(__cplusplus))
#ifndef false
#define false (0U)
#endif
#ifndef true
#define true (1U)
#endif
#endif

/* --- Standard fixed-width word size data types --- */
// This section creates portable "aliases" (using typedef) for fundamental data types.
// The goal is to guarantee the exact size in bits for each type, regardless of the
// compiler or hardware architecture.
typedef signed char   int8_T;  
typedef unsigned char uint8_T;  
typedef short         int16_T;  
typedef unsigned short uint16_T; 
typedef int           int32_T;  
typedef unsigned int  uint32_T; 
typedef float         real32_T;
typedef double        real64_T; 

/* --- Generic type definitions --- */
// These are the general-purpose types that Simulink will use for most calculations.
// By default, 'real_T' is set to 'double' to ensure maximum numerical precision and avoid rounding errors in control calculations.
typedef double        real_T;   // The default type for all floating-point calculations.
typedef double        time_T;   // The default type for representing time.
typedef unsigned char boolean_T;
typedef int           int_T;   
typedef unsigned int  uint_T;   
typedef unsigned long ulong_T;  
typedef char          char_T;  
typedef unsigned char uchar_T;  
typedef char_T        byte_T;  

/* --- Min and Max values --- */
// These macros define the minimum and maximum possible values for the fixed-width
// integer types. This is useful for checking for overflows or for saturation logic.
#define MAX_int8_T   ((int8_T)(127))
#define MIN_int8_T   ((int8_T)(-128))
#define MAX_uint8_T  ((uint8_T)(255U))
#define MAX_int16_T  ((int16_T)(32767))
#define MIN_int16_T  ((int16_T)(-32768))
#define MAX_int32_T  ((int32_T)(2147483647))
#define MIN_int32_T  ((int32_T)(-2147483647-1))
#define MAX_uint32_T ((uint32_T)(0xFFFFFFFFU))

/* --- Block D-Work pointer type --- */
// This defines a generic pointer type used internally by Simulink for managing
// the memory for block states ("D-Work" vectors).
typedef void * pointer_T;

#endif /* End of the RTWTYPES_H header guard */