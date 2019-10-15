//#include "global_define.h"
#include <stm32f10x_lib.h>
#include "gran.h"
#include "main.h"

//-----------------------------------------------
void gran_ring_char(signed char *adr, signed char min, signed char max)
{
if (*adr<min) *adr=max;
if (*adr>max) *adr=min; 
} 
 
//-----------------------------------------------
void gran_char(signed char *adr, signed char min, signed char max)
{
if (*adr<min) *adr=min;
if (*adr>max) *adr=max; 
} 

//-----------------------------------------------
void gran(signed short *adr, signed short min, signed short max)
{
if (*adr<min) *adr=min;
if (*adr>max) *adr=max; 
} 

//-----------------------------------------------
void gran_ring(signed short *adr, signed short min, signed short max)
{
if (*adr<min) *adr=max;
if (*adr>max) *adr=min; 
} 

//-----------------------------------------------
void gran_long(signed long *adr, signed long min, signed long max)
{
if (*adr<min) *adr=max;
if (*adr>max) *adr=min; 
} 
