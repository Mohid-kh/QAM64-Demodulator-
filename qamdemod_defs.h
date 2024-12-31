// clk, nco, iq, lpf, eqf, and slicer implemented
#ifndef __qamdemod_defs_h__
#define __qamdemod_defs_h __

#include "hls_includes.h"

////////////////////////////////////////////////////////////////////////////////////////////////
// Design Parameters
////////////////////////////////////////////////////////////////////////////////////////////////

// I/O bits (DATABITS and COEFFBITS taken from 0_setup_design.tcl)

#ifndef DATABITS
#define DATABITS                   32
#endif
#ifndef COEFFBITS
#define COEFFBITS                  32
#endif
#define SYMBOLBITS                 6

// clock 
#define CLOCK_BITS                 6

// NCO

#define CORDIC_ITERATIONS          30   // initial : 10, to compensate for log2 error : set to 30

// Low-pass filter LPF

#define LPF_COEFFS                 5
#define LPF_B0                     0
#define LPF_B1                     1
#define LPF_B2                     2
#define LPF_A1                     3
#define LPF_A2                     4

// Matched filter MF

#define MF_TAPS                    25
#define MF_COEFFS                  (MF_TAPS/2+1)

// Equalization filter EQF

#define CMA_R                      1.38
#define CMA_MU                     0.000001
#define EQF_TAPS                   24
#define EQF_COEFFS                 (2*EQF_TAPS)

////////////////////////////////////////////////////////////////////////////////////////////////
// Data types
////////////////////////////////////////////////////////////////////////////////////////////////

typedef ac_fixed<COEFFBITS, 2, true> coeff_t; 
typedef ac_fixed<DATABITS,  2, true> data_t;
typedef ac_int<6, false>             symbol_t;

typedef ac_int<CLOCK_BITS, false>   	             myclock_t;
typedef ac_int<5, false> 			     ncovar_t;
typedef ac_fixed<16, 2, true>       		     nco_t;
typedef const ac_fixed<16, 1, false>		     ncoconst_t; 
typedef ac_complex< ac_fixed<40, 2, true> >  	     eqfc_t;

////////////////////////////////////////////////////////////////////////////////////////////////
// Modulation Parameters
////////////////////////////////////////////////////////////////////////////////////////////////

#define CLOCKS_PER_CARRIER_CYCLE   32
#define CARRIER_CYCLES_PER_SYMBOL  2
#define T_SYMBOL                   64
#define SYMBOL_UPSAMPLING_RATE     8
#define T_SAMPLE                   8
#define PSF_SYMBOLS                3
#define PSF_TAPS                   193

////////////////////////////////////////////////////////////////////////////////////////////////
// Simulation parameters
////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef CLOCK_PERIOD
#define CLOCK_PERIOD                100
#endif

#define MAX_SYMBOLS                1024

#define DEBUG_BASIC  1
#define DEBUG_NCO    2
#define DEBUG_IQ     4
#define DEBUG_LPF    8
#define DEBUG_MF    16
#define DEBUG_EQF   32


#endif
