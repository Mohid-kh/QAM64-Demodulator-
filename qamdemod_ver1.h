// NCO and clock Implementation
#ifndef __qamdemod_h__
#define __qamdemod_h__

#include "qamdemod_defs.h"
#include <iostream>

using namespace std;

#pragma hls_design top
class qamdemod {

  // Declare member variables
  ac_int<6, false> clk_cntr;         // counter to track clock (0-63)
  ac_int<6, false> lpf_clk;          // lpf clock 
  ac_int<6, false> mf_clk;           // matched filter clock
  ac_int<6, false> eqf_clk;          // equalizer clock
  ac_int<6, false> slc_clk;          // slice clock
  ac_fixed<18,  2, true> iq;         // read input iq 
  ac_int<6, false> symbol; 	     // write to output symbol_out

  ac_int<5, false> nco_acc;   	     // nco accumulator (phase)
  ac_int<2, false> qdr;              // quadrant number
  ac_int<3, false> offset;           // offset from own quadrant
  ac_int<4, false> q0off;            // offset mapped to first quadrant
  const ac_fixed<16, 1, false> K = 0.607252935;  // constant initializer
  const ac_fixed<16, 1, false> theta_lut[CORDIC_ITERATIONS] = {0.7853981634, 0.463647609, 0.2449786631, 0.1243549945, 0.06241881,
						      0.03123983343, 0.01562372862, 0.00781234106, 0.003906230132, 0.001953122516};
  ac_fixed<16, 1, false> cos_theta;  // fixed-point with 1 integer bit, 15 fractional bits, first quadrant cos value
  ac_fixed<16, 1, false> sin_theta;  // first quadrant sine value
  ac_fixed<16, 1, false> cos_shift;
  ac_fixed<16, 1, false> sin_shift;
  ac_fixed<16, 1, true>  theta;    
  ac_fixed<16, 1, true>  sinv;
  ac_fixed<16, 1, true>  cosv;

  
  // Constructor: Initialize variables that represent static storage
 public:           
  qamdemod() {     

  }

  // Synthesizable member function
  
#pragma hls_design interface
  void CCS_BLOCK(run) (ac_channel<data_t> &iq_in,
		       coeff_t lpf_coeff_in[LPF_COEFFS],
		       coeff_t mf_coeff_in[MF_COEFFS],			  
		       ac_channel<symbol_t> &symbol_out
#ifdef QAMDEMOD_DEBUG
		       ,
		       data_t &sin_out,
		       data_t &cos_out,
		       data_t &rxi_out,
		       data_t &rxq_out,	      			  
		       data_t &lpfi_out,
		       data_t &lpfq_out,	      
		       data_t &mfi_out,
		       data_t &mfq_out,	      
		       data_t &eqfi_out,
		       data_t &eqfq_out	      
#endif	      
		       )
  {

    /* To do: Write code below */
    
    iq = iq_in.read();             // read input   fifo style
    // symbol_out.write(symbol);   // write output fifo style

    // clock generation (move to function later)
    clk_cntr++;
    if(clk_cntr%1 == 0)
    { 
      lpf_clk++;
     // cout<<"lpf clock : "<<lpf_clk<<endl;      
    }    
    if(clk_cntr%8 == 0)
    {
      mf_clk++;
     // cout<<"mf clock : "<<mf_clk<<endl; 
    }
    if(clk_cntr%16 == 0)
    {
      eqf_clk++;
     // cout<<"eqf clock : "<<eqf_clk<<endl;
    }
    if(clk_cntr%64 == 0)
    {
      slc_clk++;
     // cout<<"scl clock : "<<slc_clk<<endl;
    }
    if(clk_cntr == 64)
    {
      clk_cntr = 0;      // counter reset
     // cout<<"clk counter : "<<clk_cntr<<endl;
    }
    
    // nco implementation (move to function later) 
    cout<<"nco_acc : "<<nco_acc<<endl;
    qdr    = nco_acc >> 3;
    offset = nco_acc & 0x07;

    if(qdr == 0)
    {
      q0off = offset;
      cosv  = cos_theta; 
      sinv  = sin_theta;
    }
    else if(qdr == 1)
    {
      q0off = 8 - offset;
      cosv  = ~cos_theta + 1;  // cosv = -cos_theta
      sinv  = sin_theta;
    }
    else if(qdr == 2)
    {
      q0off = offset;
      cosv  = ~cos_theta + 1; 
      sinv  = ~sin_theta + 1;
    }
    else
    {
      q0off = 8 - offset;
      cosv  = cos_theta;
      sinv  = ~sin_theta + 1;
    }
    cout<<"qdr : "<<qdr<<endl;
    cout<<"offset : "<<offset<<endl;
    cout<<"q0off : "<<q0off<<endl;
    theta = q0off * (ac_fixed<16, 1, true>)((2*3.142)/32);      // convert angle to radians
    // CORDIC ALGO [1st quadrant]    
    cos_theta = K;
    sin_theta = 0;
    for(uint8_t i = 0; i < CORDIC_ITERATIONS; ++i)
    {
      cos_shift = cos_theta >> i;
      sin_shift = sin_theta >> i;
      if(theta >= 0)
      {
        cos_theta = cos_theta - sin_shift;
        sin_theta = sin_theta + cos_shift;
        theta     = theta  - theta_lut[i];
      }
      else
      {
        cos_theta = cos_theta + sin_shift;
        sin_theta = sin_theta - cos_shift;
        theta     = theta  + theta_lut[i];
      }
      cout<<"theta : "<<theta<<endl;
      cout<<"cos : "<<cos_theta<<endl;
      cout<<"sin : "<<sin_theta<<endl;
    }  
    nco_acc++;
    if(nco_acc > 30)
    {
      nco_acc = 0;     // roll over 
    }

    /* To do: Assign your variables to debug outputs */
#ifdef QAMDEMOD_DEBUG
    sin_out  = sinv;
    cos_out  = cosv;
    rxi_out  = 0;
    rxq_out  = 0;
    lpfi_out = 0;
    lpfq_out = 0;
    mfi_out  = 0;
    mfq_out  = 0;
    eqfi_out = 0;
    eqfq_out = 0;
#endif
  }
    
};

#endif
