// nco, clock, IQ Demod, LPF, and EQF implemented
#ifndef _qamdemod_h_
#define _qamdemod_h_

#include "qamdemod_defs.h"
#include <iostream>

using namespace std;

static  ac_complex < ac_fixed<40, 2, true> > eqf_coeffs[EQF_TAPS] = {0};        // = c 
static  ac_complex < ac_fixed<40, 2, true> > del_in[EQF_TAPS]     = {0};        // delayed inputs of x(t) 

#pragma hls_design top
class qamdemod {

  // clk variables
  ac_int<6, false> clk_cntr;         // counter to track clock (0-63)
  ac_int<6, false> lpf_clk;          // lpf clock 
  ac_int<1, false> mf_clk;           // matched filter clock
  ac_int<6, false> eqf_clk;          // equalizer clock
  ac_int<6, false> slc_clk;          // slice clock
  ac_fixed<16,  2, true> iq;         // read input iq 
  ac_int<6, false> symbol;           // write to output symbol_out

  // nco variables
  ac_int<5, false> nco_acc;          // nco accumulator (phase)
  ac_int<2, false> qdr;              // quadrant number
  ac_int<3, false> offset;           // offset from own quadrant
  ac_int<4, false> q0off;            // offset mapped to first quadrant
  const ac_fixed<16, 1, false> K = 0.607252935;  // constant initializer
  const ac_fixed<16, 1, false> theta_lut[CORDIC_ITERATIONS] = {0.7853981634, 0.463647609, 0.2449786631, 0.1243549945, 0.06241881,
                  0.03123983343, 0.01562372862, 0.00781234106, 0.003906230132, 0.001953122516};
  ac_fixed<16, 2, true> cos_theta;  // fixed-point with 1 integer bit, 15 fractional bits, first quadrant cos value
  ac_fixed<16, 2, true> sin_theta;  // first quadrant sine value
  ac_fixed<16, 1, false> cos_shift;
  ac_fixed<16, 1, false> sin_shift;
  ac_fixed<16, 2, true>  theta;    
  ac_fixed<16, 2, true>  sinv;
  ac_fixed<16, 2, true>  cosv;

  // iq demod and lpf variables
  ac_fixed<32, 2, true> rxi,  rxq;
  ac_fixed<32, 2, true> lpfi, lpfq; 

  // mf variables
  ac_fixed<32, 2, true> mfi, mfq;

  // eqf variables
  ac_complex< data_t > eqf_r, eqf_img;                       
  ac_complex< ac_fixed<40, 2, true> > y_t[EQF_TAPS];	  // store individual coeff*delayed inputs (fir)
  ac_complex< ac_fixed<40, 2, true> > eqf_out = 0;        // cumulative sum (output of fir) 
  ac_complex< ac_fixed<40, 2, true> > CMA_e;            		  // e(t)  
  ac_fixed<8, 1, false> R = CMA_R;           	 	  // 1.38 
  ac_fixed<32, 2, false> MU = CMA_MU;			  // 0.000001
  ac_fixed<80, 2, false> modsq_eqfout;                    // eqf_out mod squared           
  
  // Constructor: Initialize variables that represent static storage
 public:           
  qamdemod() {           
  }
  void clk_gen(void){
    // clock generation 
    clk_cntr++;
    //cout<<"Main clock : "<<clk_cntr<<endl; 
    if(clk_cntr%1 == 0)
    { 
      lpf_clk++;
     // cout<<"lpf clock : "<<lpf_clk<<endl;      
    }
    //clk_cntr == 4 || (clk_cntr > 4 && (clk_cntr - 4) % 8 == 0)    
    if(clk_cntr%8==0)
    {
      mf_clk=1;
      //cout<<"mf clock : "<<mf_clk<<endl; 
    }
    else 
    {
      mf_clk=0;
    }
    if(clk_cntr%16 == 0)
    {
      eqf_clk = 1;
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
  }
  void nco(void){        // nco implementation  

   // cout<<"nco_acc : "<<nco_acc<<endl;
    qdr    = nco_acc >> 3;    // = msb 2 bits of nco
    offset = nco_acc & 0x07;  // = lsb 3 bits of nco

    if(qdr == 0 || qdr == 2)
      q0off = offset;
    else
      q0off = 8 - offset;

   // cout<<"qdr : "<<qdr<<endl;
   // cout<<"offset : "<<offset<<endl;
   // cout<<"q0off : "<<q0off<<endl;

    theta  = q0off * (ac_fixed<16, 1, false>)((2*3.142)/32);      // convert angle to radians
  
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
     // cout<<"theta : "<<theta<<endl;
     // cout<<"cos : "<<cos_theta<<endl;
     // cout<<"sin : "<<sin_theta<<endl;
    }  

    if(qdr == 0)
    {
      cosv  = cos_theta; 
      sinv  = sin_theta;
    }
    else if(qdr == 1)
    {
      cosv  = -cos_theta;  // cosv = -cos_theta
      sinv  = sin_theta;
    }
    else if(qdr == 2)
    {
      cosv  = -cos_theta; 
      sinv  = -sin_theta;
    }    
    else 
    {
      cosv  = cos_theta;
      sinv  = -sin_theta;
    }
   // cout<<"cosv : "<<cosv<<endl;
   // cout<<"sinv : "<<sinv<<endl;
    nco_acc++;
    if(nco_acc > 31)
    {
      nco_acc = 0;     // roll over 
    }

    for(uint8_t i = 0; i < EQF_TAPS; i++)
    {
    //  y_t = eqf_taps * x_t(eqf_clk-i);       // FIR difference equation
    }
    
    //CMA_error = (R - |y(t)^2|*y(t); 

  } 
  void iq_demodulate(void) {
    rxi = iq * cosv; // Real component
    rxq = iq * sinv; // Imaginary component
   // cout<<"sinv: "<<sinv;
    //cout<<"\tcosv: "<<cosv;
    //cout<<"\tiq: "<<iq; 
    //cout<<"my rxi is : "<<rxi<<endl;
    //cout<<"my rxq is : "<<rxq<<endl;
    //cout<<"my iq is :  "<<iq<<endl;
    //cout<<"my sinv is :  "<<sinv<<endl;
  }  

  void lpf(coeff_t lpf_coeff_in[LPF_COEFFS]){
    // Static delay elements for input and output state storage
    static data_t delay_rxi[3] = {0}; // x(t), x(t-1), x(t-2)
    static data_t delay_rxq[3] = {0};
    static data_t delay_lpfi[2] = {0}; // y(t-1), y(t-2)
    static data_t delay_lpfq[2] = {0};

    // Compute LPF output for rxi
    lpfi = 2*(lpf_coeff_in[0] * rxi) 
           + (lpf_coeff_in[1] * delay_rxi[0])
           + (lpf_coeff_in[2] * delay_rxi[1])
           - (lpf_coeff_in[3] * delay_lpfi[0])
           - (lpf_coeff_in[4] * delay_lpfi[1]);

    // Updating delay line for rxi
    delay_rxi[1] = delay_rxi[0];
    delay_rxi[0] = rxi;

    //cout<<"my lpfi post bz1     : "<<delay_rxi[0];
    //cout<<"my lpfi post  bz2     : "<<delay_rxi[1]<<endl;


    // Update delay line for lpfi (output)
    delay_lpfi[1] = delay_lpfi[0];
    delay_lpfi[0] = lpfi;

    // Compute LPF output for rxq
    lpfq = 2*(lpf_coeff_in[0] * rxq)
           + (lpf_coeff_in[1] * delay_rxq[0])
           + (lpf_coeff_in[2] * delay_rxq[1])
           - (lpf_coeff_in[3] * delay_lpfq[0])
           - (lpf_coeff_in[4] * delay_lpfq[1]);

    // Updating delay line for rxq
    delay_rxq[1] = delay_rxq[0];
    delay_rxq[0] = rxq;

    // Update delay line for lpfq (output)
    delay_lpfq[1] = delay_lpfq[0];
    delay_lpfq[0] = lpfq;

    //cout<<"my lpfq is : "<<lpfq<<endl;
    //cout<<"my lpfi is : "<<lpfi<<endl;
    //cout<<"my b0      : "<<lpf_coeff_in[0]<<endl;
    //cout<<"my b1      : "<<lpf_coeff_in[1]<<endl;
    //cout<<"my b2      : "<<lpf_coeff_in[2]<<endl;
    //cout<<"my -a1     : "<<lpf_coeff_in[3]<<endl;
    //cout<<"my rxi     : "<<rxi<<endl;
  }

  void mf(coeff_t mf_coeff_in[MF_COEFFS]) {
  
    ac_int<4, false> index_1=12;
    ac_int<4, false> index_2=12;
    mfi=0;
    mfq=0;

    static data_t delay_lpfi[25] = {0}; // Input delay line for lpfi
    static data_t delay_lpfq[25] = {0}; // Input delay line for lpfq

    delay_lpfi[0] = lpfi;
    delay_lpfq[0] = lpfq;
    //cout<<"my lpfi      : "<<delay_lpfi[0]<<endl;
    //cout<<"my lpfq      : "<<delay_lpfq[0]<<endl;

    for (int i = 0; i < 25; i++) {
      mfi += mf_coeff_in[index_1] * delay_lpfi[i];
      if(i<12)
      {
        index_1--;
      }
      else{
        index_1++;
      }
    }

    // Compute MF output for lpfq
    for (int i = 0; i < 25; i++) {

      mfq += mf_coeff_in[index_2] * delay_lpfq[i];
      
      if(i<12)
      {
        index_2--;
      }
      else{
        index_2++;
      }
    }

    // Shift delay lines for lpfi and lpfq
    for (int i = 24; i >0; i--) {
        delay_lpfi[i] = delay_lpfi[i - 1];
        delay_lpfq[i] = delay_lpfq[i - 1];
    }

    // Debug output
    //cout << "my mfi: " << mfi << ", my mfq: " << mfq << endl;
  }
  void eqf(void){ 
    eqf_out.r() = 0;
    eqf_out.i() = 0;
    del_in[0].r() = mfi;   // x(t)
    del_in[0].i() = mfq;
    eqf_coeffs[EQF_TAPS/2].r() = 1;
    eqf_coeffs[EQF_TAPS/2].i() = 0; 
    //cout<<"mfi : "<<mfi<<"\tmfq : "<<mfq<<endl;
    cout<<"del_in[0] : "<<del_in[0]<<endl;
    for(uint8_t i = 0; i < EQF_TAPS; i++)       // FIR difference equation 
    {
      y_t[i] = eqf_coeffs[i] * del_in[i];       // complex numbers multiplication
      eqf_out.r() = eqf_out.r() + y_t[i].r();   // real part summed output
      eqf_out.i() = eqf_out.i() + y_t[i].i();   // img  part summed output
      //cout<<"eqf_out (R,Img) : "<<eqf_out.r()<<"\t"<<eqf_out.i()<<endl;
    }    
    for(uint8_t i = 0; i < EQF_TAPS; i++)       // del_in[0] = x(t), del_in[1] = x(t-1), del_in[2] = x(t-2)
    {
      del_in[i + 1] = del_in[i];      
    }
 
    modsq_eqfout = (eqf_out.r() * eqf_out.r()) + (eqf_out.i() * eqf_out.i());
    CMA_e = ac_complex< ac_fixed<40, 2, true> >((R - modsq_eqfout)*(eqf_out));                   // e(t) real part
    //cout<<"CMA error (R, Img) : "<<CMA_e.r()<<"\t"<<CMA_e.i()<<endl;  
 
    for(uint8_t i = 0; i < EQF_TAPS; i++)   
    {
      //cout<<"gradient (mu*e(t)) : "<<ac_complex <ac_fixed<32, 2, true> >(MU*CMA_e)<<endl;
      eqf_coeffs[i] = ac_complex< ac_fixed<40, 2, true> >(eqf_coeffs[i] + (MU*CMA_e*del_in[i].conj()));
      //cout<<"eqf_coeffs (R, Img): "<<eqf_coeffs[i].r()<<"\t"<<eqf_coeffs[i].i()<<endl;
    }
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
    
    // symbol_out.write(symbol);   // write output fifo style
    iq = iq_in.read();             // read input   fifo style
    clk_gen();
    nco();
    iq_demodulate();
    lpf(lpf_coeff_in);
    //executes once every 8 clock cycles
    if(mf_clk)
     mf(mf_coeff_in); 
    if(eqf_clk)
    {
      eqf();
      eqf_clk = 0;
    }

    /* To do: Assign your variables to debug outputs */
#ifdef QAMDEMOD_DEBUG
    sin_out  = sinv;
    cos_out  = cosv;
    rxi_out  = rxi;
    rxq_out  = rxq;
    lpfi_out = lpfi;
    lpfq_out = lpfq;
    mfi_out  = mfi;
    mfq_out  = mfq;
    eqfi_out = eqf_out.r();
    eqfq_out = eqf_out.i();
#endif

    //LPF Block
    //lpf(rxi, rxq, lpf_coeff_in, lpfi, lpfq);
  }
    
};

#endif
