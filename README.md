This repository contains the source code for a 64-QAM demodulator implemented in SystemC. High-Level Synthesis (HLS) was done using Catapult. The demodulator processes 64-QAM signals for communication systems and includes following key modules for signal processing.

Main Modules:
NCO (Numerically Controlled Oscillator): Generates the reference carrier signal for the demodulation process.  
Low Pass Filter: Filters out high-frequency components to ensure accurate signal reception.  
Matched Filter: Optimizes the received signal to match the expected symbol waveform for better signal quality.  
Equalization Filter: Compensates for channel impairments (e.g., noise and distortion) to improve signal integrity.  
Slicer: Converts the filtered signal into discrete symbols, performing the final decision-making process to identify the transmitted bits.  
Key Features:  
High-level design using SystemC for simulation.  
Synthesis and optimization performed using Catapult HLS for hardware implementation.  
Modular architecture for easy integration and testing.  
