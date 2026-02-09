# power electronics design with simscape

This subfolder contains a collection of models and control applications concerning power electronics. Most of the projects use the library which must added to the path of matlab to be able to use these models.
Each folder contains a power electronics project, in the following a description of each project;

**ac-ac-buck-converter**:
- buck converter implemented with BDS (Bidirectional Switches);
- tipical application with two antiseries igbts used to create a bidirectional switch. The modulation strategy use overlapping deadtime, a detailed explanation can be found into the subfolders of the project;

**afe-inv-loopback**:
- a three phase active front end (AFE) + DClink + INVERTER + Transformer closed back to AFE;
- see **afe-inv-psm** for addition details;
- this repo is an application of **afe-inv-psm**;

**afe-inv-psm**:
- bidirectional electrical drive for permanent magnet synchrounous machine. The model contains a three phase AFE (active front end) a dclink braking unit, an inverter, and a permanent magnet synchronous motor;
- model contains:
  - current vector control implemented both in simulink as well as in C code using C-caller blocks;
  - psm (permanent magnet synchronous machine) back emf based observer in simulink and C code;
  - three phase current control with resonant PI;
  - sequence calculation without buffering (very efficient);
  - model can run as driver or as gerator e.g. wind turbine applications;
  - new dqPLL based on FHT;

**afe-inv-psm-cascade**:
- this folder contains the most advanced concept in modelization of the whole repo. n-time domains are implemented, each time domain generates a trigger (TRGO) for sampling, control and pwm generation. In this modelization, effects of sliding pwm as well as clocks deviation can be fully analyzed. The model contains up to three systems in parallel, where each system is composed by a three phase active rectifier, an inverter and a permanent magnet synchronous generator. All physical blocks are developed with open source custom simscape code;

**afe-inv-threelevel-psm**:
- same functionalities of **afe-inv-psm** project but implementing three level t-type inverters (for both afe and inverter);

**bat-inv-psm**:
- same functionalities of **afe-inv-psm** project but implementing lithium-ion battery, and a not isolated dcdc converter; no grid or afe is implemented;

**booster-dipole-dls**:
- booster dipole for the diamond light source (UK): diode rectifier + boost + full bridge inverter, parallel of three;
- under construction;

**dab-1k5V-250kW**:
- single phase dual active bridge 1.5kV/1.5kV 250kW;
- three phase (as per de doncker) dual active bridge 1.5kV/1.5kV 250kW;

**dab-inv-single-phase-6kVA**:
UPS application with battery, dab, and single phase invertert for emergy light.
In particular:
- lithium iojn battery with residual charge estimation model;
- dual active bridge with control;
- single phase inverter with voltage and current control;
- dq-pll;

**dcdc-zcs-with-double-resonant-tank**:
- converter based on half bridge converter and control based on variable switching frequency;
- this architecture implements a dcdc unidirection isolated by a zero current switching;
- the model implements also a strategy for dclink voltage ripple compensation;

**dcdc-zvs-full-bridge**:
- dcdc with insolation, with full bridgr at primary side and diode rectifier at secondary side;
- dimensioning of the magnetics;
- power flow via phase shift between the two primary legs;

**distributed-ctrl**:
- same functionalities of **afe-inv-psm** but a distributed control is implemented, basically remote sensors with two step sampling time of delay;
- investigation on distributed control, effects of delayed feedback;

**flyback-250V-500W**:
- study of a flyback converter using simscape and flux domain;

**flying-capacitor-single-phase-inverter**:
- study of a flying capacitor single phase inverter from 4000Vdc to 230Vac - 500W; two orders of modelization: one fundamental and one with parassitic effects;

**frt-dqpll**:
- investigation of different DQ-PLL implementations;
- really good behavior about FHT_dqPLL which is a variation of the SOGI_dqPLL;

**grid-emulator**:
- grid forming and grid emulator at 50Hz;

**grid-emulator-400Hz**:
- grid forming and grid emulator at 400Hz;

**im-analysis-and-design**:
- induction motor and indirect field oriented control (IFOC) analysis;

**im-sensorless-analysis-and-design**:
- induction motor and sensorless indirect field oriented control (IFOC) analysis;

**inv-psm**:
- inverter with psm;
- this model is basically used for control system and power losses evaluation;
- model is made by a battery plusc an inverter plus an psm;

**modular-multiphase**:
- investigation on multiphase machine;
- unfortunately market is not so reactive, at least around me;

**mpc-ttype-psm**:
- model predicitive control of three phase ttype inverter (three levels leg based); very useful model for model predictive analysis and investigation on convex optimization;
 
**mpc psm**:
- system with battery (faster than simple Udc ideal source), inverter, and psm; current control based on model predictive control of psm for two level inverter application (resources from Geyer and Rodriguez); some interesting hw implementation of deadtime with CMOS architecture;

**power_meter**:
- in this folder an implementation of the power measure according to VDE/IEC is analyzed.

**psm-sensorless-ctrl**:
- this folder contains a simplified model useful for control design for faster simulation: no pwm, but still psm simscape model;
- **psm_sv_bemf_ctrl** contains a current control based on simple space vector and simple luemberg BEMF observer;
- **psm_mpc_bemf_ctrl** contains a current control based on model predictive for one step horizon and simple luemberg BEMF observer;
- **psm_sv_ekf_bemf_ctrl** contains a current control based on simple space vector and BEMF observer based on Extended Kalman Filter;
- **psm_mpc_ekf_bemf_ctrl** contains a current control based on model predictive for one step horizon and BEMF observer based on Extended Kalman Filter;
  
**sic-inverter-for-drone**:
- design HW/SW of an inverter for drone application (20kW);

**single phase inverter**:
- single phase inverter application with resonant pi, SOGI, active SOGI, and system identification;

**space-vector-and-model-predictive**:
- some theoretical investigatrion following Tobias Geyer and Rodriguez;

**theory analysis dab**:
- two batteries connected among them by a DAB. This model is intended for deep DAB analysis in term of modulation strategies and efficiency analysis. The model contains a detailed Mosfet model;

**theory analysis resonant CLLC**:
- two batteries connected among them by an CLLC. This model is intended for deep resonant-LLC analysis in term of modulation strategies and efficiency analysis. The model contains a detailed Mosfet model;

**thyristor-based-models**:
- twelve and six pulse rectifier;
- bidirectional applications;
- study and firing mechanism analysis and implemetation in C code;

**zvs-inv-single-phase-inverter-35kVA**:
- zero voltage switching based on full-bridge LLC (switching frequenzy 5 times higher than resonant tank);
- single phase inverter with control algoritms;

**zvs-inv-single-phase-inverter-6kVA**:
- zero voltage switching based on full-bridge LLC (switching frequenzy 5 times higher than resonant tank);
- single phase inverter with control algoritms;


