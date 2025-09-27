# power electronics design with simscape

This subfolder contains a collection of models and control applications concerning power electronics. Most of the projects use the library which must added to the path of matlab to be able to use these models.
Each folder contains a power electronics project, in the following a description of each project. 

**ac-ac-buck-converter**:
- Tipical application with two antiseries igbts used to create a bidirectional switch. The modulation strategy use overlapping deadtime, a detailed explanation can be found into the subfolders of the project.

**afe-inv-psm**:
- Bidirectional electrical drive for permanent magnet synchrounous machine. The model contains a three phase AFE (active front end) a dclink braking unit, an inverter, and a permanent magnet synchronous motor.
- Model contains:
  - current vector control implemented both in simulink as well as in C code using C-caller blocks;
  - psm (permanent magnet synchronous machine) back emf based observer in simulink and C code;
  - three phase current control with resonant PI;
  - sequence calculation without buffering (very efficient);
  - model can run as driver or as gerator e.g. wind turbine applications.

**afe-inv-3L-psm**:
- same functionalities of **afe-inv-psm** project but implementing three level t-type inverters (for both afe and inverter).

**bat-inv-psm**:
- same functionalities of **afe-inv-psm** project but implementing lithium-ion battery, and a not isolated dcdc converter; no grid or afe is implemented.

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
