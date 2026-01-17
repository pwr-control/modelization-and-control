# Modelization and control
This repo contains a collection of models and control applications developed in a multi domain physics environment based on Matlab/Simscape. 
The repo contains applications on: power electronics, hydrostatic systems, and control system engineering.

**How to use the repo**:
- In matlab add to the path/with-subfolders the repo "library". This repo contains all fundamentals simscape (ssc) models compiled with "ssc_bultd".
- The folder ./library/user_defined_functions/ccaller contains a list of c-coded functions used inside the simulink models by ccallers.
- The folder ./library/foundation contains all simscape language based model used in the simulink models.

 # Documentation
Documentation is available into the **library** repository.


# Preliminary description
**Applications**:
- folder **power_electronics_projects_design_with_simscape** contains power electronics based projects with application on psm, im, dab, cllc, llc, flyback, afe, inverter, hard-parallelization analysis, space-vector multilevel, model predictive control;
- folder **power_systems** contains some anaysis of different power system architectures;
- folder **semiconductor_device_modeling** contains projects on electronics, semiconductor devices, and power converter for watt range applications; modelization of semiconductor devices is shyly approached, and is still on development; some CMOS equivalent circuit analysis, usefull for verilog translation, are apporached for gate cmds leg-deadtime generation;
- folder **foundation_of_mechanics** contains projects on mechanics taken from literature and applications;
- folder **foundation_of_magnetics** application to magnetics with analytical solution and FEMM;

**power_electronics_projects_design_with_simscape**:
- this folder is becoming quite huge, so please, for more details check readme into the specific folder; to use these models it remains fundamental to add **library** to matlab path;

**foundation_of_mechanics**:
- please see also the readme into the specific folder.

**foundation_of_magnetics**:
- some intro with FEMM and magnetics design mainly for power electronics applications.
