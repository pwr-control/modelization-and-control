# Modelization and control
This repo contains a collection of models and control applications developed in a multi domain physics environment based on Matlab/Simscape. The repo contains applications on: power electronics, hydrostatic systems, and control system engineering.

**How to use the repo**:
- In matlab add to the path/with-subfolders the directory "library". This directory contains simscape (ssc) models compiled with "ssc_bultd".
- The folder ./library/user_defined_functions/ccaller contains a list of c-coded functions used inside the simulink models by ccallers.
- The folder ./library/foundation contains all simscape language based model used in the simulink models.

# Documentation
Documentation is available into the folder: **repo_documentations**.
- This folder contains the following documentations.
  - **advanced control engineering**: collections of problems and model derivations with some pages on theory derivation;
  - **electrification of heavy duty**: an approach to mapping an hydrostatic power train into an electrical powertrain;
  - **litium-ion battery**: model derivation;
  - **nonlinear observer**: apporach to hydristatic drivetrain using Khalil apporach;
  - **pmsm motor model and control**: $\alpha-\beta$ and $dq$ derivation and representation of the three phase permanent magnet synchronous machine and model predictive control; 
  - **induction motor model and control**: $\alpha-\beta$ and $dq$ derivation and representation of the three phase induction motor and controls; 
  - **pem fuel cell**: model of the proton-exchange membrane fuel cell;
  - **three phase inductors**: simscape modelization of a three phase inductance;
 
# Preliminary description
**Applications**:
- folder **power_electronics_projects_design_with_simscape** contains power electronics based projects with application on psm, im, dab, zvs, flyback, afe, inverter, hard-parallelization analysis, space-vector multilevel;
- folder **hydrostatic_projects_design_with_simscape** contains projects on hydrostatic powertrains and utilities based on hydrostatic system like: controlled pressure pump, multiways valves, drive-lines based variable swash plate pump/motors;
- folder **electronics_with_simscape** contains projects on electronics, and power converter for watt range applications;
- folder **foundation_of_mechanics** contains projects on mechanics taken from literature and applications;
- folder **lectures_on_advanced_control_engineer** contains problems carries out at MCI during academic years 2019/2022;

**power_electronics_projects_design_with_simscape**:
- please see also the readme into the specific folder.

**hydrostatic_projects_design_with_simscape**:
- please see also the readme into the specific folder.

**foundation_of_mechanics**:
- please see also the readme into the specific folder.

**lectures_on_advanced_control_engineer**:
- please see also the readme into the specific folder.
