# modelization and control
This repo contains a collection of models and control applications developed in a multi domain physics environment based on Matlab/Simscape. The repo contains applications on: power electronics, hydrostatic systems, and control system engineering.
How to use the repo:
- In matlab add to the path/with-subfolders the directory "library". This directory contains simscape (ssc) models compiled with "ssc_bultd".
- The folder ./library/user_defined_functions/ccaller contains a list of c-coded functions used inside the simulink models by ccallers.
- The folder ./library/foundation contains all simscape language based model used in the simulink models.

Applications:
- folder power_electronics_projects_design_with_simscape contains power electronics based projects with application on psm, im, dab, zvs, flyback, afe, inverter, hard-parallelization analysis, space-vector multilevel.
- folder hydrostatic_projects_design_with_simscape contains project on hydrostatic powertrains and utilities based on hydrostatic system like: controlled pressure pump, multiways valves, drive-lines based variable swash plate pump/motors.

Power electronics projects:
- afe_inv_psm : model which implements a afe (active front end) and inverter for psm (permanent magnet sunchronos motors/genrators).
- 

