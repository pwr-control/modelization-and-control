%[text] # Power system: three-phase grid emulator, FRT, three-phase transformers, IEC power measurement
%[text] Initialization of power_system.slx: three controlled voltage sources driven by the FRT grid emulator
%[text] of the library (positive/negative sequence references, grid-code dips), series grid impedance,
%[text] the 1 MVA transformer of the emulator (block grid/grid_emulator/3ph_Dyg/saturable_transformer_Dyn1/Dd/T1,
%[text] Simscape language component three_phase_transformer.transformer_Dd of the library, parameters
%[text] param.grid.emu.trafo.pu.*), a second transformer hung on the 690 V bus at no load to test the library
%[text] blocks (.../saturable_transformer_Dyn1/Dyn/T2, three_phase_transformer.transformer_Dyn, parameters
%[text] param.load.trafo.pu.*), three-phase voltage/current measurements, IEC power meters (sequence powers)
%[text] and the analysis subsystem (moving average filters, scopes). The previous transformer of the emulator
%[text] (Dyn11 made of three saturable single-phase transformers, 3ph_Dyg/saturable_transformer_Dyn) and the
%[text] "load" subsystem of the root (RL load, three-phase inductor with saturation) are commented out in the
%[text] model; their parameters are still here.
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, grid_emulator_param, three_phase_transformer_param, grid_fault_param,
%[text] check_model_param_refs). Nothing is read from library/setup_files; the library is needed for the
%[text] Simulink/Simscape blocks (three_phase_transformer_lib.slx must be built: ssc_build three_phase_transformer
%[text] from library/foundation/electrical_machines) and for the C sources of the C-Caller moving average filter.
%[text] Run it from this folder (the functions of ./private are visible only to the files of this folder).
%[text] The previous init is kept as init_model_legacy.m: it no longer runs with the current library
%[text] (timing_setup and grid_fault_generator changed signature) and it initialized converters and machines
%[text] that this model does not contain.
clear;
model = 'power_system';

% Simulink/Simscape library blocks and C-Caller sources (blocks, not parameters)
library_path = 'C:\github\library';
if exist(library_path, 'dir')
    addpath(genpath(library_path));
else
    warning('%s init: library path %s not found, the library blocks must already be on the path', model, library_path);
end

% environment (was init_environment of the library)
close all; clc; beep off;
try
    pm_addunit('percent', 0.01, '1');       % custom Simscape unit used by the library blocks
catch ME
    if ~contains(ME.message, 'percent')     % already registered in this session: fine
        rethrow(ME);
    end
end
param = struct();

% local parameter functions of this model (./private)
local_functions = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
fprintf('%s init: local parameter functions in private/: %s\n', model, strjoin({local_functions.name}, ', '));
clear local_functions library_path
%[text] ## Simulation
param.sim.simlength = 1.25;                 % [s] model StopTime
param.sim.tc = 2.5e-6;                      % [s] step of the Simscape local solver (backward Euler, grid/grid_emulator/
                                            %     Solver Configuration) and sample time of the IEC power meters: ts_afe/50
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration, maximum nonlinear iterations
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% There is no converter in this model: the "AFE" clock domain is only the sample time of the discrete blocks
% of the analysis (moving average filters, To Workspace, 125 us) and the base of the solver step tc.
param.time.fpwm = 4e3;                              % [Hz]
param.time.trgo = 1;                                % double update: ts_afe = 1/(2*fpwm)
param.time.dead_time = 0;                           % [s] unused, kept for the name
param.time.t_measure = 0.2;                         % [s] logging window of the To Workspace blocks (Ns_afe samples)
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts_afe/tc, so that glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

param.glb_time = timing_param(param.time);
%[text] ## Grid emulator
%[text] ### Grid data and transformer nameplate
param.grid.application_voltage = 690;               % [V] 400, 480 or 690: selects the data set below
param.grid.grid_nominal_power = 1000e3;             % [W] transformer rating
param.grid.eq_grid_inductance = 75e-6;              % [H] series impedance of the voltage sources (per phase)
param.grid.eq_grid_resistance = 2e-3;               % [Ohm]

% transformer of the emulator (block T1 = three_phase_transformer.transformer_Dd of the library): primary on
% the voltage sources, secondary on the 690 V bus of the grid subsystem. The connection selects the vector
% group and must match the block in the model: T1 is a Dd block, so 'Dd0' or 'Dd6'. To use again the
% commented subsystem 3ph_Dyg/saturable_transformer_Dyn (Dyn11 made of three saturable single-phase
% transformers, secondary star with the neutral to ground) set 'Dyn11': the coil data below follow the
% connection. The data set is selected by the application voltage.
param.grid.name = 'grid emulator transformer';
param.grid.connection = 'Dd0';                      % 'Dd0'/'Dd6' (T1), 'Dyn11' (saturable_transformer_Dyn)
if param.grid.application_voltage == 690
    param.grid.us1 = 690; param.grid.us2 = 690; param.grid.fgrid = 50;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1800;
elseif param.grid.application_voltage == 480
    param.grid.us1 = 480; param.grid.us2 = 480; param.grid.fgrid = 60;
    param.grid.eta = 95; param.grid.ucc = 4.5;
    param.grid.p_iron = 1400;
else
    param.grid.us1 = 400; param.grid.us2 = 400; param.grid.fgrid = 50;
    param.grid.eta = 95; param.grid.ucc = 4.5;
    param.grid.p_iron = 1000;
end
param.grid.lm0_pu = [];                             % [pu] zero-sequence magnetizing inductance of the library block:
                                                    %      [] = lm (bank of single-phase units), 0.3 - 1 three-limb core

% coil voltages of the connection (delta coil: line voltage, star coil: phase voltage) and turns of the
% single-phase transformer blocks: n1 of the primary coil is given, n2 follows the coil ratio
if param.grid.connection(1) == 'D'
    param.grid.u1_coil = param.grid.us1;
else
    param.grid.u1_coil = param.grid.us1/sqrt(3);
end
if param.grid.connection(2) == 'd'
    param.grid.u2_coil = param.grid.us2;
else
    param.grid.u2_coil = param.grid.us2/sqrt(3);
end
param.grid.n1 = 24;                                 % turns of the primary coil (Lm1 = 0.145 H with the core below)
param.grid.n2 = round(param.grid.n1*param.grid.u2_coil/param.grid.u1_coil);     % 24 for Dd 690/690, 14 for Dyn11

% core of the single-phase transformers: the blocks compute the magnetizing inductance from the geometry
% (Lm = n1^2*mu0*mur*core_area/core_length) and, with enable_sat = 1, saturate the flux at 1.5*psi
param.grid.core_area = 0.05;                        % [m^2]
param.grid.core_length = 2.5;                       % [m]
param.grid.mu0 = 4*pi*1e-7;
param.grid.mur = 10e3;
param.grid.enable_sat = 0;                          % saturation of the single-phase transformers (0: linear)

% magnetizing current of the primary coil from the core geometry (coil current at the coil rated voltage):
% 15 A on the 690 V delta coil, i.e. lm = rated current / magnetizing current = 32 pu for the library block
param.grid.Lm1 = param.grid.n1^2*param.grid.mu0*param.grid.mur*param.grid.core_area/param.grid.core_length;  % [H]
param.grid.i1m = param.grid.u1_coil/param.grid.Lm1/(2*pi*param.grid.fgrid);     % [A]

% references of the emulated voltage: positive and negative sequence [pu of the peak phase voltage]
param.grid.up_xi_pu_ref = 1;
param.grid.up_eta_pu_ref = 0;
param.grid.un_xi_pu_ref = 0;
param.grid.un_eta_pu_ref = 0;
param.grid.ugrid_factor = 1;                        % amplitude factor of the emulated voltage (Vemu_ref_norm)
param.grid.f_sof = 50;                              % [Hz] second order low-pass on the voltage references

% param.grid.emu keeps the name and the fields of the legacy grid emulator object: the blocks read
% param.grid.emu.* (emulator, grid impedance, IEC power meters) and param.grid.emu.trafo.* (3ph_Dyg)
param.grid.emu = grid_emulator_param(param.grid);
%[text] ### Grid fault (FRT)
% two ways to build the fault in the emulator block, selectable independently: type 1 from the phase
% amplitudes and the phase shift of the grid-code catalogue (abc), type 2 from the sequence components
% deepPOS*/deepNEG* (xi/eta); both between start_time_LVRT and start_time_LVRT + error_length of the test
param.grid.frt_in.test_index = 25;                  % test of the catalogue: 25.4 = 75 % dip, two-phase, full load
param.grid.frt_in.test_subindex = 4;
param.grid.frt_in.asymmetric_error_type = 1;        % two-phase tests: 0 variant C, 1 variant D
param.grid.frt_in.enable_frt_1 = 0;                 % fault from the abc amplitudes of the catalogue
param.grid.frt_in.enable_frt_2 = 1;                 % fault from the sequence components below
param.grid.frt_in.start_time_LVRT = 0.125;          % [s]
param.grid.frt_in.t_ramp = 20e-3;                   % [s] ramp of the phase voltages at the fault edges
param.grid.frt_in.deepPOSxi = 1;                    % [pu] positive sequence during the fault (xi, eta)
param.grid.frt_in.deepPOSeta = -0.4;
param.grid.frt_in.deepNEGxi = 0.4;                  % [pu] negative sequence during the fault
param.grid.frt_in.deepNEGeta = 0.4;

param.grid.frt = grid_fault_param(param.grid.frt_in);
%[text] ## Load (root subsystem "load", commented out in the model)
% RL load (block load/three_phase_rl_load, one RL per phase to the neutral of the block) and three-phase
% inductor with saturation in series (block load/threephase inductor with saturation). Same numbers as the
% masks had before the parameters became local: 1 Ohm + 1 mH per phase is about 0.5 pu at 690 V.
param.load.Rs_load = [1 1 1];                       % [Ohm] per phase
param.load.Ls_load = [1e-3 1e-3 1e-3];              % [H] per phase
param.load.ind.LFu = 500e-6;                        % [H] positive sequence inductance
param.load.ind.Rs = 2e-3;                           % [Ohm] positive sequence resistance
param.load.ind.LFu0 = 500e-6;                       % [H] zero sequence inductance
param.load.ind.Rs0 = 1e-3;                          % [Ohm] zero sequence resistance
param.load.ind.isat = 360;                          % [Arms] saturation current
%[text] ### Load transformer (block grid/grid_emulator/3ph_Dyg/saturable_transformer_Dyn1/Dyn/T2)
% Second transformer, Simscape language component three_phase_transformer.transformer_Dyn of the library,
% hung on the 690 V bus of the emulator at no load to test the block: in the model the bus is connected to
% its star side (ports a, b, c; the neutral n is the electrical reference of the bus, the Dd of the emulator
% has none) and the delta side (A, B, C) is open. Both sides are rated 690 V so that the core is at rated
% flux whichever side is fed; set u2_nom = 400 for a 690/400 V distribution transformer (then the bus must
% feed the delta side). i0_pu is the magnetizing current, lm = 1/i0_pu; lm0_pu empty = lm (bank of
% single-phase units), 0.3 - 1 pu for a three-limb core, which shows with a single-phase load on the star.
param.load.trafo_in.name = 'load transformer';
param.load.trafo_in.connection = 'Dyn11';           % T2 is a Dyn block: 'Dyn11', 'Dyn1', 'Dyn5' or 'Dyn7'
param.load.trafo_in.pwr_nom = 250e3;                % [VA]
param.load.trafo_in.u1_nom = 690;                   % [V] delta side (A, B, C)
param.load.trafo_in.u2_nom = 690;                   % [V] star side (a, b, c, n), on the bus in the model
param.load.trafo_in.f_nom = param.grid.fgrid;       % [Hz]
param.load.trafo_in.eta = 98;                       % [%]
param.load.trafo_in.ucc = 5;                        % [%]
param.load.trafo_in.p_iron = 750;                   % [W] 0.3 %
param.load.trafo_in.i1m = [];                       % [A] magnetizing current of the delta coil: from i0_pu
param.load.trafo_in.i0_pu = 0.01;                   % [pu] 1 % magnetizing current, lm = 100
param.load.trafo_in.lm0_pu = [];                    % [pu] [] = lm
param.load.trafo = three_phase_transformer_param(param.load.trafo_in);
%[text] ## Model setup
open_system(model);
%[text] ### C-Caller types
% output struct types of the C-Caller blocks (analysis/moving average filters1/dmavg_flt1 of the library:
% mavgflt_process_simulink). They need the C sources of library/user_defined_functions/ccaller on the path
% and in the custom code settings of the model.
ctypes = {'mavgflt_output_t', 'dsmavgflt_output_t', 'mavgflts_output_t'};
for k = 1:numel(ctypes)
    try
        Simulink.importExternalCTypes(model, 'Names', ctypes(k));
    catch ME
        warning('%s init: import of the C type %s failed (%s): check the C-Caller custom code settings', ...
            model, ctypes{k}, ME.message);
    end
end
clear ctypes k ME
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Check of the param.* references of the model
if param.sim.check_param_refs
    [missing_refs, model_refs] = check_model_param_refs(model, param);
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the model blocks resolve\n', model, numel(model_refs));
    else
        warning('%s init: %d param.* references of the model blocks do not exist in param:', model, numel(missing_refs));
        disp(missing_refs);
    end
    clear missing_refs model_refs
end
