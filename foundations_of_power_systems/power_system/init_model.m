%[text] # Power system: three-phase grid emulator, FRT, Dyn11 transformer, IEC power measurement
%[text] Initialization of power_system.slx: three controlled voltage sources driven by the FRT grid emulator
%[text] of the library (positive/negative sequence references, grid-code dips), series grid impedance,
%[text] 1 MVA Dyn11 transformer built with three saturable single-phase transformers (grid/grid_emulator/3ph_Dyg),
%[text] three-phase voltage/current measurements, IEC power meters (sequence powers) and the analysis
%[text] subsystem (moving average filters, scopes). The "load" subsystem of the root (RL load and three-phase
%[text] inductor with saturation) is commented out in the model: the transformer runs at no load.
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, grid_emulator_param, three_phase_transformer_param, grid_fault_param,
%[text] check_model_param_refs). Nothing is read from library/setup_files any more; the library is still needed
%[text] for the Simulink/Simscape blocks and for the C sources of the C-Caller moving average filter.
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

% transformer of the emulator: Dyn11, primary (delta) on the voltage sources, secondary (star, neutral to
% ground) on the ports of the grid subsystem. Three saturable single-phase transformers, one per limb
% (block grid/grid_emulator/3ph_Dyg/saturable_transformer_Dyn); the variants with the linear single-phase
% transformers and with the equivalent series inductors are commented out in the model.
param.grid.name = 'Dyn11';
param.grid.delta_star = 1;                          % 1: Dyn (delta primary), 0: Yyn
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

% core of the single-phase transformers: the blocks compute the magnetizing inductance from the geometry
% (Lm = n1^2*mu0*mur*core_area/core_length) and, with enable_sat = 1, saturate the flux at 1.5*psi
param.grid.n2 = 14;                                 % turns of the secondary coil
param.grid.n1 = floor(param.grid.n2*sqrt(3));       % turns of the primary (delta) coil: 24, coil ratio sqrt(3)
param.grid.core_area = 0.05;                        % [m^2]
param.grid.core_length = 2.5;                       % [m]
param.grid.mu0 = 4*pi*1e-7;
param.grid.mur = 10e3;
param.grid.enable_sat = 0;                          % saturation of the single-phase transformers (0: linear)

% magnetizing current of the primary coil from the core geometry (i1m is the coil current at the coil
% rated voltage: the line voltage for the delta coil, the phase voltage for a star coil)
param.grid.Lm1 = param.grid.n1^2*param.grid.mu0*param.grid.mur*param.grid.core_area/param.grid.core_length;  % [H]
if param.grid.delta_star == 1
    param.grid.u1_coil = param.grid.us1;
else
    param.grid.u1_coil = param.grid.us1/sqrt(3);
end
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
