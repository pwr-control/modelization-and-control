%[text] # Single-phase PLLs (single\_phase\_dqPLLs\_analysis, model single\_phase\_pll\_analysis)
%[text] Initialization of single\_phase\_pll\_analysis.slx. The model is a test bench for three phase locked loops on the same single-phase voltage, compared side by side, all discrete at the control period ts = 125 us: the Park PLL (block single\_phase\_pll, from the single-phase inverter control: quadrature signal generator made of a direct and an inverse Park transform with two low-pass filters, a SOGI in disguise, then a PI on eta with the nominal frequency as feed-forward), the FHT-PLL (first harmonic tracker as quadrature generator, two more trackers and the positive-sequence calculator, then the PI loop of the AFE control) and the DDSRF-PLL (the same tracker, then the decoupled double synchronous reference frame of the three-phase AFE control, block copied from frt\_dqpll). Each loop ends in a double-integrator observer for the frequency.
%[text] The input is the block test\_signal (MATLAB Function, mask parameter p = param.test): a voltage in pu of the nominal peak with sags, phase jumps, frequency steps, DC offset and harmonics, together with the angle and the frequency of its fundamental, so the phase error of every PLL is known. Six test cases are in private/test\_param.m; case 1 is the test signal of the 2021 model (4th and 5th harmonic 10 percent, sag to 0.2 pu from 0.25 s to 0.5 s, start 180 degrees away from the PLLs). The To Workspace blocks of data\_logging keep the input and the outputs of the three loops (sig\_log, park\_pll\_log, fht\_pll\_log, ddsrf\_pll\_log).
%[text] All the data are in one struct param: sim, time, glb\_time, grid, test, afe. The PLL blocks keep the names of the AFE control of frt\_dqpll (param.afe.ctrl, param.grid.emu.omega\_grid\_nom, param.glb\_time.ts\_afe), so a block copied between the two models needs no change; the README lists the old names. The functions of ./private are called through the handles of single\_phase\_pll\_analysis\_private.m (prv.timing\_param, prv.test\_param, ...): the Live Editor runs a live script from a copy outside this folder and does not see ./private. Run the whole script once (Run): a section run on its own needs prv and param in the workspace.
%[text] After a simulation, plotting\_results\_plls.m plots the input, the phase errors and the frequencies of the three loops; run\_test\_cases.m simulates the six cases and prints a table of errors, settling times and ripples. Design note of this model (the three PLLs as built, their linear models, the test results, the criticalities and what to change): repos\_doc/single\_phase\_lock\_loop/single\_phase\_pll.pdf of pwr-control/repositories-documentation, https://github.com/pwr-control/repositories-documentation/tree/main/repos\_doc/single\_phase\_lock\_loop.
clear;
model = 'single_phase_pll_analysis';
prv = single_phase_pll_analysis_private();     % handles to the functions of ./private (the Live Editor does not see ./private)
prv.library_setup();            % pwr-control/library on the path (setup classes, discrete low-pass filter block), cloned when missing

% environment (was init_environment of the library)
close all; clc; beep off;
options = bodeoptions;
options.FreqUnits = 'Hz';
param = struct();
fprintf('%s init: functions of ./private, called as prv.<name>: %s\n', model, strjoin(fieldnames(prv)', ', '));
%%
%[text] ## Simulation
param.sim.simlength = 1;                    % [s] model StopTime: the events of the test cases fall between 0.25 s and 0.5 s
param.sim.test_case = 1;                    % test signal, 1 to 6 (section Test signal)
param.sim.check_model = true;               % last section: param.* references of the model and From blocks without a Goto
param.sim.design_figures = true;            % section PLL design check: figures of the loops (false: numbers only)
%%
%[text] ## Timing
%[text] One clock domain: the modulator timer of the converter controls the PLLs come from runs at 4 kHz with double update, so the control period is ts = 125 us. There is no modulator in this model: ts is the sample time of the test signal (Digital Clock), of every PLL block and the fixed step of the solver (discrete, ode1). The FHT-PLL and the DDSRF-PLL read ts\_afe, the Park PLL ts\_inv: same value.
param.time.fpwm = 4e3;                      % [Hz] timer frequency of the converter controls the PLLs come from
param.time.trgo = 1;                        % double update: control at 2*fpwm
param.time.t_measure = param.sim.simlength; % [s] logging window: the whole simulation
param.glb_time = prv.timing_param(param.time);
%%
%[text] ## Grid
%[text] Nominal frequency of the voltage, 50 Hz. The PLL blocks read the nominal pulsation as param.grid.emu.omega\_grid\_nom, the name of the grid emulator of frt\_dqpll; this model has no emulator and the struct holds this number only.
param.grid.fgrid = 50;                      % [Hz]
param.grid.emu.omega_grid_nom = 2*pi*param.grid.fgrid;      % [rad/s]
%%
%[text] ## Test signal
%[text] Block test\_signal, every ts, in pu of the nominal peak: u = A(t) ( cos(phi + theta0) + sum of harm\_amp(k) sin(harm\_order(k) phi + harm\_phase(k)) ) + dc, where phi = omega\_n t with the frequency step and the phase jump, and A(t) steps from a0 to a\_sag and a\_rec. theta\_g = phi + theta0 wrapped to (-pi, pi\] is the angle of the fundamental (cosine convention, the angle the three loops lock on) and omega\_g its pulsation in pu. Cases (an event at Inf is off):
%[text] - 1: the 2021 model, 4th and 5th harmonic 10 percent, sag to 0.2 pu from 0.25 s to 0.5 s, start 180 degrees away
%[text] - 2: phase jump of +30 degrees at 0.4 s
%[text] - 3: frequency step from 50 Hz to 49 Hz at 0.4 s
%[text] - 4: DC offset of 0.02 pu from 0.4 s
%[text] - 5: odd harmonics 3rd 5 percent, 5th 6 percent, 7th 5 percent (limits of EN 50160)
%[text] - 6: sag to 0.2 pu from 0.25 s to 0.5 s, no harmonics \
[param.test, param.sim.test_name] = prv.test_param(param.sim.test_case, param.grid.emu.omega_grid_nom);
fprintf('%s init: test case %d, %s\n', model, param.sim.test_case, param.sim.test_name);
%%
%[text] ## PLLs
%[text] ### FHT-PLL and DDSRF-PLL (class ctrl\_afe\_setup)
%[text] The observers and the gains of the two loops of the AFE control come from the class, as in frt\_dqpll: first harmonic trackers with delta 0.05 and poles at \[-1 -4\] omega\_n (fht\_grid), double-integrator observer with poles at \[-1 -4\] omega\_n (di\_obs), FHT-PLL gains pll\_p\_fht 1 and pll\_i1\_fht 80 (the loop works on omega in pu: s^2 + omega\_n kp s + omega\_n ki, 25 Hz, damping 0.99), DDSRF-PLL gains 0.32 and 8 (8 Hz, damping 1: the values of frt\_dqpll, where this loop is the default) with tau\_ddsrf = 1/omega\_n for the four decoupling low passes. The DDSRF-PLL clamps its frequency integrator to \[0.5 1.5\] pu (Saturation block).
param.afe.ctrl = ctrl_afe_setup(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom);
param.afe.ctrl.pll_p_ddsrt  = 0.32;
param.afe.ctrl.pll_i1_ddsrt = 8;
%[text] Option: first harmonic trackers on the undamped oscillator with the exact discretization Ad = expm(A ts). The damping delta = 0.05 of the class model leaves a quadrature error at 50 Hz: the positive sequence of one tracker leads the input by about 0.7 degrees and a negative sequence of 1.4 percent remains (2 omega\_n ripple), so the DDSRF-PLL locks 0.7 degrees ahead and the FHT-PLL, with three trackers, 1.5 degrees ahead. With delta = 0 the error at 50 Hz is zero (see the PLL design check and the note).
param.afe.pll.fht_undamped = false;         % true: FHT on the undamped oscillator, exact discretization
if param.afe.pll.fht_undamped
    A0 = [0 1; -param.afe.ctrl.fht_grid.omega_fht^2 0];
    param.afe.ctrl.fht_grid.delta_fht = 0;
    param.afe.ctrl.fht_grid.Afht = A0;
    param.afe.ctrl.fht_grid.Ad_fht = expm(A0*param.glb_time.ts_afe);
    param.afe.ctrl.fht_grid.Ld_fht = acker(param.afe.ctrl.fht_grid.Ad_fht', param.afe.ctrl.fht_grid.Cfht', ...
        exp(param.glb_time.ts_afe*[-1 -4]*param.afe.ctrl.fht_grid.omega_fht))';
    clear A0
end
%[text] ### Park PLL (block single\_phase\_pll)
%[text] Low-pass filters of the quadrature generator with tau = 2/omega\_n (the generator is a SOGI with gain k = 1/(omega\_n tau) = 0.5), PI block pi-ctrl with kp = 1 and ki = 0 (a proportional loop: 25 Hz and damping 0.75 at 1 pu, a frequency step of 1 Hz leaves a phase error of 1.15 degrees) and its output saturated at +-2 pu around the nominal frequency. The error eta is not normalized: at 0.2 pu the loop gain is five times lower.
param.afe.pll.park.tau = 2/param.grid.emu.omega_grid_nom;   % [s] low passes uxi_pll_flt and ueta_pll_flt
param.afe.pll.park.kp = 1;                  % [pu omega / pu eta]
param.afe.pll.park.ki = 0;                  % [1/s]
param.afe.pll.park.sat_pos = 2;             % [pu] output limits of pi-ctrl (added to the nominal 1 pu)
param.afe.pll.park.sat_neg = -2;
%%
%[text] ## PLL design check
%[text] Linear models of the three loops with the gains above: the Park PLL with its SOGI-equivalent generator and the phase detector (s + wf)/(2 s + wf) at 1 pu and at the sag of the test, the frequency response of the first harmonic tracker (quadrature error, positive and negative sequence, harmonics), the stage B loops of the FHT-PLL (the trackers do not see the PLL angle: they are outside the loop) and of the DDSRF-PLL (its decoupling network is inside the loop: crossover 103 rad/s, phase margin 57 degrees), the double-integrator observer. Numbers in param.afe.design, figures when param.sim.design\_figures is true.
%[text] The design note checks these models against the simulation and proposes three changes, not applied here (the blocks stay as in the controls they come from): ki = 20 for the Park PLL (no phase error after a frequency step, phase margin 55 degrees at 0.2 pu), fht\_undamped = true (no offset at 50 Hz, half the error at 49 Hz), the frequency integrator of the DDSRF-PLL started at 1 pu (lock in 0 to 135 ms instead of 94 to 180 ms).
param.afe.design = prv.pll_design(param, options);
%%
%[text] ## Model setup
open_system(model);
%[text] Scopes closed at start.
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%%
%[text] ## Check of the model
%[text] Every param.\* read by an active block must exist, and every From block of the active part of the model must have a Goto.
if param.sim.check_model
    [missing_refs, model_refs] = prv.check_model_param_refs(model, param);
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the model blocks resolve\n', model, numel(model_refs));
    else
        warning('%s init: %d param.* references of the model blocks do not exist in param:', model, numel(missing_refs));
        disp(missing_refs);
    end
    lost = prv.check_from_goto(model);
    if isempty(lost)
        fprintf('%s init: every active From block has its Goto\n', model);
    else
        warning('%s init: %d tags read by active From blocks have no active Goto:\n  %s', model, numel(lost), ...
            strjoin(lost, '\n  '));
    end
    clear missing_refs model_refs lost
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
