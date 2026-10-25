function build_interleaved_buck_compact()
%BUILD_INTERLEAVED_BUCK_COMPACT  Build or update interleaved_buck_compact.slx: the compact block of the library
%   (interleaved_buck_3legs/interleaved_buck_3legs_mod1: power stage as a Simscape language component, control and
%   interleaved PWM as one MATLAB Function) with the battery and the load of interleaved_buck.slx.
%
%   build_interleaved_buck_compact
%
%   Run init_model first (param in the base workspace: the block reads param.buck as its mask struct p, the
%   battery param.battery.pack_1, the load param.load.R_load, the solver param.glb_time.tc), with the library on
%   the path and the compact block built (cd library/foundation/pwr_electronics; ssc_build buck_ssc;
%   cd ../../masked_models/interleaved_buck_3legs; update_interleaved_buck_3legs). When interleaved_buck_compact.slx
%   does not exist it is created next to this file, otherwise it is loaded and refreshed: blocks and lines are
%   added only when missing, the parameters are set again. Then simulate it as the discrete model:
%
%       init_model; build_interleaved_buck_compact; sim('interleaved_buck_compact');
%       idx = interleaved_buck_3legs_log_index();
%       plot(log_buck.Time, log_buck.Data(:, [idx.i_L1 idx.i_L2 idx.i_L3 idx.i_leg_ref])); grid on
%
%   Model: Clock -> t_master of the block; battery (lithium_ion_battery/lithium_ion_battery of the library, same
%   parameters as the battery subsystem of interleaved_buck.slx) on dc+/dc- with the Electrical Reference on dc-;
%   resistor R_load on out+/out-; To Workspace meas_buck (m of the block, 10x1 pu, timeseries at tc) and log_buck
%   (ctrl_log, 30x1, indices in interleaved_buck_3legs_log_index.m), battery_data (bus of the battery block);
%   Solver Configuration with the local solver at param.glb_time.tc as the discrete model; ode23tb, MaxStep tc.
%   The same param.buck drives both models; the differences are listed in the header of
%   library/masked_models/interleaved_buck_3legs/private/interleaved_buck_3legs_ctrl_pwm.m (no loss and thermal
%   model of the switches, back-calculation anti-windup instead of the clamped integrators, no 1/(tc*s + 1)
%   filters on the duties, DC link started at Vdc0 instead of charged by the battery).

    model = 'interleaved_buck_compact';
    here  = fileparts(mfilename('fullpath'));
    assert(evalin('base', 'exist(''param'', ''var'') == 1'), 'build_interleaved_buck_compact: run init_model first (param not found in the base workspace)');
    assert(exist('interleaved_buck_3legs', 'file') == 4, ['build_interleaved_buck_compact: interleaved_buck_3legs.slx not on the path: ' ...
        'build it first (cd library/foundation/pwr_electronics; ssc_build buck_ssc; cd ../../masked_models/interleaved_buck_3legs; update_interleaved_buck_3legs)']);
    assert(exist('lithium_ion_battery', 'file') == 4, 'build_interleaved_buck_compact: lithium_ion_battery.slx of the library not on the path');
    load_system('interleaved_buck_3legs');
    load_system('lithium_ion_battery');

    created = exist(model, 'file') ~= 4;
    if created
        new_system(model);
    else
        load_system(model);
    end
    set_param(model, 'StopTime', 'param.sim.simlength', 'SolverType', 'Variable-step', 'Solver', 'ode23tb', ...
        'MaxStep', 'param.glb_time.tc', 'ZeroCrossControl', 'DisableAll', 'SaveTime', 'off', 'SaveOutput', 'off', ...
        'SimscapeLogType', 'none');

    % ---- converter: the compact block with param.buck as mask struct
    buck = add_once(model, 'interleaved_buck_mod1', 'interleaved_buck_3legs/interleaved_buck_3legs_mod1', 'Position', [330 95 510 215]);
    set_param(buck, 'p', 'param.buck');
    ph_b = get_param(buck, 'PortHandles');
    assert(numel(ph_b.LConn) == 2 && numel(ph_b.RConn) == 2 && numel(ph_b.Inport) == 1 && numel(ph_b.Outport) == 2, ...
        'build_interleaved_buck_compact: unexpected ports on %s', getfullname(buck));

    % ---- battery on the DC link (same mask values as battery/lithium-ion_battery of interleaved_buck.slx)
    batt = add_once(model, 'battery', 'lithium_ion_battery/lithium_ion_battery', 'Position', [60 80 225 230]);
    bp = {'Q', 'number_of_cells', 'E_1', 'E0', 'E1', 'E2', 'E3', 'Elog', 'R0', 'R1', 'C1', 'M', 'alpha', 'soc_init', ...
          'q1Kalman', 'q2Kalman', 'q3Kalman', 'rKalman'};
    for k = 1:numel(bp)
        set_param(batt, bp{k}, ['param.battery.pack_1.' bp{k}]);
    end
    set_param(batt, 'ts_battery', 'param.glb_time.ts_dab', 'tc', 'param.glb_time.tc', 'enable_white_noise_m', '0', ...
        'battery_voltage_init_m', 'param.battery.nominal_battery_voltage_1');
    ph_bt = get_param(batt, 'PortHandles');
    assert(numel(ph_bt.RConn) == 2, 'build_interleaved_buck_compact: the battery block has %d right physical ports, 2 expected (+, -)', numel(ph_bt.RConn));

    % ---- reference, solver, load, clock, logging
    eref = add_once(model, 'Electrical Reference', 'fl_lib/Electrical/Electrical Elements/Electrical Reference', 'Position', [270 290 290 310]);
    solv = add_once(model, 'Solver Configuration', 'nesl_utility/Solver Configuration', 'Position', [250 20 295 52]);
    set_param(solv, 'UseLocalSolver', 'on', 'LocalSolverChoice', 'NE_BACKWARD_EULER_ADVANCER', ...
        'LocalSolverSampleTime', 'param.glb_time.tc', 'DoFixedCost', 'on', 'MaxNonlinIter', 'param.sim.nonlinear_iterations');
    rload = add_once(model, 'R_load', 'fl_lib/Electrical/Electrical Elements/Resistor', 'Position', [600 130 640 170], 'Orientation', 'down');
    set_param(rload, 'R', 'param.load.R_load');
    clk  = add_once(model, 'Clock', 'simulink/Sources/Clock', 'Position', [250 160 270 180]);
    twm  = add_once(model, 'meas_buck', 'simulink/Sinks/To Workspace', 'Position', [560 220 640 250]);
    set_param(twm, 'VariableName', 'meas_buck', 'SampleTime', 'param.glb_time.tc', 'Decimation', 'param.glb_time.decimation_tc', 'SaveFormat', 'Timeseries');
    twl  = add_once(model, 'log_buck', 'simulink/Sinks/To Workspace', 'Position', [560 265 640 295]);
    set_param(twl, 'VariableName', 'log_buck', 'SampleTime', 'param.glb_time.tc', 'Decimation', 'param.glb_time.decimation_tc', 'SaveFormat', 'Timeseries');
    twb  = add_once(model, 'battery_data', 'simulink/Sinks/To Workspace', 'Position', [60 265 140 295]);
    set_param(twb, 'VariableName', 'battery_data', 'SampleTime', 'param.glb_time.tc', 'Decimation', 'param.glb_time.decimation_tc', 'SaveFormat', 'Timeseries');
    scp  = add_once(model, 'm', 'simulink/Sinks/Scope', 'Position', [560 40 590 70]);
    set_param(scp, 'Open', 'off');

    % ---- lines
    ph_r = get_param(rload, 'PortHandles');
    ph_e = get_param(eref, 'PortHandles');
    ph_s = get_param(solv, 'PortHandles');
    link_once(model, port1(clk, 'Outport'), ph_b.Inport(1));                  % t_master
    link_once(model, ph_b.Outport(1), port1(twm, 'Inport'));                  % m -> meas_buck
    link_once(model, ph_b.Outport(1), port1(scp, 'Inport'));                  % m -> scope
    link_once(model, ph_b.Outport(2), port1(twl, 'Inport'));                  % ctrl_log -> log_buck
    link_once(model, ph_bt.Outport(1), port1(twb, 'Inport'));                 % battery bus -> battery_data
    link_once(model, ph_bt.RConn(1), ph_b.LConn(1));                          % battery + -> dc+
    link_once(model, ph_bt.RConn(2), ph_b.LConn(2));                          % battery - -> dc-
    link_once(model, ph_e.LConn(1),  ph_b.LConn(2));                          % reference on dc-
    link_once(model, ph_s.RConn(1),  ph_b.LConn(1));                          % solver on the network
    link_once(model, ph_b.RConn(1),  ph_r.LConn(1));                          % out+ -> load
    link_once(model, ph_b.RConn(2),  ph_r.RConn(1));                          % out- -> load

    if created
        save_system(model, fullfile(here, [model '.slx']));
    else
        save_system(model);
    end
    fprintf('%s built: %s\n', model, get_param(model, 'FileName'));
end

% ------------------------------------------------------------------------------------------------------
function h = add_once(sys, name, source, varargin)
%ADD_ONCE  Handle of the block sys/name, added from source with the given parameters when missing.
    path = [sys '/' name];
    if isempty(find_system(sys, 'SearchDepth', 1, 'Name', name))
        h = add_block(source, path, varargin{:});
    else
        h = get_param(path, 'Handle');
    end
end

% ------------------------------------------------------------------------------------------------------
function h = port1(blk_h, kind)
%PORT1  First port of the given kind ('Inport' or 'Outport') of a block handle.
    ph = get_param(blk_h, 'PortHandles');
    h = ph.(kind)(1);
end

% ------------------------------------------------------------------------------------------------------
function link_once(sys, src_h, dst_h)
%LINK_ONCE  add_line(sys, src_h, dst_h) unless the destination port already has a line (signal or physical).
    if get_param(dst_h, 'Line') == -1
        add_line(sys, src_h, dst_h, 'autorouting', 'on');
    end
end
