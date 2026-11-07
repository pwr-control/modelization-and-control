function uvw = afe_uvw_param(param)
%AFE_UVW_PARAM  Parameter struct of the library blocks afe_uvw_ctrl used by frt_dqpll (mask parameter p = param.afe.uvw).
%
%   uvw = afe_uvw_param(param)   reads param.glb_time, param.grid, param.afe.ctrl, param.frt.data and param.ctrl
%
%   The blocks of the library package masked_models/afe_uvw_ctrl in the model: grid_voltage_sequences
%   (afe_control/grid_voltage_sequences), frt_vde4110 (FRT-VDE4110), grid_current_sequences_q_ctrl
%   (grid_current_sequences_and_reactive_pwr_ctrl) and islanding_detector (islanding_mode_detector). The field
%   names are those of afe_uvw_ctrl_param_example.m of the library (see the README of the package); the group
%   uvw.ictrl is for the block current_ctrl_uvw, which this model does not have, and is kept so that the struct is
%   the complete one of the example. Same values as afe_uvw_three_phase/init_model.m (section "AFE control in uvw").
%   The blocks inherit the sample time of the triggered subsystem afe_control: uvw.ts must be its period.

    c = param.afe.ctrl;
    uvw = struct();
    uvw.ts        = param.glb_time.ts_afe;              % [s] control period: integration step of all blocks
    uvw.omega_nom = param.grid.emu.omega_grid_nom;      % [rad/s] nominal grid pulsation: base of omega_pu

    % grid voltage sequences: xi-eta of the positive and negative sequence, moving average
    uvw.vseq.avg_order    = param.ctrl.mavarage_filter_frequency_base_order;  % window = grid period/avg_order (2: half period)
    uvw.vseq.omega_pu_min = 0.1;                        % [pu] limits of the measured pulsation used for the window
    uvw.vseq.omega_pu_max = 2;
    uvw.vseq.t_enable     = param.ctrl.dmavg_filter_enable_time;   % [s] averages are zero before this time

    % grid current sequences and reactive power control: PIs on the averaged sequences
    uvw.iseq.avg_order    = uvw.vseq.avg_order;
    uvw.iseq.omega_pu_min = uvw.vseq.omega_pu_min;
    uvw.iseq.omega_pu_max = uvw.vseq.omega_pu_max;
    uvw.iseq.t_enable     = uvw.vseq.t_enable;
    uvw.iseq.kp_pos       = c.kp_rc_pos_grid;           % PI of the positive-sequence eta grid current
    uvw.iseq.ki_pos       = c.ki_rc_pos_grid;           % [1/s]
    uvw.iseq.kp_neg       = c.kp_rc_neg_grid;           % PIs of the negative-sequence xi and eta grid currents
    uvw.iseq.ki_neg       = c.ki_rc_neg_grid;           % [1/s]
    uvw.iseq.lim_pos_eta  = c.ieta_pos_ref_lim;         % [pu] limits of the stack current references
    uvw.iseq.lim_neg_xi   = c.ieta_pos_ref_lim;         % [pu] (the old subsystem used the positive limit here too)
    uvw.iseq.lim_neg_eta  = c.ieta_neg_ref_lim;         % [pu]
    uvw.iseq.err_sign     = -1;                         % error = -(reference - measured): grid current measured towards the converter

    % FRT (VDE-AR-N 4110): voltage-rate observers, fault detection, power limitation, reactive current references
    uvw.frt.obs_kx        = c.vr_obs.kx;                % gains of the voltage-rate observers of ctrl_afe_setup (poles [-0.1 -0.4]*omega_base)
    uvw.frt.obs_kv        = c.vr_obs.kv;
    uvw.frt.t_obs_release = 0.025;                      % [s] observers held at zero before this time
    uvw.frt.v_pos_low     = 0.85;                       % [pu] fault when the positive sequence is below this ...
    uvw.frt.v_pos_high    = 1.15;                       % [pu] ... or above this ...
    uvw.frt.v_neg_max     = 0.1;                        % [pu] ... or the negative sequence is above this
    uvw.frt.v_hyst        = 0.1;                        % [pu] band narrowing to leave the fault
    uvw.frt.v_pos_dot_max = 2.0;                        % [pu/s] the voltage must also change faster than this
    uvw.frt.v_neg_dot_max = 2.0;                        % [pu/s]
    uvw.frt.t_rearm       = 0.125;                      % [s] idle time needed before a new detection
    uvw.frt.t_hold_max    = 7.625;                      % [s] a fault longer than this ends in the state EXCEEDED
    uvw.frt.pwrlim_v_pos  = 0.8;                        % [pu] active power limited (pwr_lim_enable) when v_pos < this ...
    uvw.frt.pwrlim_v_neg  = 0.1;                        % [pu] ... or v_neg > this
    uvw.frt.pwrlim_hyst   = 0.1;                        % [pu] released when v_pos > pwrlim_v_pos + hyst ...
    uvw.frt.pwrlim_v_neg_release = 0.7;                 % [pu] ... and v_neg < this
    uvw.frt.reddyn_enabled = 0;                         % reduced dynamic grid support (off, as in the old subsystem)
    uvw.frt.reddyn_v_pos  = 0.7;                        % [pu]
    uvw.frt.reddyn_v_neg  = 0.2;                        % [pu]
    uvw.frt.k_frt_ref     = param.frt.data.k_frt_ref;   % [pu/pu] reactive current per voltage change
    uvw.frt.t_avg_prefault = 0.5;                       % [s] window of the pre-fault voltage averages
    uvw.frt.t_avg_start   = 0.05;                       % [s] the pre-fault averages start here
    uvw.frt.lim_pos_eta   = c.i_grid_pos_eta_lim;       % [pu] limits of the grid current references
    uvw.frt.lim_neg_xi    = c.i_grid_neg_xi_lim;
    uvw.frt.lim_neg_eta   = c.i_grid_neg_eta_lim;
    uvw.frt.lim_pos_eta_red = c.i_grid_pos_eta_red_lim; % [pu] the same with reduced dynamic support
    uvw.frt.lim_neg_xi_red  = c.i_grid_neg_xi_red_lim;
    uvw.frt.lim_neg_eta_red = c.i_grid_neg_eta_red_lim;
    uvw.frt.f_lpf_ref     = 161;                        % [Hz] low pass of the references

    % current control in uvw (block current_ctrl_uvw, not in this model): resonant PI per phase, zero-sequence PI
    uvw.ictrl.kp        = c.res_pi.kp_rpi;              % resonant PI kp + ki*s/(s^2 + 2*delta*w*s + w^2)
    uvw.ictrl.ki        = c.res_pi.ki_rpi;              % [1/s]
    uvw.ictrl.delta_res = 0.005;                        % damping of the resonator (forward-backward Euler in the library block)
    uvw.ictrl.omega_res = c.omega_base;                 % [rad/s] resonance pulsation
    uvw.ictrl.mag_lim   = 1;                            % [pu] the alpha/beta output vector is scaled down above this length
    uvw.ictrl.mag_scale_max = 10;                       % largest scaling factor
    uvw.ictrl.out_delay = 1;                            % 1: alpha/beta output one step late, 0: no delay
    uvw.ictrl.kp_z      = c.kp_is_z;                    % zero-sequence current PI
    uvw.ictrl.ki_z      = c.ki_is_z;                    % [1/s]
    uvw.ictrl.u_z_lim   = 1;                            % [pu] limit of u_gamma
    uvw.ictrl.iz_ref    = 0;                            % [pu] zero-sequence current reference

    % islanding detector (passive): U<, U>, f<, f>, ROCOF and 5th/7th harmonic criteria with on-delays.
    % Thresholds of the network protection of VDE-AR-N 4105 / 4110 for a 50 Hz grid; the slow U< stage
    % (2.7 s) lets the converter ride through a grid fault (frt_vde4110) without an islanding trip.
    uvw.island.h_avg_order  = 1;                        % harmonic averaging window = grid period/h_avg_order
    uvw.island.omega_pu_min = 0.5;                      % [pu] limits of the measured pulsation used for the window
    uvw.island.omega_pu_max = 1.5;
    uvw.island.t_enable     = 0.1;                      % [s] no detection before this time
    uvw.island.tau_f        = 0.02;                     % [s] low pass of the frequency
    uvw.island.obs_poles    = [-20 -80];                % [rad/s] poles of the ROCOF observer (double integrator, as vr_obs)
    Ad = [1 uvw.ts; 0 1];
    L = acker(Ad', [1 0]', exp(uvw.island.obs_poles*uvw.ts))';
    uvw.island.obs_kx = L(1)/uvw.ts;
    uvw.island.obs_kv = L(2)/uvw.ts;
    uvw.island.u_valid      = 0.5;                      % [pu] frequency and harmonic criteria only above this voltage
    uvw.island.u_min        = 0.8;                      % [pu] under-voltage ...
    uvw.island.t_trip_u_min = 2.7;                      % [s] ... for this time
    uvw.island.u_max        = 1.15;                     % [pu] over-voltage ...
    uvw.island.t_trip_u_max = 0.1;                      % [s] ... for this time
    uvw.island.f_min        = 47.5/param.grid.fgrid;    % [pu] under-frequency (47.5 Hz) ...
    uvw.island.f_max        = 51.5/param.grid.fgrid;    % [pu] over-frequency (51.5 Hz) ...
    uvw.island.t_trip_f     = 0.1;                      % [s] ... for this time
    uvw.island.rocof_max    = 2.5;                      % [Hz/s] rate of change of frequency ...
    uvw.island.t_trip_rocof = 0.1;                      % [s] ... for this time
    uvw.island.h_max        = 0.1;                      % [pu/pu] sqrt(u5^2 + u7^2)/u1 above this ...
    uvw.island.t_trip_h     = 0.1;                      % [s] ... for this time
    uvw.island.latch        = 1;                        % the output stays 1 until enable goes to 0
end
