function [Dn, Du, info] = device_lm_params(d, u)
% DEVICE_LM_PARAMS  Device data -> RR Diode (Lauritzen-Ma) parameters of ONE
% device, for diode_rr blocks placed by hand (one block per device).
%
%   [Dn, Du, info] = device_lm_params(d, u)
%
%   d  : diode struct of init_model.m (vf_hot, If0_ref, Qrr_ref, Irrm_ref,
%        di_dt_ref, Tj, N, Cj, roff, optional tau_override)
%   u  : Qrr unbalance, scalar or vector (default 0). Du(k) has Qrr*(1 - u(k)),
%        i.e. tau and TM times sqrt(1 - u(k)) with the same forward
%        characteristic, like level 1 of crowbar_stack_rr. u > 0: less
%        stored charge, the device recovers first; u < 0: more charge
%        (u = [0.05 -0.05] for a +-5 % spread).
%
%   Dn, Du : diode_rr block parameters, SI units, block names:
%            Is, N, Vt, tau, TM, Rs, Rr, Cj0, Vjp, mg, FC, dsm
%   info   : Vf0, rt, vf_I0 (tangent of vf_hot at If0_ref), t_tail,
%            Qrr_longramp (a*tau^2 at di/dt = di_dt_ref), u
%
%   Same conversion as stack_lm_params.m with np = 1:
%     Reverse recovery, one point at di/dt = a (long-ramp limit):
%       t_tail = Qrr/Irrm - Irrm/(2a),  tau = t_tail + Irrm/a,
%       TM = tau*t_tail*a/Irrm
%     If d.tau_override > 0, tau and TM are scaled to it (same TM/tau).
%     On-state: the max characteristic at Tvj max (vf_hot) is replaced by
%     its tangent at If0_ref (Vf0 + rt*I), matched by the exponential
%     junction plus Rs:  Rs = rt - N*Vt/If0,  vj = Vf0 + N*Vt,
%       Is = If0/(exp(vj/(N*Vt)) - 1)*(tau + TM)/tau
%     If0_ref is the current of ONE device at turn-off (init_model.m sets
%     165 kA/np); with another number of devices in parallel, set it
%     before the call.
%   Blocking: Rr = roff of one device; Cj0 = Cj with mg = 0 (constant
%   junction capacitance). Vjp, FC and dsm as in the FAIST model: with
%   mg = 0, Vjp and FC have no effect.
%   Initial state of each block: reverse biased like the stack levels,
%   vj = vsn0 (= -V0/ns in init_model.m).
%
%   Note: the D3001N68T datasheet point is at IFM = 1 kA, 10 A/us, so the
%   ramp (IFM + Irrm)/a = 130 us is only about 2.5 tau: the long-ramp fit
%   is approximate there (5SDD 33L5500: 4 kA, 434 us, about 9 tau).
%
%   Example, after init_model:
%     [Dn, Du] = device_lm_params(diode, u_unbal);
%     % mask of each diode_rr block: Is = Dn.Is, N = Dn.N, Vt = Dn.Vt,
%     % tau = Dn.tau, TM = Dn.TM, Rs = Dn.Rs, Rr = Dn.Rr, Cj0 = Dn.Cj0,
%     % mg = 0; the unbalanced device takes Du instead of Dn.

if nargin < 2 || isempty(u), u = 0; end
assert(all(u < 1), 'u must be < 1.');

% k_q = k/q [V/K] Questo valore è noto anche come l'equivalente in elettronvolt 
% per Kelvin (\(8,617333 \times 10^{-5}\text{ eV/K}\)), molto utilizzato 
% in fisica dei semiconduttori e termodinamica per convertire la temperatura 
% in energia termica (\(k_{B}T\)

k_q = 1.380649e-23/1.602176634e-19;     
Vt  = k_q*(d.Tj + 273.15);
N   = d.N;
a   = d.di_dt_ref;

% reverse recovery from the datasheet point
Qrr = d.Qrr_ref;  Irrm = d.Irrm_ref;
t_tail = Qrr/Irrm - Irrm/(2*a);
assert(t_tail > 0, 'No Lauritzen-Ma fit: Irrm >= trr*di/dt.');
tau = t_tail + Irrm/a;
TM  = tau*t_tail*a/Irrm;
if isfield(d, 'tau_override') && d.tau_override > 0
    TM  = TM*d.tau_override/tau;
    tau = d.tau_override;
end

% tangent of the max on-state characteristic at If0_ref (one device)
c  = d.vf_hot;  I0 = d.If0_ref;
vf = c(1) + c(2)*I0 + c(3)*log(I0 + 1) + c(4)*sqrt(I0);
rt = c(2) + c(3)/(I0 + 1) + c(4)/(2*sqrt(I0));
Vf0 = vf - rt*I0;

Rs = rt - N*Vt/I0;
assert(Rs > 0, 'rt too small for N = %g.', N);
vj = Vf0 + N*Vt;

Dn = one_set(tau, TM);
Du = repmat(Dn, 1, numel(u));
for k = 1:numel(u)
    Du(k) = one_set(tau*sqrt(1 - u(k)), TM*sqrt(1 - u(k)));
end

info = struct('Vf0', Vf0, 'rt', rt, 'vf_I0', vf, 't_tail', t_tail, ...
              'Qrr_longramp', a*tau^2, 'u', u);

if nargout == 0
    fprintf('%s, one device: tau = %.1f us, TM = %.1f us, Is = %.3g A, Rs = %.3f mOhm, Rr = %.2f MOhm, Cj0 = %.1f nF\n', ...
        d.name, tau*1e6, TM*1e6, Dn.Is, Rs*1e3, d.roff/1e6, d.Cj*1e9);
    fprintf('  forward fit at %.1f kA: vf = %.2f V, Vf0 = %.3f V, rt = %.3f mOhm\n', ...
        I0/1e3, vf, Vf0, rt*1e3);
    for k = 1:numel(u)
        fprintf('  u = %+.2f: tau = %.1f us, TM = %.1f us\n', u(k), Du(k).tau*1e6, Du(k).TM*1e6);
    end
end

    function P = one_set(tau_k, TM_k)
        Is = I0/(exp(vj/(N*Vt)) - 1)*(tau_k + TM_k)/tau_k;
        P = struct('Is', Is, 'N', N, 'Vt', Vt, 'tau', tau_k, 'TM', TM_k, ...
                   'Rs', Rs, 'Rr', d.roff, 'Cj0', d.Cj, 'Vjp', 0.7, 'mg', 0, ...
                   'FC', 0.5, 'dsm', 10e-3);
    end
end
