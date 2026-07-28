function m = three_phase_power_meter_iec(phase, u_abc, i_abc, omega, ts, enable_time)
%#codegen
% THREE_PHASE_POWER_METER_IEC
% Equivalente in singolo blocco MATLAB Function del sottosistema Simulink
% "three_phase_power_meter_iec" (libreria power_meter_iec).
%
% INGRESSI
%   phase       : angolo del PLL [rad]      (scalare)
%   u_abc       : tensioni di fase [V]      (3x1)
%   i_abc       : correnti di linea [A]     (3x1)
%   omega       : pulsazione di rete [rad/s] (scalare)
%   ts          : passo di campionamento [s]  -> parametro di maschera
%   enable_time : istante di abilitazione [s] -> parametro di maschera
%
% USCITA
%   m : struct (bus) con i campi, nello stesso ordine del Bus Creator
%       originale: Q1p, Q1n, P1n, P1p, U1n, U1p,
%                  i_act_1n, i_react_1n, i_act_1p, i_react_1p
%
% CATENA DI ELABORAZIONE (identica all'originale)
%   1) proiezione di ogni fase su cos(phase) e sin(phase), guadagno 2
%   2) media mobile dinamica su finestra pari a mezzo periodo di rete
%      (Tw = pi/|omega|), con lunghezza frazionaria interpolata
%   3) componenti simmetriche di sequenza diretta e inversa (Fortescue)
%   4) potenze attiva e reattiva per sequenza, tensioni concatenate RMS,
%      correnti attiva e reattiva per sequenza
%
% NOTE
%   - Il blocco deve girare a passo fisso ts (gli Zero-Order Hold di
%     ingresso dell'originale sono impliciti nel sample time del blocco).
%   - ts ed enable_time possono essere dichiarati come dati di scope
%     "Parameter" nel Symbols pane del MATLAB Function block, cosi'
%     risolvono direttamente sui parametri di maschera e la firma
%     esterna resta a 4 ingressi come nel blocco originale.

% ------------------------------------------------------------------
% Costanti
% ------------------------------------------------------------------
MAVGFLT_SIZE_MAX = 17500;   % lunghezza buffer circolare (come originale)
NCH              = 12;      % canali filtrati: 6 per u_abc, 6 per i_abc

% Ordine canali:
%   1..3  ua_cos ub_cos uc_cos      7..9   ia_cos ib_cos ic_cos
%   4..6  ua_sin ub_sin uc_sin     10..12  ia_sin ib_sin ic_sin

% ------------------------------------------------------------------
% Stato persistente
% ------------------------------------------------------------------
persistent buf cb_index sample_count sum_short sum_long prev_input ...
           k_step period_held

if isempty(buf)
    buf          = zeros(NCH, MAVGFLT_SIZE_MAX);
    cb_index     = ones(NCH, 1);
    sample_count = ones(NCH, 1);
    sum_short    = zeros(NCH, 1);
    sum_long     = zeros(NCH, 1);
    prev_input   = zeros(NCH, 1);
    k_step       = 0;
    period_held  = 0;          % uscita iniziale dell'Enabled Subsystem
end

% ------------------------------------------------------------------
% Abilitazione (equivalente al blocco Step con Time = enable_time)
% ------------------------------------------------------------------
t_now  = k_step * ts;
k_step = k_step + 1;
enable = (t_now >= enable_time);

% ------------------------------------------------------------------
% Finestra di media: Enabled Subsystem 2*pi/w, alimentato da 2*|omega|.
% Quando |omega| = 0 l'Enabled Subsystem mantiene l'ultimo valore.
% ------------------------------------------------------------------
w2 = 2 * abs(omega);
if w2 > 0
    period_held = 2*pi / w2;   % = pi/|omega|, mezzo periodo di rete
end
period = period_held;

% ------------------------------------------------------------------
% Proiezione sin/cos, guadagno 2
% ------------------------------------------------------------------
c = cos(phase);
s = sin(phase);

x = zeros(NCH, 1);
x(1)  = 2 * u_abc(1) * c;   x(2)  = 2 * u_abc(2) * c;   x(3)  = 2 * u_abc(3) * c;
x(4)  = 2 * u_abc(1) * s;   x(5)  = 2 * u_abc(2) * s;   x(6)  = 2 * u_abc(3) * s;
x(7)  = 2 * i_abc(1) * c;   x(8)  = 2 * i_abc(2) * c;   x(9)  = 2 * i_abc(3) * c;
x(10) = 2 * i_abc(1) * s;   x(11) = 2 * i_abc(2) * s;   x(12) = 2 * i_abc(3) * s;

% ------------------------------------------------------------------
% Banco di 12 filtri a media mobile dinamica (dmavgflt_process)
% ------------------------------------------------------------------
y = zeros(NCH, 1);

if enable && period > 0

    desired = period / ts;
    if desired > (MAVGFLT_SIZE_MAX - 1)
        desired = MAVGFLT_SIZE_MAX - 1;
    end

    n_samples = floor(desired + 1);          % campioni che coprono il periodo
    frac      = desired + 1 - n_samples;     % parte frazionaria
    if n_samples < 1
        n_samples = 1;
    end

    for ch = 1:NCH

        ci = cb_index(ch);
        sc = sample_count(ch);

        if n_samples > sc
            % finestra in crescita: aggiungi campioni
            sum_short(ch) = sum_short(ch) + prev_input(ch);
            sum_long(ch)  = sum_long(ch)  + x(ch);
            sc = sc + 1;

        elseif n_samples < sc
            % finestra in contrazione: rimuovi i due campioni piu' vecchi
            i1 = mod(ci - 1 - sc,       MAVGFLT_SIZE_MAX) + 1;
            i2 = mod(ci - 1 - (sc - 1), MAVGFLT_SIZE_MAX) + 1;
            old_sum = buf(ch, i1) + buf(ch, i2);

            sum_short(ch) = sum_short(ch) + prev_input(ch) - old_sum;
            sum_long(ch)  = sum_long(ch)  + x(ch)          - old_sum;
            sc = sc - 1;

        else
            % finestra costante
            i1     = mod(ci - 1 - sc, MAVGFLT_SIZE_MAX) + 1;
            oldest = buf(ch, i1);

            sum_short(ch) = sum_short(ch) + prev_input(ch) - oldest;
            sum_long(ch)  = sum_long(ch)  + x(ch)          - oldest;
        end

        % aggiornamento buffer circolare
        buf(ch, ci)      = x(ch);
        cb_index(ch)     = mod(ci, MAVGFLT_SIZE_MAX) + 1;
        prev_input(ch)   = x(ch);
        sample_count(ch) = sc;

        % uscita: interpolazione lineare fra finestra corta e lunga
        if sc > 1
            avg_short = sum_short(ch) / (sc - 1);
        else
            avg_short = 0;
        end
        avg_long = sum_long(ch) / sc;

        y(ch) = avg_short * (1 - frac) + avg_long * frac;
    end
end
% se enable = false oppure period <= 0: y resta 0 e lo stato non evolve
% (comportamento identico all'originale)

ua_cos = y(1);  ub_cos = y(2);  uc_cos = y(3);
ua_sin = y(4);  ub_sin = y(5);  uc_sin = y(6);
ia_cos = y(7);  ib_cos = y(8);  ic_cos = y(9);
ia_sin = y(10); ib_sin = y(11); ic_sin = y(12);

% ------------------------------------------------------------------
% Componenti simmetriche (fasori con convenzione U = u_cos - j*u_sin)
% ------------------------------------------------------------------
sq3 = sqrt(3);

u1p_cos = (2*ua_cos - ub_cos - uc_cos - sq3*(uc_sin - ub_sin)) / 6;
u1p_sin = (2*ua_sin - ub_sin - uc_sin - sq3*(ub_cos - uc_cos)) / 6;
u1n_cos = (2*ua_cos - ub_cos - uc_cos - sq3*(ub_sin - uc_sin)) / 6;
u1n_sin = (2*ua_sin - ub_sin - uc_sin - sq3*(uc_cos - ub_cos)) / 6;

i1p_cos = (2*ia_cos - ib_cos - ic_cos - sq3*(ic_sin - ib_sin)) / 6;
i1p_sin = (2*ia_sin - ib_sin - ic_sin - sq3*(ib_cos - ic_cos)) / 6;
i1n_cos = (2*ia_cos - ib_cos - ic_cos - sq3*(ib_sin - ic_sin)) / 6;
i1n_sin = (2*ia_sin - ib_sin - ic_sin - sq3*(ic_cos - ib_cos)) / 6;

% ------------------------------------------------------------------
% Potenze per sequenza
% ------------------------------------------------------------------
P1p = 1.5 * (u1p_cos*i1p_cos + u1p_sin*i1p_sin);
Q1p = 1.5 * (u1p_cos*i1p_sin - u1p_sin*i1p_cos);
P1n = 1.5 * (u1n_cos*i1n_cos + u1n_sin*i1n_sin);
Q1n = 1.5 * (u1n_cos*i1n_sin - u1n_sin*i1n_cos);

% ------------------------------------------------------------------
% Tensioni concatenate RMS per sequenza: sqrt(3/2)*ampiezza di fase
% ------------------------------------------------------------------
U1p = sqrt(1.5 * (u1p_cos^2 + u1p_sin^2));
U1n = sqrt(1.5 * (u1n_cos^2 + u1n_sin^2));

% ------------------------------------------------------------------
% Correnti attiva e reattiva per sequenza (RMS di linea)
% La protezione contro la divisione per zero non era presente nello
% schema originale (i blocchi Divide restituivano Inf/NaN a U = 0).
% Portare U_MIN a 0 per replicare esattamente il comportamento vecchio.
% ------------------------------------------------------------------
U_MIN = 1e-9;

if U1p > U_MIN
    i_act_1p   = P1p / (sq3 * U1p);
    i_react_1p = Q1p / (sq3 * U1p);
else
    i_act_1p   = 0;
    i_react_1p = 0;
end

if U1n > U_MIN
    i_act_1n   = P1n / (sq3 * U1n);
    i_react_1n = Q1n / (sq3 * U1n);
else
    i_act_1n   = 0;
    i_react_1n = 0;
end

% ------------------------------------------------------------------
% Bus di uscita (stesso ordine dei campi dell'originale)
% ------------------------------------------------------------------
m.Q1p        = Q1p;
m.Q1n        = Q1n;
m.P1n        = P1n;
m.P1p        = P1p;
m.U1n        = U1n;
m.U1p        = U1p;
m.i_act_1n   = i_act_1n;
m.i_react_1n = i_react_1n;
m.i_act_1p   = i_act_1p;
m.i_react_1p = i_react_1p;

end
