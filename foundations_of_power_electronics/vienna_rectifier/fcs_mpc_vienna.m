function s_opt = fcs_mpc_vienna(i_abc, vg_abc, vC1, vC2, iref_ab)
%#codegen
% =========================================================================
% FCS-MPC per Vienna rectifier (3 switch, 8 stati) - orizzonte N = 1
% con compensazione del ritardo di calcolo (predizione a due passi).
%
% Da incollare in un blocco "MATLAB Function" in Simulink.
% Sample time del blocco = Ts (ereditato dal solver discreto o esplicito).
%
% INGRESSI
%   i_abc   [3x1]  correnti di fase misurate (convenzione: positive
%                  entranti nel convertitore, lato rete)
%   vg_abc  [3x1]  tensioni di rete fase-neutro (misurate o stimate)
%   vC1     [1x1]  tensione condensatore superiore
%   vC2     [1x1]  tensione condensatore inferiore
%   iref_ab [2x1]  riferimento di corrente in alfa-beta (da anello Vdc
%                  esterno + PLL, gia' ruotato: i_ab* = T(theta)*[id*;iq*])
%
% USCITA
%   s_opt   [3x1]  comandi di gate (1 = switch ON -> fase al punto medio M)
%
% NOTE
%   - Il vincolo di unidirezionalita' del Vienna e' gestito nativamente:
%     la tensione applicata dipende da sgn(i), quindi il modello di
%     predizione "vede" automaticamente i soli vettori ammissibili.
%   - Verificare in simulazione il segno della dinamica del punto neutro
%     (dipende dalla convenzione di i_abc e dall'orientamento di C1/C2).
%   - Discretizzazione Eulero in avanti: valida per Ts << L/R e
%     f_campionamento >> f_rete (tipicamente Ts = 10..50 us).
% =========================================================================

% ------------------------- Parametri (tuning) ---------------------------
Ts     = 20e-6;    % [s]  periodo di campionamento
L      = 1.0e-3;   % [H]  induttanza di boost per fase
R      = 0.05;     % [Ohm] resistenza serie equivalente
C      = 1.0e-3;   % [F]  capacita' C1 = C2 del DC-link
lam_dc = 5.0;      % peso bilanciamento punto neutro [A^2/V^2]
lam_sw = 0.05;     % peso sulle commutazioni [A^2/comm]
i_eps  = 0.5;      % [A] soglia zero-crossing: sotto, uso sgn(iref)

% ------------------------- Stato persistente ----------------------------
persistent s_old
if isempty(s_old)
    s_old = zeros(3,1);
end

% ------------------------- Trasformate ----------------------------------
Tab  = (2/3)*[1, -0.5, -0.5; 0, sqrt(3)/2, -sqrt(3)/2];   % Clarke (ampiezza-invariante)
Tabi = [1, 0; -0.5, sqrt(3)/2; -0.5, -sqrt(3)/2];         % inversa (sistema a 3 fili)

i_ab  = Tab * i_abc;
vg_ab = Tab * vg_abc;
dv    = vC1 - vC2;

% Riferimento in abc solo per il segno vicino allo zero-crossing
iref_abc = Tabi * iref_ab;

% ---------- Passo 1: compensazione ritardo (predizione a k+1) -----------
% Applico lo stato ottimo calcolato al ciclo precedente (s_old) e
% predico lo stato a k+1, da cui valutero' i candidati per k+2.
[vconv_ab, iM] = vienna_voltage(s_old, i_abc, iref_abc, vC1, vC2, i_eps, Tab);

i1_ab  = i_ab + (Ts/L) * (vg_ab - R*i_ab - vconv_ab);
dv1    = dv - (Ts/C) * iM;
i1_abc = Tabi * i1_ab;

% Rete a k+1: tengo vg costante (a 20 us l'errore e' trascurabile;
% in alternativa estrapolare/ruotare con omega dal PLL).
vg1_ab = vg_ab;

% Riferimento a k+2: qui uso iref_ab costante. Per ridurre il ritardo di
% inseguimento si puo' ruotare di 2*omega*Ts:
%   th = 2*w*Ts; Rot = [cos(th) -sin(th); sin(th) cos(th)];
%   iref2_ab = Rot * iref_ab;
iref2_ab = iref_ab;

% ---------- Passo 2: enumerazione degli 8 stati e minimizzazione --------
Jmin  = inf;
s_opt = s_old;

for k = 0:7
    s = [double(bitand(k,4) ~= 0); ...
         double(bitand(k,2) ~= 0); ...
         double(bitand(k,1) ~= 0)];

    [v_ab, iMk] = vienna_voltage(s, i1_abc, iref_abc, vC1, vC2, i_eps, Tab);

    % Predizione corrente e sbilanciamento a k+2
    i2_ab = i1_ab + (Ts/L) * (vg1_ab - R*i1_ab - v_ab);
    dv2   = dv1 - (Ts/C) * iMk;

    % Funzione di costo
    e_ab = iref2_ab - i2_ab;
    J = e_ab(1)^2 + e_ab(2)^2 ...
        + lam_dc * dv2^2 ...
        + lam_sw * sum(abs(s - s_old));

    if J < Jmin
        Jmin  = J;
        s_opt = s;
    end
end

s_old = s_opt;

end

% =========================================================================
function [v_ab, iM] = vienna_voltage(s, i_abc, iref_abc, vC1, vC2, i_eps, Tab)
% Tensione di convertitore (riferita al punto medio M) e corrente nel
% punto medio per un dato stato di switch s e segno delle correnti.
%
%   s_x = 1 -> fase collegata a M      -> v_xM = 0,  i_x scorre in M
%   s_x = 0 -> conduce il diodo        -> v_xM = +vC1 se i_x > 0
%                                          v_xM = -vC2 se i_x < 0
%
% Vicino allo zero-crossing (|i| < i_eps) il segno della misura e'
% inaffidabile: uso il segno del riferimento (noto e pulito dal PLL).

vM = zeros(3,1);
iM = 0;

for x = 1:3
    if abs(i_abc(x)) > i_eps
        sg = sign(i_abc(x));
    else
        sg = sign(iref_abc(x));
        if sg == 0
            sg = 1;
        end
    end

    if s(x) > 0.5
        vM(x) = 0;
        iM = iM + i_abc(x);
    else
        if sg > 0
            vM(x) = vC1;
        else
            vM(x) = -vC2;
        end
    end
end

% La Clarke ampiezza-invariante elimina la componente omopolare, quindi
% non serve sottrarre esplicitamente il modo comune.
v_ab = Tab * vM;

end
