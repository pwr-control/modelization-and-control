% function s_opt = fcs_mpc_vienna(i_abc, vg_abc, vC1, vC2, iref_ab)
% %#codegen
% % =========================================================================
% % FCS-MPC per Vienna rectifier (3 switch, 8 stati) - orizzonte N = 1
% % con compensazione del ritardo di calcolo (predizione a due passi).
% % Versione inference-friendly per il blocco MATLAB Function di Simulink:
% %   - uscita inizializzata a dimensione fissa PRIMA di ogni altro uso
% %   - enumerazione stati via tabella costante (niente bitand)
% %   - nessuna assegnazione che possa cambiare dimensione a runtime
% %
% % INGRESSI (dimensioni da impostare anche nel Ports and Data Manager)
% %   i_abc   [3x1]  correnti di fase misurate
% %   vg_abc  [3x1]  tensioni di rete fase-neutro
% %   vC1     [1x1]  tensione condensatore superiore
% %   vC2     [1x1]  tensione condensatore inferiore
% %   iref_ab [2x1]  riferimento di corrente in alfa-beta
% %
% % USCITA
% %   s_opt   [3x1]  comandi di gate (1 = switch ON -> fase al punto medio)
% % =========================================================================
% 
% % ------------------- Uscita: dimensione fissa subito --------------------
% s_opt = zeros(3,1);
% 
% % ------------------------- Parametri (tuning) ---------------------------
% Ts     = 20e-6;    % [s]  periodo di campionamento (= sample time blocco)
% L      = 1.0e-3;   % [H]  induttanza di boost per fase
% R      = 0.05;     % [Ohm] resistenza serie equivalente
% C      = 1.0e-3;   % [F]  capacita' C1 = C2 del DC-link
% lam_dc = 5.0;      % peso bilanciamento punto neutro
% lam_sw = 0.05;     % peso sulle commutazioni
% i_eps  = 0.5;      % [A] soglia zero-crossing: sotto, uso sgn(iref)
% 
% % Tabella degli 8 stati di switch [sa sb sc]
% STATES = [0 0 0;
%           0 0 1;
%           0 1 0;
%           0 1 1;
%           1 0 0;
%           1 0 1;
%           1 1 0;
%           1 1 1];
% 
% % ------------------------- Stato persistente ----------------------------
% persistent s_old
% if isempty(s_old)
%     s_old = zeros(3,1);
% end
% 
% % ------------------------- Trasformate ----------------------------------
% Tab  = (2/3)*[1, -0.5, -0.5; 0, sqrt(3)/2, -sqrt(3)/2];
% Tabi = [1, 0; -0.5, sqrt(3)/2; -0.5, -sqrt(3)/2];
% 
% i_ab  = Tab * i_abc(:);
% vg_ab = Tab * vg_abc(:);
% dv    = vC1 - vC2;
% 
% iref_abc = Tabi * iref_ab(:);
% 
% % ---------- Passo 1: compensazione ritardo (predizione a k+1) -----------
% [vconv_ab, iM] = vienna_voltage(s_old, i_abc(:), iref_abc, vC1, vC2, i_eps, Tab);
% 
% i1_ab  = i_ab + (Ts/L) * (vg_ab - R*i_ab - vconv_ab);
% dv1    = dv - (Ts/C) * iM;
% i1_abc = Tabi * i1_ab;
% 
% vg1_ab   = vg_ab;
% iref2_ab = iref_ab(:);
% 
% % ---------- Passo 2: enumerazione degli 8 stati e minimizzazione --------
% Jmin = realmax;
% 
% for k = 1:8
%     s = STATES(k,:).';   % [3x1], dimensione costante
% 
%     [v_ab, iMk] = vienna_voltage(s, i1_abc, iref_abc, vC1, vC2, i_eps, Tab);
% 
%     i2_ab = i1_ab + (Ts/L) * (vg1_ab - R*i1_ab - v_ab);
%     dv2   = dv1 - (Ts/C) * iMk;
% 
%     e_ab = iref2_ab - i2_ab;
%     J = e_ab(1)^2 + e_ab(2)^2 ...
%         + lam_dc * dv2^2 ...
%         + lam_sw * sum(abs(s - s_old));
% 
%     if J < Jmin
%         Jmin = J;
%         s_opt(:) = s;    % assegnazione per elementi: dimensione invariata
%     end
% end
% 
% s_old = s_opt;
% 
% end
% 
% % =========================================================================
% function [v_ab, iM] = vienna_voltage(s, i_abc, iref_abc, vC1, vC2, i_eps, Tab)
% % Tensione di convertitore (riferita al punto medio M) e corrente nel
% % punto medio per un dato stato di switch s e segno delle correnti.
% 
% vM = zeros(3,1);
% iM = 0;
% 
% for x = 1:3
%     if abs(i_abc(x)) > i_eps
%         sg = sign(i_abc(x));
%     else
%         sg = sign(iref_abc(x));
%         if sg == 0
%             sg = 1;
%         end
%     end
% 
%     if s(x) > 0.5
%         vM(x) = 0;
%         iM = iM + i_abc(x);
%     else
%         if sg > 0
%             vM(x) = vC1;
%         else
%             vM(x) = -vC2;
%         end
%     end
% end
% 
% v_ab = Tab * vM;
% 
% end
