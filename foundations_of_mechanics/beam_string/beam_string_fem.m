%% =======================================================================
%  beam_string_fem.m
%  FEM 1D di una "barra-stringa": trave di Eulero-Bernoulli in tensione
%
%  PDE:  rhoA*w_tt + c*w_t + EI*w_xxxx - T*w_xx = f(x,t)
%
%  - Elementi di Hermite (2 nodi, 4 gdl: w, w')
%  - K = K_flessionale (EI) + K_geometrica (T)
%  - Condizioni al contorno: appoggiato-appoggiato (pinned-pinned)
%  - Analisi modale + confronto con soluzione analitica
%  - Risposta armonica forzata (FRF) attorno a f0 = 2 kHz
%  - Risposta nel tempo (Newmark) a f0: crescita fino al regime
%
%  Obiettivo: mostrare che la barra-stringa entra in risonanza per
%  f0 = 2 kHz +/- uno spread legato allo smorzamento (banda a -3 dB).
%
%  Davide -- verificato numericamente (errore FEM/analitico ~1e-6 %)
%% =======================================================================
clear; close all; clc;

%% ---------------------- 1) Parametri fisici ---------------------------
L    = 0.8;            % [m]   lunghezza campata
d    = 2e-3;           % [m]   diametro filo (acciaio armonico)
E    = 210e9;          % [Pa]  modulo di Young
rho  = 7850;           % [kg/m^3]
A    = pi*d^2/4;       % [m^2] sezione
I    = pi*d^4/64;      % [m^4] momento d'inerzia
EI   = E*I;            % [N m^2] rigidezza flessionale
rhoA = rho*A;          % [kg/m] massa per unita' di lunghezza

f0     = 2000;         % [Hz]  frequenza dell'eccitazione armonica
w0     = 2*pi*f0;
zeta   = 0.005;        % [-]   smorzamento modale target (0.5 %)
spread = 0.10;         % [-]   semi-larghezza relativa dello sweep FRF

F0  = 1.0;             % [N]   ampiezza forza puntuale
xF  = 0.30*L;          % [m]   punto di applicazione (evita nodi del modo)
xo  = 0.37*L;          % [m]   punto di osservazione

tune_to_resonance = true;   % se true: T viene calcolata per mettere
T0 = 3000;                  % il modo piu' vicino ESATTAMENTE a f0
                            % (altrimenti si usa T = T0)

%% ------------- 2) Tuning della tensione (opzionale) -------------------
% Analitico appoggiato-appoggiato:
%   w_n^2 = beta^2*(T/rhoA) + beta^4*(EI/rhoA),  beta = n*pi/L
fn_ana = @(n,T) sqrt((n*pi/L).^2*T/rhoA + (n*pi/L).^4*EI/rhoA)/(2*pi);

if tune_to_resonance
    nv    = (1:60)';
    [~,k] = min(abs(fn_ana(nv,T0) - f0));
    nstar = nv(k);
    beta  = nstar*pi/L;
    T     = (rhoA*w0^2 - EI*beta^4)/beta^2;   % inversione della dispersione
    assert(T > 0, 'T non fisica: cambiare L, d o il modo target');
else
    T = T0;
    [~,nstar] = min(abs(fn_ana((1:60)',T) - f0));
end
fprintf('Tensione T = %.1f N  (sigma = %.0f MPa), modo target n* = %d\n', ...
        T, T/A/1e6, nstar);

%% ------------------ 3) Assemblaggio FEM (Hermite) ---------------------
Ne   = 200;                 % n. elementi (>= 20 elementi/lunghezza d'onda)
h    = L/Ne;
Nn   = Ne + 1;
Ndof = 2*Nn;                % gdl: [w1 th1 w2 th2 ...]

% Matrici elementari (trave E-B + stiffness geometrica + massa consistente)
Kb = EI/h^3 * [ 12    6*h   -12    6*h ;
                 6*h  4*h^2  -6*h  2*h^2;
               -12   -6*h    12   -6*h ;
                 6*h  2*h^2  -6*h  4*h^2];

Kg = T/(30*h) * [ 36    3*h   -36    3*h ;
                   3*h  4*h^2  -3*h   -h^2;
                 -36   -3*h    36   -3*h ;
                   3*h   -h^2  -3*h  4*h^2];

Me = rhoA*h/420 * [156    22*h    54   -13*h ;
                    22*h   4*h^2  13*h  -3*h^2;
                    54    13*h   156   -22*h ;
                   -13*h  -3*h^2 -22*h   4*h^2];

K = sparse(Ndof,Ndof);  M = sparse(Ndof,Ndof);
for e = 1:Ne
    idx = 2*(e-1) + (1:4);
    K(idx,idx) = K(idx,idx) + Kb + Kg;
    M(idx,idx) = M(idx,idx) + Me;
end

% BC appoggiato-appoggiato: w(0)=w(L)=0 (le rotazioni restano libere)
fix  = [1, Ndof-1];
keep = setdiff(1:Ndof, fix);
Kr = K(keep,keep);  Mr = M(keep,keep);

%% ---------------------- 4) Analisi modale -----------------------------
nModi = 15;
[V,D]  = eigs(Kr, Mr, nModi, 'smallestabs');
[wn2,ord] = sort(diag(D));  V = V(:,ord);
fn_fem = sqrt(wn2)/(2*pi);

fprintf('\n  n | f_FEM [Hz] | f_analitica [Hz] | err %%\n');
for n = 1:min(nModi,12)
    fa = fn_ana(n,T);
    fprintf(' %2d | %9.2f | %14.2f | %.2e\n', n, fn_fem(n), fa, ...
            abs(fn_fem(n)-fa)/fa*100);
end
fprintf('--> modo n* = %d a f = %.2f Hz (target %.0f Hz)\n\n', ...
        nstar, fn_fem(nstar), f0);

%% -------------- 5) Smorzamento di Rayleigh C = a*M + b*K --------------
% zeta imposto a w0 e 2*w0 (banda di interesse)
w1 = w0; w2 = 2*w0;
aR = 2*zeta*w1*w2/(w1+w2);
bR = 2*zeta/(w1+w2);
Cr = aR*Mr + bR*Kr;

%% --------- 6) Vettore forza (carico puntuale via funzioni forma) ------
loc = @(x) deal( min(floor(x/h),Ne-1), (x - min(floor(x/h),Ne-1)*h)/h );
shp = @(xi) [1-3*xi^2+2*xi^3, h*(xi-2*xi^2+xi^3), ...
             3*xi^2-2*xi^3,   h*(-xi^2+xi^3)];

[eF,xiF] = loc(xF);  NF = shp(xiF);  idxF = 2*eF + (1:4);
F = zeros(Ndof,1);  F(idxF) = F0*NF.';
Fr = F(keep);

[eo,xio] = loc(xo);  No = shp(xio);  idxo = 2*eo + (1:4);
g2r = zeros(Ndof,1);  g2r(keep) = 1:numel(keep);   % mappa gdl->ridotto
pick = @(W) sum( No(:) .* [W(g2r(idxo(1))); W(g2r(idxo(2))); ...
                           W(g2r(idxo(3))); W(g2r(idxo(4)))] );

%% ------------- 7) FRF: sweep armonico attorno a f0 --------------------
fv  = linspace((1-spread)*f0, (1+spread)*f0, 401);
Wamp = zeros(size(fv));
for k = 1:numel(fv)
    w = 2*pi*fv(k);
    Zw = Kr + 1i*w*Cr - w^2*Mr;      % impedenza dinamica
    W  = Zw\Fr;
    Wamp(k) = abs(pick(W));
end

[Wpk,ipk] = max(Wamp);
% banda a meta' potenza (-3 dB) --> "spread" di risonanza
i3 = find(Wamp >= Wpk/sqrt(2));
df3 = fv(i3(end)) - fv(i3(1));
fprintf('Picco FRF a %.1f Hz, |W| = %.3f mm/N\n', fv(ipk), Wpk*1e3);
fprintf('Banda -3 dB = %.1f Hz  (2*zeta*f0 teorico = %.1f Hz)\n', ...
        df3, 2*zeta*f0);

figure('Name','FRF');
semilogy(fv, Wamp*1e3, 'LineWidth', 1.4); grid on; hold on;
xline(f0,'--r','f_0 = 2 kHz'); xline(fn_fem(nstar),':k');
yline(Wpk/sqrt(2)*1e3,':','-3 dB');
xlabel('f [Hz]'); ylabel('|W(x_o,f)| [mm/N]');
title(sprintf('FRF barra-stringa: risonanza modo n=%d', nstar));

%% -------- 8) Deformata operativa (ODS) alla risonanza -----------------
w  = w0;
W  = (Kr + 1i*w*Cr - w^2*Mr)\Fr;
Wfull = zeros(Ndof,1);  Wfull(keep) = W;
xn = linspace(0,L,Nn);
figure('Name','ODS');
plot(xn, abs(Wfull(1:2:end))*1e3, 'LineWidth', 1.4); grid on;
xlabel('x [m]'); ylabel('|w(x)| [mm]');
title('Deformata operativa a 2 kHz (ampiezza)');

%% -------- 9) Risposta nel tempo a f0 (Newmark, beta=1/4) --------------
Tend = 60/(zeta*w0);  Tend = min(Tend, 0.25);   % ~ tempo di assestamento
dt   = 1/(40*f0);
Nt   = round(Tend/dt);
gam = 1/2; bet = 1/4;

u = zeros(size(Fr)); v = u;
a = Mr\(Fr*sin(0) - Cr*v - Kr*u);
Keff = Kr + gam/(bet*dt)*Cr + 1/(bet*dt^2)*Mr;
dKeff = decomposition(Keff);
wo_t = zeros(Nt,1); tvec = (1:Nt)*dt;
for k = 1:Nt
    t  = tvec(k);
    Fk = Fr*sin(w0*t);
    rhs = Fk + Mr*(u/(bet*dt^2) + v/(bet*dt) + (1/(2*bet)-1)*a) ...
             + Cr*(gam/(bet*dt)*u + (gam/bet-1)*v + dt*(gam/(2*bet)-1)*a);
    un = dKeff\rhs;
    an = (un - u)/(bet*dt^2) - v/(bet*dt) - (1/(2*bet)-1)*a;
    vn = v + dt*((1-gam)*a + gam*an);
    u = un; v = vn; a = an;
    wo_t(k) = pick(u);
end

figure('Name','Transitorio');
plot(tvec*1e3, wo_t*1e3, 'LineWidth', 0.8); grid on;
xlabel('t [ms]'); ylabel('w(x_o,t) [mm]');
title('Risposta a f_0 = 2 kHz: crescita risonante fino al regime (lim. da \zeta)');

%% ---------------------- 10) Nota di verifica --------------------------
% - fn_fem coincide con fn_ana a ~1e-6 % con Ne = 200
% - il picco FRF cade su f0 se tune_to_resonance = true
% - banda -3 dB ~ 2*zeta*f0: e' lo "spread" entro cui la struttura
%   risponde in risonanza (qui ~20 Hz con zeta = 0.5 %)
