# FEM 1D + LQR — Co-simulazione MATLAB/Simulink
## Barra incastrata-incastrata, 50 elementi, disturbo al 2° modo, controllo LQR

---

## Struttura file

```
fem1d_init.m            ← (1) Assemblaggio FEM, analisi modale, progetto LQR
fem1d_matlabfcn.m       ← Codice da incollare nel MATLAB Function block
fem1d_build_model.m     ← (2) Costruisce il modello Simulink .slx
fem1d_postprocess.m     ← (3) Plot e analisi post-simulazione
fem1d_workspace.mat     ← Generato automaticamente da fem1d_init.m
```

---

## Workflow operativo

```
1. >> fem1d_init          % assembla matrici, progetta LQR, salva .mat
2. >> fem1d_build_model   % crea fem1d_simulink.slx
3. Aprire il blocco "FEM_Core" e incollare il codice da fem1d_matlabfcn.m
4. Simulare (Play)
5. >> fem1d_postprocess   % analisi risultati
```

---

## Architettura matematica

### Modello FEM — barra vibrante assiale

Elemento lineare a 2 nodi (vibrazione longitudinale):

```
         u_1           u_2
          ●─────────────●
          |←    le     →|
```

Matrici di elemento:

```
k_e = (EA/le) * [ 1 -1 ]      m_e = (ρAl/6) * [ 2  1 ]
                [-1  1 ]                        [ 1  2 ]
```

Equazione del moto globale (dopo applicazione BC incastro-incastro):

```
M·ü + C·u̇ + K·u = B_dist·f_dist(t) + B_ctrl·f_ctrl(t)
```

- **M, K** ∈ ℝ^(49×49)   (n_dof = n_el - 1 = 49 nodi interni)
- **C** = Rayleigh: C = α·M + β·K  con ξ = 0.5% (modi 1,2)

### Analisi modale

Frequenze naturali barra incastrata-incastrata (teoriche):

```
f_n = n/(2L) · √(E/ρ)    n = 1, 2, 3, ...
```

Per acciaio tipico (E=210 GPa, ρ=7800 kg/m³, L=1m, A=1cm²):
- f₁ ≈ 2550 Hz
- f₂ ≈ 5100 Hz
- f₃ ≈ 7650 Hz

> **Nota**: con 50 elementi la discretizzazione è molto accurata per i primi modi.

### Posizionamento forze

| Forza     | Nodo     | Posizione | Motivazione                          |
|-----------|----------|-----------|--------------------------------------|
| Disturbo  | 13 (L/4) | 0.25 m    | Anti-nodo del modo 2 (max |φ₂|)      |
| Controllo | 25 (L/2) | 0.50 m    | Anti-nodo del modo 1, nodo del modo 2 |

> **Attenzione**: L/2 è un **nodo modale del modo 2** (φ₂ = 0).  
> Il controllore agisce prevalentemente sul modo 1.  
> Per controllare anche il modo 2, spostare il controllo a L/4 o L×3/4.
> Lo script segnala questo con un warning se |φ₂(ctrl)| < 1e-6.

### Spazio di stato

```
ẋ = A·x + B·u_ctrl + Bd·d
y = C·x

x = [u; u̇] ∈ ℝ^98,  u_ctrl ∈ ℝ,  d ∈ ℝ

A = [ 0    I  ]    B  = [ 0      ]    Bd = [ 0        ]
    [-M⁻¹K  -M⁻¹C]        [ M⁻¹B_c]         [ M⁻¹B_d  ]
```

### LQR

Minimizza il costo:

```
J = ∫₀^∞ (x'Qx + u'Ru) dt

Q = diag(q_u·I_{49}, q_v·I_{49})    q_u = 1e6,  q_v = 1e2
R = 1.0  [N⁻²]
```

Legge di controllo: **u_ctrl = −K_lqr · x**

---

## Schema Simulink

```
                    ┌─────────────────────────────────────────┐
                    │           FEM_Core (MATLAB Fcn)         │
 Sine Wave ─────────┤ f_dist                     u_ddot ├────► Integrator ─► u̇
 (ω = ω₂)          │                                         │               │
                    │ f_ctrl                      y_obs ├────► Scope/WS      │
         ┌──────────┤                                         │               ▼
         │          │ u ◄─────────────────────────────────────────────── Integrator ─► u
         │          │ u̇ ◄──────────────────────────────────────────────────────┘
         │          └─────────────────────────────────────────┘
         │
    Ctrl_Switch (t > t_ctrl_on)
         │
    [-K_lqr] ◄── [u; u̇] ← Mux(u, u̇)
```

---

## Parametri principali

| Parametro       | Valore        | Note                          |
|----------------|---------------|-------------------------------|
| L              | 1.0 m         |                               |
| E              | 210 GPa       | Acciaio                       |
| ρ              | 7800 kg/m³    |                               |
| A              | 1 cm²         | Sezione                       |
| n_el           | 50            | Elementi                      |
| ξ              | 0.5%          | Smorzamento modale (Rayleigh) |
| F_amp          | 1.0 N         | Ampiezza disturbo             |
| ω_exc          | ω₂ (modo 2)   | Risonanza forzata             |
| t_ctrl_on      | 1.0 s         | Attivazione LQR               |
| T_sim          | 5.0 s         | Durata simulazione            |
| dt             | 10 µs         | Passo fisso (ode4)            |

---

## Estensioni possibili

1. **Observer (Luenberger/Kalman)**: stimare lo stato da soli pochi sensori  
   → sistema ridotto a 1-2 misure di spostamento

2. **Riduzione modale**: proiettare su spazio modale ridotto (es. modi 1-5)  
   → `x_modal = Phi_r' * M * x_phys`, LQR su sistema 10×10

3. **Barra di Euler-Bernoulli** (flessione):  
   → elemento Hermite a 4 DOF (v, θ), stessa struttura

4. **2D FEM**: estendere a piastra rettangolare con elementi Q4/Q8

5. **Hardware-in-the-loop**: sostituire Sine Wave con segnale da dSPACE  
   → passo fisso obbligatorio, già configurato con ode4
