# Revisione del modello `voll_bridge_tobias.slx`

Rev. 00 — 2026-10-31 — revisione richiesta da Davide, modello di Tobias. Note di lavoro: da aggiornare man mano che il modello evolve.

File esaminato: `foundations_of_power_electronics/simulink_voll_bridge/voll_bridge_tobias.slx`
(branch `dev`, commit 8c38a09; identico al file sul PC di Davide, md5 383ad144…). Salvato con
R2026b, creato il 31/10/2026 alle 10:48 UTC dall'utente "trainer", risalvato da d.bagnara alle 10:59.
Nella cartella c'è solo il file `.slx`: nessun `init_model.m`, nessun README, nessuno script.

La revisione è statica (lettura del file XML, nessuna simulazione): le equazioni sono state ricavate
dai blocchi e verificate a mano; i numeri di dimensionamento sono stime indicative.

## In sintesi

1. **Il file contiene solo lo stadio di potenza.** È un modello a flusso di segnale (niente
   Simscape) con integratori di Eulero espliciti a passo `TA`: ponte primario ideale, trasformatore
   a T, ponte secondario ideale, filtro LC d'ingresso, condensatore d'uscita e un profilo di carico.
   **Non c'è né modulatore né regolazione**: gli ingressi PWM dei due ponti (`PWM_prim`, `PWM_sek`)
   sono scollegati. Il "controllo" quindi non si può valutare.
2. **Non gira da solo.** Servono 12 variabili di workspace non definite da nessuna parte
   (`TA, R1, R2, L1, L2, Lm, N, Delta, R_in_filter, L_in_filter, C_in_filter` e il tipo dato
   `S_F32`), più la libreria esterna `SDE_Blockset` (blocco Multiport Switch del selettore di carico).
   Non è nel repo né nella `library`.
3. **Equazioni verificate e corrette**: trasformatore a T (con `Delta = L1·L2 + L1·Lm + L2·Lm`),
   filtro d'ingresso, condensatore d'uscita, ponte secondario.
4. **Errori trovati**: (a) stato "tutti gli switch spenti" del ponte primario con soglie sbagliate
   (`> 0.1` e `< 0.1`, non simmetriche) e corrente DC posta a zero, quindi non conserva l'energia;
   (b) l'uscita `UDC_p` del filtro d'ingresso non è collegata: il ponte vede i 450 V ideali e il
   filtro è solo decorativo; (c) condizione iniziale della corrente di filtro `i_L(0) = 450 A`;
   (d) configurazione salvata con solver a passo variabile ("auto") mentre il modello è tutto a
   Unit Delay con passo ereditato: incoerente con gli integratori a `TA`.
5. **Dimensionamento**: si vedono solo 450 V → 400 V, carico 1.9 ↔ 14.7 kW (rampa in 5 ms),
   `Cout = 33.6 µF`, `Lm = 0.5 mH`, `n = 0.995`. Frequenza di switching, induttanza serie e `TA`
   non sono nel file. Con 33.6 µF il condensatore d'uscita tiene 0.18 ms di energia a pieno
   carico: va bene solo con fsw alta (≥ 50 kHz) e un anello di tensione veloce (qualche kHz).
6. **Convenzioni del repo**: manca tutto il contorno (init con `param`, README, nomi inglesi,
   niente numeri a mano nei blocchi). Vedi §6.

## 1. Struttura del modello

Percorso del segnale (livello radice):

```
V1 = 450 V ──► Vollbrücke3 (ponte primario) ──Uab──► Trafo (T-ESB) ──is──► Vollbrücke2 (ponte secondario) ──isDC──► Ausgang ──► UDC_s, iout, DC_pwr
                  ▲ ipDC                          ◄──ip──┘      ◄──Ucd──┘                         ▲ UDC_s          ▲ Rout (Subsystem1)
                  └──► Subsystem3 (filtro LC d'ingresso: i_L, UDC_p, Pwr_in)   [UDC_p NON torna al ponte]
```

| Sottosistema | Ruolo | Stati (Eulero esplicito, passo `TA`) | Parametri |
|---|---|---|---|
| `Vollbrücke3` | ponte primario ideale: `Up = +UDC` se `PWM_A`, `−UDC` se `PWM_B`, altrimenti logica "diodi" sul segno di `iPri`; `ipDC = ±iPri` o 0 | nessuno | soglie 0.1 A (vedi §3) |
| `Trafo (T-ESB)` | trasformatore a T riferito al primario, rapporto `N = Np/Ns` | `ip`, `is'` | `R1 L1 R2 L2 Lm N Delta` |
| `Vollbrücke2` | ponte secondario ideale: `Us = ±uDCout` o 0, `iRect = ±iS` o 0 | nessuno | — |
| `Ausgang` | condensatore d'uscita e carico resistivo: `ic = isDC − UDC_s/Rout` | `UDC_s(0) = 400 V` | `C = 33.6 µF` scritto nel Gain |
| `Subsystem3` | filtro LC d'ingresso dalla batteria `Ubat = V1`: `i_L`, `UDC_p`, `Pwr_in = Ubat·i_L` | `i_L(0) = 450`, `UDC_p(0) = 450` | `R_in_filter L_in_filter C_in_filter` |
| `Subsystem1` | selettore del carico (Multiport Switch `SDE_Blockset`, comando = 6 → ultimo ingresso) | — | 6, 20, 80, 10.9433, 1e6 Ω, rampe |
| `Trafo Einfach` | trasformatore semplificato (commentato): L serie + trafo ideale; `Lm` e `RFe` calcolati ma scollegati (Terminator) | — | `n = 0.995, Lm = 0.5 mH, RFe = 1 MΩ, L` |
| `Discrete FIR Filter3` | media mobile su `N_points` (commentata) | — | `N_points` |

Il tipo dato `S_F32` sulla costante 450 V e nella conversione dentro `Integrator2` suggerisce che il
modello sia nato per girare in float32 su un target real-time (HIL/emulatore). È una scelta
legittima, ma diversa dalla filosofia del repo (Simscape + init `param`), e va dichiarata.

Profilo di carico effettivo (Multiport Switch con comando costante 6 → somma delle due rampe):

| t | Rout | P a 400 V | I |
|---|---|---|---|
| 0 ÷ 5 ms | 84.5 Ω | 1.9 kW | 4.7 A |
| 5 ÷ 9.9 ms | rampa 84.5 → 10.9 Ω | 1.9 → 14.7 kW | 4.7 → 36.7 A |
| 9.9 ÷ 20 ms | 10.9 Ω | 14.7 kW | 36.7 A |
| 20 ÷ 25 ms | rampa 10.9 → 85.4 Ω | 14.7 → 1.9 kW | 36.7 → 4.7 A |

Gli altri rami del selettore (6 Ω = 26.7 kW, 20 Ω = 8 kW, 10.94 ↔ 80 Ω con step a 5 ms, 1 MΩ)
non sono selezionati.

## 2. Equazioni verificate (corrette)

**Trasformatore a T** (grandezze riferite al primario, `Ucd' = N·Ucd`, `is` in uscita = `N·is'`):

```
Delta · dip/dt  = (L2+Lm)·Uab − Lm·Ucd' − R1·(L2+Lm)·ip − R2·Lm·is'
Delta · dis'/dt =  Lm·Uab − (L1+Lm)·Ucd' − R1·Lm·ip − R2·(L1+Lm)·is'
```

Sono esattamente le equazioni del circuito a T con `im = ip − is'`, a patto che nel workspace sia
`Delta = L1·L2 + L1·Lm + L2·Lm` (positivo) e `N = Np/Ns`. Controllato anche il limite `Lm → ∞`
(circuito serie `R1+R2`, `L1+L2`) e il bilancio sul ramo magnetizzante. Convenzione: `is'` esce dal
secondario verso il ponte, `Ucd` è la tensione imposta dal ponte secondario.

**Filtro d'ingresso**: `L·di_L/dt = Ubat − UDC_p − R·i_L`, `C·dUDC_p/dt = i_L − ipDC`. Corretto.

**Stadio d'uscita**: `C·dUDC_s/dt = isDC − UDC_s/Rout`, `DC_pwr = UDC_s·iout`. Corretto.

**Ponte secondario**: `Us·iS = uDCout·iRect` in tutti gli stati: bilancio energetico rispettato.

Nessun loop algebrico: ogni retroazione passa da un Unit Delay.

## 3. Errori e punti critici nel modello

### 3.1 Ponte primario, stato "tutti spenti" (`PWM_A = PWM_B = 0`)

Tobias ha voluto modellare la conduzione dei diodi quando i quattro switch sono spenti:
`Up = −|UDC|` se `iPri > 0.1`, `Up = +|UDC|` se `iPri < 0.1`, altrimenti 0.

- **Soglie sbagliate**: i due Compare To Constant confrontano entrambi con `0.1` (`>` e `<`).
  Lo stato "corrente nulla → tensione nulla" si raggiunge solo per `iPri = 0.1` esatto. Per
  `0 ≤ iPri < 0.1` il modello applica `+|UDC|` e spinge la corrente in su; appena supera 0.1 applica
  `−|UDC|`: la corrente oscilla attorno a 0.1 A a frequenza `1/TA` con ±450 V sul trasformatore.
  Serviva almeno `< −0.1` sul secondo comparatore e, meglio, uno stato che tenga la corrente a
  zero (nel modello a flusso di segnale: derivata nulla, cioè `Up = Ucd'`, non `Up = 0`).
- **Energia non conservata**: in questo stato `ipDC = 0`, ma con `Up = ∓|UDC|` e corrente che
  circola la potenza `Up·iPri < 0` torna al DC link: il valore corretto è `ipDC = −|iPri|`. Così
  com'è l'energia dell'induttanza "sparisce". Finché lo stato dura solo i tempi morti l'errore è
  piccolo, ma il modello non lo rende evidente.
- **Asimmetria col secondario**: `Vollbrücke2` tratta `[0 0]` come stato zero (cortocircuito:
  `Us = 0`, `iRect = 0`), il primario come "tutti spenti". Se la stessa coppia di segnali PWM ha
  significati diversi nei due ponti, un modulatore a tre livelli (phase shift con stato zero) sul
  primario darebbe ∓UDC invece di 0. Va deciso un significato unico e, se serve il raddrizzatore
  a diodi o i tempi morti sul secondario, aggiungere la modalità diodo anche lì.
- `PWM_A = PWM_B = 1` (stato non valido, cortocircuito di gamba) non è intercettato: vince `PWM_A`.

### 3.2 Cablaggio

- **`PWM_prim` e `PWM_sek` scollegati** (nessuna linea sugli ingressi 2 dei due ponti). Simulink
  li mette a zero: il primario resta nello stato "tutti spenti" di §3.1, il secondario in
  cortocircuito, il condensatore d'uscita si scarica sul carico con τ = 2.8 ms. Una simulazione
  del file così com'è non dice nulla.
- **`UDC_p` di `Subsystem3` non va da nessuna parte.** Il ponte primario riceve `V1 = 450 V` dalla
  costante, non la tensione a valle del filtro. Il filtro LC calcola `i_L` e `UDC_p` ma non
  influisce sul convertitore: `Pwr_in` è "misurata" su un filtro che la sorgente non vede.
  Se l'intenzione è una sorgente ideale, il filtro va tolto; se no, `UDC_p` va collegato all'ingresso
  del ponte.
- **`i_L(0) = 450`**: la condizione iniziale dell'integratore della corrente di filtro è 450 A
  (copia-incolla da `UDC_p(0) = 450`). Dovrebbe essere ≈ 0 o la corrente di carico iniziale.
- **Tag Goto duplicati**: `ip`, `is` e `UDC_p` esistono come Goto globali alla radice e come Goto
  locali dentro `Trafo (T-ESB)` / `Subsystem3`. I From interni risolvono sul locale, ma è fragile
  (Simulink può segnalare il conflitto) e fuorviante: il globale `UDC_p` porta in realtà `Ubat`
  (450 V costanti), quello locale la tensione filtrata. Dentro il trafo il From `is` deve leggere
  `is'` (riferita al primario), non la `is = N·is'` del Goto globale: con `N ≈ 1` l'errore non si
  vedrebbe, ma è un'ambiguità da togliere.

### 3.3 Solver, passo di integrazione, tipi dato

- Il file è salvato con il solver di default (**`VariableStepAuto`**) e tutti gli Unit Delay hanno
  sample time ereditato (`-1`); lo Step del selettore di carico è continuo (`SampleTime = 0`). Gli
  integratori sono Eulero espliciti con guadagno `TA`: funzionano solo con un **solver a passo
  fisso discreto con `FixedStep = TA`** (o con `TA` esplicito nei blocchi). Con la configurazione
  salvata o Simulink si ferma sugli Unit Delay senza frequenza, oppure il passo non coincide con
  `TA` e l'integrazione è sbagliata. Se Tobias imposta il solver dal suo script, va salvato nel modello.
- **Eulero esplicito**: stabile sull'RL del trafo e sull'RC d'uscita per qualsiasi `TA` ragionevole
  (`TA ≪ L/R`, `TA ≪ R·C = 0.37 ms`), ma sull'LC d'ingresso poco smorzato è instabile se
  `TA·ω0 > 2ζ` (con `ω0 = 1/√(L_in·C_in)`, `ζ = R_in/2·√(C_in/L_in)`). Da verificare con i valori.
  La risoluzione del PWM è pari a `TA`: con fsw di decine di kHz servono `TA` di decine di ns.
- **`S_F32`** non è definito nel modello (nessun model workspace, nessun data dictionary): va
  definito nel workspace (es. `Simulink.AliasType` di `single`). Se è `single`, la precisione basta
  (risoluzione 3e-5 V a 450 V contro incrementi per passo di ordine 1e-2 V), ma la conversione con
  arrotondamento `Floor` è presente solo in `Integrator2` del filtro e non altrove: incoerente.
  Con tipi misti single/double alcuni Sum possono fermarsi in compilazione (opzione "stesso tipo
  dato sugli ingressi"); senza MATLAB non l'ho potuto verificare.

## 4. Dimensionamento: quello che si vede

| Grandezza | Valore nel modello | Commento |
|---|---|---|
| Tensione d'ingresso (`Ubat`) | 450 V costanti | con `n = 0.995` il rapporto di conversione è `d = n·Uout/Uin = 0.88`: per un DAB vuol dire corrente reattiva e perdita di ZVS a basso carico su un ponte; la tensione batteria reale varia con il SOC e il modello non lo copre |
| Tensione d'uscita | 400 V (condizione iniziale) | nessun riferimento esplicito |
| Potenza | 1.9 ↔ 14.7 kW (36.7 A); 26.7 kW sul ramo 6 Ω | la potenza nominale non è dichiarata |
| `Cout` | 33.6 µF | energia 2.7 J = 0.18 ms a 14.6 kW; τ = R·C = 0.37 ms a pieno carico |
| `Lm` | 0.5 mH (dal `Trafo Einfach`) | corrente magnetizzante pk-pk `Uin·T/(2·Lm)`: 22.5 A a 20 kHz (60 % della corrente di carico), 9 A a 50 kHz, 4.5 A a 100 kHz |
| `L1, L2, R1, R2, N` | non nel file | per un DAB a 14.6 kW con φ = 45° serve `L ≈ 57 µH` a 20 kHz, 23 µH a 50 kHz, 11.5 µH a 100 kHz (ordine di grandezza) |
| Filtro d'ingresso | non nel file | e comunque scollegato dal ponte |
| fsw, `TA`, tempi morti | non nel file | — |

Stime sul condensatore d'uscita (correnti rettificate pulsanti tipo DAB, carica per semiperiodo
≈ `Io·T/4`):

| fsw | ripple stimato su `UDC_s` |
|---|---|
| 20 kHz | ≈ 14 V (3.4 %) |
| 50 kHz | ≈ 5 V (1.4 %) |
| 100 kHz | ≈ 3 V (0.7 %) |

Gradino di carico 4.7 → 36.7 A: a tensione costante il condensatore da solo cala di
0.95 V/µs. Caduta stimata con un anello di tensione di banda `f_c`: 150 V a 1 kHz, 75 V a 2 kHz,
30 V a 5 kHz per un gradino secco; per la rampa in 5 ms del modello 4.8 V a 1 kHz, 1.2 V a 2 kHz.
Conclusione: 33.6 µF è coerente solo con fsw ≥ 50 kHz **e** un anello di tensione da qualche kHz
(un DAB lo permette, il phase shift agisce entro un periodo); altrimenti il condensatore è
sottodimensionato di un ordine di grandezza (confronto: il `dcdc_zvs_full_bridge` del repo ha
3.6 mF a 175 kW).

## 5. Controllo

Nel file non c'è nulla: né modulatore (phase shift / duty, tempi morti), né anello di tensione,
né limite di corrente, né campionamento. Prima di valutare il controllo serve sapere la topologia
voluta (DAB con entrambi i ponti attivi? full bridge con raddrizzatore sincrono? PSFB?), la fsw e
il tempo di calcolo `TA`. Per un DAB su questo stadio di potenza mi aspetterei: modulatore
single phase shift con tempi morti (e lo stato zero coerente fra i due ponti, §3.1), misura di
`UDC_s` campionata a fsw o 2·fsw, PI di tensione con feed-forward della potenza di carico, limite
di corrente sul trafo; la banda richiesta dal `Cout` scelto è in §4.

## 6. Rispetto alle convenzioni del repo

- Nessun `init_model.m`: il modello legge 12 nomi dal workspace senza che nessuno li definisca.
  Nel repo i modelli hanno `init_model.m` con la struct `param` (un gruppo per sottosistema,
  funzioni in `private/`, controllo dei riferimenti con `tools/list_model_workspace_names`).
- Numeri scritti dentro i blocchi: 450 V, 33.6 µF, 400 V, i resistori di carico (6, 20, 80,
  10.9433, 1e6 Ω), le rampe (84.5, 7450, 0.005 s, 0.02 s), le soglie 0.1 A. Vanno in `param`.
- Dipendenza da `SDE_Blockset` (Multiport Switch): o si sostituisce col blocco Simulink standard
  o la libreria entra in `library`.
- Nomi: cartella e file (`simulink_voll_bridge/voll_bridge_tobias.slx`) e sottosistemi in tedesco o
  generici (`Vollbrücke2/3`, `Ausgang`, `Subsystem1/3`, `Trafo (T-ESB)`). Il repo usa nomi inglesi
  descrittivi (es. `dab_single_phase_signal_flow/`, `primary_bridge`, `output_stage`).
- Residui: `Trafo Einfach` e il FIR commentati; `Lm`/`RFe` nel `Trafo Einfach` calcolati e buttati
  su Terminator. Da togliere o documentare.
- Nessun README che dichiari topologia, potenza, fsw, cosa misura il modello e come lo si lancia.

## 7. Cosa chiedere a Tobias

1. Lo script o il `.mat` con `TA, R1, R2, L1, L2, Lm, N, Delta, R_in_filter, L_in_filter,
   C_in_filter, S_F32` e la configurazione del solver che usa.
2. Topologia e modulazione previste (DAB? raddrizzatore sincrono o a diodi?), fsw, tempi morti,
   potenza nominale, range di tensione batteria.
3. A cosa serve il modello (analisi in Simulink o plant per un target real-time): decide se ha
   senso tenerlo a flusso di segnale o rifarlo in Simscape come gli altri modelli del repo.
4. Il modulatore e la regolazione, se esistono già nel suo modello completo.

## 8. Correzioni proposte, in ordine di priorità

1. Collegare (o fornire) modulatore e regolazione; fino ad allora il modello non è simulabile.
2. Aggiungere `init_model.m` con `param` e definire `S_F32`; sostituire il Multiport Switch di
   `SDE_Blockset`; impostare solver a passo fisso discreto con `FixedStep = TA`.
3. Correggere lo stato "tutti spenti" del primario (soglie ±ε, `ipDC = −|iPri|`, stato a corrente
   nulla) e allineare il significato di `[0 0]` fra i due ponti.
4. Collegare `UDC_p` al ponte primario o togliere il filtro; `i_L(0)` corretto.
5. Rinominare i tag Goto (o renderli tutti locali) e i sottosistemi; portare i numeri in `param`;
   togliere i residui commentati; scrivere il README.

## 9. Prospettiva: modelli di elettronica di potenza in puro Simulink

Idea emersa con questa revisione (Davide, 31/10/2026): costruire modelli di convertitori interamente
in Simulink, a flusso di segnale, senza Simscape, sulla falsariga di questo. Quello che la revisione
suggerisce per farlo bene:

- **Passo e integrazione**: solver a passo fisso discreto con un solo `TA` in `param`; integratori
  come blocchi di libreria mascherati. Eulero esplicito va bene su RL e RC; sugli LC poco smorzati
  serve trapezoidale o Eulero implicito (l'esplicito su un LC non smorzato è instabile per qualsiasi
  passo).
- **Ponti**: un unico blocco "full bridge" con gli stati espliciti (+, −, zero, tutti spenti),
  modalità diodo coerente su entrambi i lati, bilancio energetico garantito per costruzione
  (`iDC = u·i / UDC`), tempi morti generati dal modulatore e non dentro al ponte.
- **Trasformatore**: il modello a T di Tobias come blocco di libreria (ingressi `Uab`, `Ucd`; uscite
  `ip`, `is`), con `Delta` calcolato all'interno dalla maschera; saturazione opzionale.
- **Filtri e carichi**: celle RL, RC, LC con condizioni iniziali parametrizzate; profili di carico
  da `param`, non da costanti nei blocchi.
- **Interfaccia e convenzioni**: le stesse del repo (`param.<gruppo>`, `init_model.m`, funzioni in
  `private/`), blocchi riusabili in `library`, README per cartella, nomi inglesi.
- **Validazione**: confronto con il modello Simscape equivalente (es. `dcdc_zvs_full_bridge`) su un
  caso comune prima di usare i modelli per il progetto.
- **Vantaggi attesi**: velocità (niente solver DAE), lo stesso modello per simulazione e target
  real-time (HIL) in float32. **Limiti**: perdite di commutazione e termica vanno aggiunte a parte;
  gli stati di conduzione dipendenti dalla corrente (diodi, DCM) richiedono logica esplicita.
