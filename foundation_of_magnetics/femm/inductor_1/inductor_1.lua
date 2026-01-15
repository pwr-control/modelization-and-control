-- Inizializza FEMM e mostra la console
showconsole()
newdocument(0) -- 0 = Problema Magnetico

-- Definizione del problema (freq 0, millimetri, assialsimmetrico)
mi_probdef(0, "millimeters", "axi", 1e-8, 0, 30)

-- --- PARAMETRI (Modifica questi numeri per cambiare tutto!) ---
R_interno = 25    -- Raggio interno bobina (mm)
Spessore = 2      -- Spessore avvolgimento (mm)
Altezza = 120     -- Altezza bobina (mm)
R_nucleo = 24     -- Raggio del nucleo (mm)
Spire = 5         -- Numero di spire
Corrente = 1000   -- Ampere

-- Calcoli automatici coordinate
z_top = Altezza / 2
z_bot = -Altezza / 2
R_esterno = R_interno + Spessore

-- --- DISEGNO ---

-- 1. Disegno il NUCLEO
mi_addnode(0, z_bot)
mi_addnode(R_nucleo, z_bot)
mi_addnode(R_nucleo, z_top)
mi_addnode(0, z_top)

mi_addsegment(0, z_bot, R_nucleo, z_bot)
mi_addsegment(R_nucleo, z_bot, R_nucleo, z_top)
mi_addsegment(R_nucleo, z_top, 0, z_top)
-- L'asse non serve chiuderlo, ma per pulizia visiva lo facciamo se vuoi

-- 2. Disegno la BOBINA
mi_addnode(R_interno, z_bot)
mi_addnode(R_esterno, z_bot)
mi_addnode(R_esterno, z_top)
mi_addnode(R_interno, z_top)

mi_addsegment(R_interno, z_bot, R_esterno, z_bot)
mi_addsegment(R_esterno, z_bot, R_esterno, z_top)
mi_addsegment(R_esterno, z_top, R_interno, z_top)
mi_addsegment(R_interno, z_top, R_interno, z_bot)

-- 3. Disegno l'ARIA (Un cerchio grande)
mi_drawarc(0, -200, 0, 200, 180, 5) -- Un semicerchio grande

-- --- MATERIALI E PROPRIETA' ---

-- Carico materiali dalla libreria
mi_getmaterial("Air")
mi_getmaterial("Copper")
mi_getmaterial("Magnetics 77 ferrite") -- O "M-19 Steel"

-- Definisco il circuito
mi_addcircprop("MioCircuito", Corrente, 1) -- 1 = Serie

-- Assegno le etichette (Block Labels)
-- Aria
mi_addblocklabel(50, 0)
mi_selectlabel(50, 0)
mi_setblockprop("Air", 1, 0, "incircuit", 0, 0, 0)
mi_clearselected()

-- Nucleo
mi_addblocklabel(1, 0)
mi_selectlabel(1, 0)
mi_setblockprop("Magnetics 77 ferrite", 1, 0, "incircuit", 0, 0, 0)
mi_clearselected()

-- Bobina
mi_addblocklabel(R_interno + 0.5, 0)
mi_selectlabel(R_interno + 0.5, 0)
mi_setblockprop("Copper", 1, 0, "MioCircuito", 0, 0, Spire)
mi_clearselected()

-- Boundary Condition (Infinito)
mi_addboundprop("Open", 0, 0, 0, 0, 0, 0, 0, 0, 0)
mi_selectarcsegment(0, 200) -- Seleziona l'arco dell'aria
mi_setarcsegmentprop(5, "Open", 0, 0) -- Assegna boundary

-- Zoom per vedere tutto
mi_zoomnatural()