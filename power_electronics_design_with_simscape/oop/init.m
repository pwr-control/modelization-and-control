clc

% Definizione del modello da 400V
% Argomenti: name, us, is, freq, udc, fpwm, cfi, lfu, cfu
afe400 = afe("AFE_400V", 400, 360, 50, 660, 4000, 7.2e-3, 2*7.2e-3, 2*7.2e-3, 0.33e-3, 360e-6);

% Definizione del modello da 690V
afe690 = afe("AFE_690V", 690, 270, 50, 1070, 4000, 7.2e-3, 2*7.2e-3, 2*7.2e-3, 0.5e-3, 200e-6);

afe480 = afe("AFE_480V", 480, 360, 60, 750, 4000, 7.2e-3, 2*7.2e-3, 2*7.2e-3, 0.33e-3, 360e-6);

inv690 = inverter("INV_690V", 550, 370, 15.6, 1070, 4000, 0.230e-3);

% Accedere ai dati
disp(afe400.us_nom); 

% Usare i metodi della classe
afe400.displayInfo();
afe690.displayInfo();
afe480.displayInfo();
inv690.displayInfo();

% Puoi anche creare un array di oggetti AFE
listaDispositivi = [afe400, afe690, afe480];