classdef afe
    properties
        name          string
        us_nom        double {mustBePositive} % Nominal AC-grid voltage [V]
        is_nom        double {mustBePositive} % Nominal AC-grid current [A]
        f_nom         double {mustBePositive} % Nominal AC-grid frequency [Hz]
        udc_nom       double {mustBePositive} % Nominal DC-link voltage [V]
        fpwm_base     double {mustBePositive} % Base switching frequency [Hz]
        CFi           double {mustBePositive} % DClink capacitor [F]
        CFi_1         double {mustBePositive} % DClink capacitor bank 1 [F]
        CFi_2         double {mustBePositive} % DClink capacitor bank 2 [F]
        LFu           double {mustBePositive} % Output filter inductace [H]
        CFu           double {mustBePositive} % Output filter capacitor [F]
    end
    
    methods
        function obj = afe(name, us, is, freq, udc, fpwm, cfi, cfi_1, cfi_2, lfu, cfu)
            if nargin > 0
                obj.name = name;
                obj.us_nom = us;
                obj.is_nom = is;
                obj.f_nom = freq;
                obj.udc_nom = udc;
                obj.fpwm_base = fpwm;
                obj.CFi = cfi;
                obj.CFi_1 = cfi_1;
                obj.CFi_2 = cfi_2;
                obj.LFu = lfu;
                obj.CFu = cfu;
            end
        end
        
        function ibez = get_normalization_current_value(obj)
            ibez =  obj.is_nom * sqrt(2);
        end
        function ubez = get_normalization_voltage_value(obj)
            ubez =  obj.us_nom * sqrt(2/3);
        end   
        function displayInfo(obj)
            fprintf('Device AFE: %s\n', obj.name);
            fprintf('Nominal Voltage: %d V | Nominal Current: %d A\n', obj.us_nom, obj.is_nom);
            fprintf('Current Normalization Data: %.2f A\n', obj.get_normalization_current_value());
            fprintf('Voltage Normalization Data: %.2f V\n', obj.get_normalization_voltage_value());
            fprintf('---------------------------\n');
        end
    end
end