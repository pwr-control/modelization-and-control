classdef inverter
    properties
        name          string
        us_nom        double {mustBePositive} % Nominal AC-grid voltage [V]
        is_nom        double {mustBePositive} % Nominal AC-grid current [A]
        f_nom         double {mustBePositive} % Nominal AC-grid frequency [Hz]
        udc_nom       double {mustBePositive} % Nominal DC-link voltage [V]
        fpwm_base     double {mustBePositive} % Base switching frequency [Hz]
        LFi           double {mustBePositive} % Output filter inductace [H]
    end
    
    methods
        function obj = inverter(name, us, is, freq, udc, fpwm, lfi)
            if nargin > 0 % Se vengono passati argomenti
                obj.name = name;
                obj.us_nom = us;
                obj.is_nom = is;
                obj.f_nom = freq;
                obj.udc_nom = udc;
                obj.fpwm_base = fpwm;
                obj.LFi = lfi;
            end
        end
        
        function ibez = get_normalization_current_value(obj)
            ibez =  obj.is_nom * sqrt(2);
        end
        function ubez = get_normalization_voltage_value(obj)
            ubez =  obj.us_nom * sqrt(2/3);
        end   
        function displayInfo(obj)
            fprintf('Device INVERTER: %s\n', obj.name);
            fprintf('Nominal Voltage: %d V | Nominal Current: %d A\n', obj.us_nom, obj.is_nom);
            fprintf('Current Normalization Data: %.2f A\n', obj.get_normalization_current_value());
            fprintf('Voltage Normalization Data: %.2f V\n', obj.get_normalization_voltage_value());
            fprintf('---------------------------\n');
        end
    end
end