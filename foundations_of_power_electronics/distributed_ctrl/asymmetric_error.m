function [rho, delta_phi] = asymmetric_error(voltage_dip, error_type)
    % Voltage dip bezieht sich auf verbleibende verkettete Spannung
    % Vor Fehler U_23 = sqrt(3)
    % Nach Fehler U_23 = sqrt(3)*(1-dip)
    % error_type = 0 -> Variant C, error_type = 1 -> Variant D
    if error_type == 0
        imag = sqrt(3)/2*(1-voltage_dip);
        real = -1/2;
    else
        imag = sqrt(3)/2;
        real = -1/2*(1-voltage_dip);
    end
    [theta, rho] = cart2pol(real, imag);
    delta_phi = theta - 2/3*pi;
end