

%% LVRT grid emulator
switch test_index
    case 0
        % No FRT - For testing of standard AFE control (DCLink)
        Amplitude_U = 1;
        Amplitude_V = 1;
        Amplitude_W = 1;
        Phase_shift = 0;
        error_length = 0;
        P_load_norm = 1;
        Q_load_norm = 0.1;
        k_lvrt = 2;
    case 25
        switch test_subindex
            case 1
                % Test 25.1: 75% dip - three-phase - full load
                Amplitude_U = 0.25;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 760e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 2
                % Test 25.2: 75% dip - three-phase - partial load
                Amplitude_U = 0.25;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 760e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 3
            case 4
                % Test 25.4: 75% dip - two-phase - full load
                [Voltage_LVRT, delta_phi] = asymmetric_error(0.75, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 0.25;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end

                error_length = 915e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 5
                % Test 25.5: 75% dip - two-phase - partial load
                [Voltage_LVRT, delta_phi] = asymmetric_error(0.75, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 0.25;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 915e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 2;
            otherwise
                error('Test-Index not known!')
        end
    case 50
        switch test_subindex
            case 1
                % Test 50.1: 50% dip - three-phase - full load
                Amplitude_U = 0.5;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 1982e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 2
                % Test 50.2: 50% dip - three-phase - partial load
                Amplitude_U = 0.5;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 1982e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 3
                % Test 50.3: 50% dip - two-phase - full load
                [Voltage_LVRT, delta_phi] = asymmetric_error(0.5, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 0.5;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 2305e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 4
                % Test 50.4: 50% dip - two-phase - partial load
                [Voltage_LVRT, delta_phi] = asymmetric_error(0.5, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 0.5;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 2305e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 5
                % Test 50.5: 50% dip - three-phase - full load - Reduced dym. grid-support
                Amplitude_U = 0.5;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 1982e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 6
                % Test 50.6: 50% dip - two-phase - full load - Reduced dym. grid-support
                [Voltage_LVRT, delta_phi] = asymmetric_error(0.5, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 0.5;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 2305e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            otherwise
                error('Test-Index not known!')
        end
    case 75
        switch test_subindex
            case 1
                % Test 75.1: 25% dip - three-phase - full load
                Amplitude_U = 0.75;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 2796e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 2
                % Test 75.2: 25% dip - three-phase - partial load
                Amplitude_U = 0.75;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 2796e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 3
                % Test 75.3: 25% dip - three-phase - partial load - max. untererregt
                Amplitude_U = 0.75;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 2796e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0.33;
                k_lvrt = 2;
            case 4
                % Test 75.4: 25% dip - three-phase - partial load - max. übererregt
                Amplitude_U = 0.75;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 2796e-3;
                P_load_norm = 0.5;
                Q_load_norm = -0.33;
                k_lvrt = 2;
            case 5
                % Test 75.5: 25% dip - three-phase - partial load - k = 4
                Amplitude_U = 0.75;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 2796e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 4;
            case 6
                % Test 75.6: 25% dip - two-phase - full load
                [Voltage_LVRT, delta_phi] = asymmetric_error(0.25, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 0.75;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 3000e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 7
                % Test 75.7: 25% dip - two-phase - partial load
                [Voltage_LVRT, delta_phi] = asymmetric_error(0.25, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 0.75;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 3000e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 8
                % Test 75.8: 25% dip - two-phase - partial load - k = 4
                [Voltage_LVRT, delta_phi] = asymmetric_error(0.25, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 0.75;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 3000e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 4;
            otherwise
                error('Test-Index not known!')
        end
    case 80
        switch test_subindex
            case 1
                % Test 80.1: 20% dip - three-phase - full load - Reduced dym. grid-support
                Amplitude_U = 0.8;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 3000e-3;
                P_load_norm = 1;
                Q_load_norm = 0.1;
                k_lvrt = 2;
            case 2
                % Test 80.2: 20% dip - two-phase - full load - Reduced dym. grid-support
                [Voltage_LVRT, delta_phi] = asymmetric_error(0.20, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 0.80;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 3000e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            otherwise
                error('Test-Index not known!')
        end
    case 85
        switch test_subindex
            case 1
                % Test 85.1: 12% dip - three-phase - full load
                Amplitude_U = 0.88;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 60000e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            otherwise
                error('Test-Index not known!')
        end
    case 110
        switch test_subindex
            case 1
                % Test 110.1: 10% rise - two-phase - full load
                [Voltage_LVRT, delta_phi] = asymmetric_error(-0.15, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 1.15;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 5000e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 2
                % Test 110.2: 10% rise - two-phase - partial load
                [Voltage_LVRT, delta_phi] = asymmetric_error(-0.15, asymmetric_error_type);
                if asymmetric_error_type == 0
                    Amplitude_U = 1;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                else
                    Amplitude_U = 1.15;
                    Amplitude_V = Voltage_LVRT;
                    Amplitude_W = Amplitude_V;
                    Phase_shift = delta_phi;
                end
                error_length = 5000e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 3
                % Test 110.3: 10% rise - three-phase - partial load
                Amplitude_U = 1.16;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 60000e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 2;
            otherwise
                error('Test-Index not known!')
        end
    case 115
        switch test_subindex
            case 1
                % Test 115.1: 15% rise - three-phase - full load
                Amplitude_U = 1.16;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 5000e-3;
                P_load_norm = 1;
                Q_load_norm = 0;
                k_lvrt = 2;
            case 2
                % Test 115.2: 15% rise - three-phase - partial load
                Amplitude_U = 1.16;
                Amplitude_V = Amplitude_U;
                Amplitude_W = Amplitude_U;
                Phase_shift = 0;
                error_length = 5000e-3;
                P_load_norm = 0.5;
                Q_load_norm = 0;
                k_lvrt = 2;
            otherwise
                error('Test-Index not known!')
        end
    otherwise
        error('Test-Index not known!')
end

end_time_LVRT = start_time_LVRT + error_length;

% Ramp time of phase voltage
t_ramp = 20e-3;