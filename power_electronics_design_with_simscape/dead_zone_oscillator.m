function dead_zone_oscillator()
    % Parameters
    tspan = [0 50];      % Time range
    y0 = [2.5; 0];       % Initial conditions [position; velocity]
    
    % Solve using ode45
    [t, y] = ode45(@(t, y) system_dynamics(t, y), tspan, y0);

    % Plotting Displacement
    subplot(2,1,1);
    plot(t, y(:,1), 'LineWidth', 1.5);
    ylabel('Displacement (x)');
    title('Dead-Zone Oscillator Response');
    grid on;

    % Plotting Phase Portrait
    subplot(2,1,2);
    plot(y(:,1), y(:,2), 'r', 'LineWidth', 1.5);
    xlabel('Position (x)');
    ylabel('Velocity (v)');
    title('Phase Portrait');
    grid on;
end

function dydt = system_dynamics(~, y)
    x = y(1);
    v = y(2);
    
    % Physical Constants
    k = 5;      % Stiffness outside dead zone
    c = 0.2;    % Damping
    delta = 1;  % Dead zone threshold
    
    % Dead-zone Restoring Force Logic
    if x > delta
        F_res = -k * (x - delta);
    elseif x < -delta
        F_res = -k * (x + delta);
    else
        F_res = 0; % Inside the Dead Zone
    end
    
    % ODEs: dx/dt = v; dv/dt = Accel
    dydt = [v; F_res - c*v];
end