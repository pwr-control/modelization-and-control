clc

g = 9.81;
lp = 5474; % /m length of the plant
pd = 4.9; % /m pulley diameter
pr = pd/2; % /m pulley radius
m_rope_pu = 9.3; % /kg\m mass of the rope for unity of length 
nv = 61; % number of veicles
mv_min = 950; % /kg minimum weigth of a vehicle
mv_max = 1750; % /kg maximum weigth of a vehicle
% plant_max_slope = 180/pi*atan((2340-1985)/878) % /deg maximum slop plant
young_module_rope = 100.75e3; % /N\mm^2
% s_rope = 25^2*pi; % /mm^2
torque_nom = 324e3; % /Nm nominal torque
Jr = 3428; % /kgm^2

length_rope = lp*2;
rope_mass = length_rope * m_rope_pu;
whole_mass_vehicles_max = mv_max*nv;
whole_mass_vehicles_min = mv_min*nv;

ratio = 0.5; % ration between "m-pulley" and "l-pulley" of the inertia

Jm_max = (rope_mass+whole_mass_vehicles_max)*(ratio)*(pr)^2+Jr % /kg*m^2
Jl_max = (rope_mass+whole_mass_vehicles_max)*(1-ratio)*(pr)^2 % /kg*m^2
Jm_min = (rope_mass+whole_mass_vehicles_min)*(ratio)*(pr)^2+Jr; % /kg*m^2
Jl_min = (rope_mass+whole_mass_vehicles_min)*(1-ratio)*(pr)^2; % /kg*m^2
Ktheta = young_module_rope

% Dl = torque_nom/50;
% Dm = torque_nom/5;
% Dtheta = torque_nom/50;

Dl = torque_nom/5
Dm = torque_nom/5
Dtheta = torque_nom/50

f0_min = 1/2/pi*sqrt(Ktheta*(1/Jm_max+1/Jl_max)) %Hz
f0_max = 1/2/pi*sqrt(Ktheta*(1/Jm_min+1/Jl_min)) %Hz

linear_speed = 7; % m/s
omega_nom_rpm = linear_speed/pr/2/pi*60
omega_nom = omega_nom_rpm/60*2*pi;
