g = 9.81;
lp = 1288; % /m length of the plant
pd = 4.9; % /m pulley diameter
pr = pd/2; % /m pulley radius
m_rope_pu = 7.67; % /kg\m mass of the rope for unity of length 
nv = 58; % number of veicles
mv_min = 850; % /kg minimum weigth of a vehicle
mv_max = 1490; % /kg maximum weigth of a vehicle
plant_max_slope = 27; % /deg maximum slop plant
young_module_rope = 125e3; % /N\mm^2
srope = 872; % /mm^2
torque_nom = 180e3; % /Nm nominal torque
Jr = 1979; % /kgm^2

length_rope = lp*2;
rope_mass = length_rope * m_rope_pu;
whole_mass_vehicles_max = mv_max*nv;
whole_mass_vehicles_min = mv_min*nv;

ratio = 0.5; % ration between "m-pulley" and "l-pulley" of the inertia

Jm_max = 1/3*(rope_mass+whole_mass_vehicles_max)*(ratio)*(pr)^2+Jr; % /kg*m^2
Jl_max = 1/3*(rope_mass+whole_mass_vehicles_max)*(1-ratio)*(pr)^2; % /kg*m^2
Jm_min = 1/3*(rope_mass+whole_mass_vehicles_min)*(ratio)*(pr)^2+Jr; % /kg*m^2
Jl_min = 1/3*(rope_mass+whole_mass_vehicles_min)*(1-ratio)*(pr)^2; % /kg*m^2
Ktheta = 125000;

Dl = torque_nom/10;
Dm = torque_nom/10;
Dtheta = torque_nom/10;

f0_min = 1/2/pi*sqrt(Ktheta*(1/Jm_max+1/Jl_max)) %Hz
f0_max = 1/2/pi*sqrt(Ktheta*(1/Jm_min+1/Jl_min)) %Hz


