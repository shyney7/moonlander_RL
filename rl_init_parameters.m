M       = 7.346e22;         % mass of moon [kg]
R       = 3474e3/2;         % radius of moon [m]
gamma   = 6.6743e-11;       % gravitational constant
m0_sol  = 2e3;              % mass of the lunar spacecraft without fuel [kg]
h0      = 10e3;             % initial altitude above ground level [m]
Pmax    = 45e3;             % maximum thrust of descent propulsion system [N]
mdmax   = Pmax/2500;        % fuel rate at max thrust Pmax/deltav_fuel
m0_fuel = 8e3;              % initial fuel mass [kg]
m0      = m0_sol + m0_fuel; % total initial mass of spacecraft

F0      = gamma * M * m0 / (h0 + R)^2;
Ff      = gamma * M * m0 / R^2;
P_eq_0  = Ff/Pmax*100;
Ff_80   = gamma * M * (m0_sol+0.8*m0_fuel) / R^2;
P_eq_80 = Ff_80/Pmax*100;

A = [0,1;0,0];
B = [0; Pmax/(100*m0)];
C = [1, 0];
D = 0;
sys = ss(A,B,C,D);
poles = [-1/20-1/40*1i; -1/20+1/40*1i];
K = place(A,B,poles);
V = inv(C*inv(B*K-A)*B);
t = [0,150,200,250,500];
hs= [10e3, 10e2, 200, 0, 0];