%[text] # Fuzzy Control Moon Landing Challenge
%[text] This is a simplified moon landing simulation.
%[text] Parameters of the system:
M       = 7.346e22;         % mass of moon [kg]
R       = 3474e3/2;         % radius of moon [m]
gamma   = 6.6743e-11;       % gravitational constant
m0_sol  = 2e3;              % mass of the lunar spacecraft without fuel [kg]
h0      = 10e3;             % initial altitude above ground level [m]
Pmax    = 45e3;             % maximum thrust of descent propulsion system [N]
mdmax   = Pmax/2500;        % fuel rate at max thrust Pmax/deltav_fuel
m0_fuel = 8e3;              % initial fuel mass [kg]
m0      = m0_sol + m0_fuel; % total initial mass of spacecraft
%%
%[text] Calculate the initial and final gravitational force with initial mass:
F0      = gamma * M * m0 / (h0 + R)^2 %[output:5ea58e2b]
Ff      = gamma * M * m0 / R^2 %[output:6e6f8f2a]
%[text] Calculate the equilibrium thrust settings for full and 80% full vehicle:
P_eq_0  = Ff/Pmax*100 %[output:9a0670ac]
Ff_80   = gamma * M * (m0_sol+0.8*m0_fuel) / R^2;
P_eq_80 = Ff_80/Pmax*100 %[output:19bbcd52]
%%
%[text] ## Simplified State Space Model for a Classic Approach
%[text] Equations of motion read:
%[text] $\\dot{v} = 1/m\_0 (F\_{thrust}-F\_g)\\\\\n\\dot{h} = v$
%[text] Gravitational force Fg is now assumed to be constant and the thrust is proportional to an input signal u \[0,100%\]:
%[text] $F\_{thrust} = \\frac{P\_{max}}{100} \\cdot u$
%[text] hence with x1=h and x2=v:
%[text] $\\dot{x} = \\left\[\\begin{array}{l}\nx\_2\\\\\n1/m\_0 \\cdot (P\_{max}/100\\cdot u - F\_f)\n\\end{array}\n\\right\]$
%[text] Let $u\_0\n$ be the thrust equal to Fg and $u = u\_0+\\Delta u\n$. Then the state space model reads
%[text] $\\dot x = \\left\[ \\begin{array}{ll}\n0 & 1 \\\\\n0 & 0\n\\end{array}\n\\right\] \\cdot x + \n\\left\[ \\begin{array}{c}\n0  \\\\\n\\frac{P\_{max}}{100\\cdot m\_0} \n\\end{array}\n\\right\] \\cdot \\Delta u$
%[text] Output equation:
%[text] $y=\[1\~\~0\]\\cdot x$
%[text] A state space controller u=-K\*x with appropriate negative real poles to avoid crash:
A = [0,1;0,0];
B = [0; Pmax/(100*m0)];
C = [1, 0];
D = 0;
sys = ss(A,B,C,D);
poles = [-1/20-1/40*1i; -1/20+1/40*1i];
K = place(A,B,poles) %[output:045f08f6]
V = inv(C*inv(B*K-A)*B) %[output:082154b6]
t = [0,150,200,250,500] %[output:4fef88b6]
hs= [10e3, 10e2, 200, 0, 0];
%%
%[text] ## Fuzzy Approach ...
%[text] The first approach is Fuzzy Control straight forward. We use altitude and velocity as input variables for a FIS that creates thrust as output. We use the expertise we gained by manually trying to land the spacecraft by observing altitude and velocity to set the thrust.
% create single comprehensive fis:
fis_full    = sugfis("Name","lunar_full");
fis_full    = addInput(fis_full,[0 1e4],"Name","altitude");
fis_full    = addInput(fis_full,[-120, +20],"Name","velocity");
fis_full    = addOutput(fis_full,[0, 100], "Name", "thrust");
% define input membership functions for 1st input:
fis_full    = addMF(fis_full,"altitude","trimf",[-300, 0, 300],"Name","very low");
fis_full    = addMF(fis_full,"altitude","trimf",[0, 1500, 3000],"Name","low");
fis_full    = addMF(fis_full,"altitude","linsmf",[1500, 3000],"Name","high");
figure(1); plotmf(fis_full,"input",1) %[output:7b7fa337]
% define input membership functions for 2nd input:
fis_full    = addMF(fis_full,"velocity","linzmf",[-80,-60],"Name","very fast down");
fis_full    = addMF(fis_full,"velocity","trapmf",[-80,-70,-30,-20],"Name","fast down");
fis_full    = addMF(fis_full,"velocity","trapmf",[-30,-20,-10,-5],"Name","moderate down");
fis_full    = addMF(fis_full,"velocity","trapmf",[-10,-5,-2,0],"Name","slowly down");
fis_full    = addMF(fis_full,"velocity","linsmf",[0,5],"Name","upwards");
figure(2); plotmf(fis_full,"input",2) %[output:0b37f5e8]
fis_full = addMF(fis_full,"thrust","constant",0 ,"Name","off");
fis_full = addMF(fis_full,"thrust","constant",25 ,"Name","low");
fis_full = addMF(fis_full,"thrust","constant",31 ,"Name","medium");
fis_full = addMF(fis_full,"thrust","constant",60 ,"Name","high");
fis_full = addMF(fis_full,"thrust","constant",100 ,"Name","maximum");
% Add rules:
% Column 1 — Index of membership function for first input h
% Column 2 — Index of membership function for second input v
% Column 3 — Index of membership function for output P
% Column 4 — Rule weight (from 0 to 1)
% Column 5 — Fuzzy operator (1 for AND, 2 for OR)
rules = ... %[output:group:89ab01fb] %[output:78405e0b]
    [1, 4, 3, 1.0, 1; ...   % IF h=very low AND v=slowly down THEN P=medium %[output:78405e0b]
     1, 3, 4, 1.0, 1; ...   % IF h=very low AND v=moderate down THEN P=high %[output:78405e0b]
     1, 2, 4, 1.0, 1; ...   % IF h=very low AND v=fast down THEN P=high %[output:78405e0b]
     1, 1, 5, 1.0, 1; ...   % IF h=very low AND v=very fast down THEN P=maximum %[output:78405e0b]
     2, 1, 5, 1.0, 1; ...   % IF h=low AND v=very fast down THEN P=maximum %[output:78405e0b]
     2, 2, 4, 1.0, 1; ...   % IF h=low AND v=fast down THEN P=high %[output:78405e0b]
     2, 3, 3, 1.0, 1; ...   % IF h=low AND v=moderate down THEN P=medium %[output:78405e0b]
     2, 4, 3, 1.0, 1; ...   % IF h=low AND v=slowly down THEN P=low %[output:78405e0b]
     3, 1, 4, 1.0, 1; ...   % IF h=high AND v=very fast down THEN P=high %[output:78405e0b]
     3, 2, 3, 1.0, 1; ...   % IF h=high AND v=fast down THEN P=medium %[output:78405e0b]
     3, 3, 2, 1.0, 1; ...   % IF h=high AND v=moderate down THEN P=low %[output:78405e0b]
     3, 4, 1, 1.0, 1; ...   % IF h=high AND v=slowly down THEN P=off %[output:78405e0b]
     0, 5, 1, 1.0, 1]       % IF v=upwards THEN P=off    ] %[output:group:89ab01fb] %[output:78405e0b]
fis_full = addRule(fis_full,rules);
figure(6); gensurf(fis_full) %[output:7182b99a]
writeFIS(fis_full,fis_full.Name)
%[text]  
%[text] ## Create control theory like Sugeno FISes
%[text] This approach generates 2 fuzzy inference systems. The first will generate a reference velocity trajectory dependent on the altitude of the vehicle. The second FIS computes the required thrust dependent on the control error. The control error is the difference between reference velocity and current velocity of the spacecraft:
%[text] $e = v\_{ref} - v\n$
%[text] Both velocity variables are negative. If e.g. v\_ref = -80 and v = -100, e=+20 and thrust should be higher to decrease speed. 
%[text] ### FIS for generating reference velocity dependend on altitude
% create fis for reference velocity:
fis_refspeed = sugfis("Name","refspeed");
fis_refspeed = addInput(fis_refspeed,[0 1e4],"Name","altitude");
fis_refspeed = addOutput(fis_refspeed,[-120, 0],"Name","ref_vel");
% input membership functions:
fis_refspeed = addMF(fis_refspeed,"altitude","trimf",[-300, 0, 300],"Name","very low");
fis_refspeed = addMF(fis_refspeed,"altitude","trimf",[0, 1500, 3000],"Name","low");
fis_refspeed = addMF(fis_refspeed,"altitude","linsmf",[1500, 3000],"Name","high");
figure(3); plotmf(fis_refspeed,"input",1) %[output:62f3885a]
% output membership functions:
fis_refspeed = addMF(fis_refspeed,"ref_vel","constant",-100,"Name","fast");
fis_refspeed = addMF(fis_refspeed,"ref_vel","constant",-20,"Name","moderate");
fis_refspeed = addMF(fis_refspeed,"ref_vel","constant",-1,"Name","slow");
% rules:
% Column 1 — Index of membership function for first input
% Column 2 — Index of membership function for output
% Column 3 — Rule weight (from 0 to 1)
% Column 4 — Fuzzy operator (1 for AND, 2 for OR)
rules_refspeed = [ ...
    1, 3, 1, 1; ... % if altitude==very low => ref_vel=slow
    2, 2, 1, 1; ... % if altitude==low => ref_vel=moderate
    3, 1, 1, 1];    % if altitude==high => ref_vel=fast
fis_refspeed = addRule(fis_refspeed,rules_refspeed);
figure(4); gensurf(fis_refspeed) %[output:43e43514]
writeFIS(fis_refspeed,fis_refspeed.Name)
%[text] ### FIS for generating thust from control error
%[text] 
% create fis for reference velocity:
fis_thrust = sugfis("Name","thrust");
fis_thrust = addInput(fis_thrust,[-50,50],"Name","vel_err");
fis_thrust = addOutput(fis_thrust,[0,100],"Name","thrust");
% input membership functions:
fis_thrust = addMF(fis_thrust,"vel_err","linzmf",[-20,-10],"Name","much slower");
fis_thrust = addMF(fis_thrust,"vel_err","trimf",[-20,-10,0],"Name","slower");
fis_thrust = addMF(fis_thrust,"vel_err","trimf",[-5,0,5],"Name","good");
fis_thrust = addMF(fis_thrust,"vel_err","trimf",[0,10,20],"Name","fast");
fis_thrust = addMF(fis_thrust,"vel_err","linsmf",[10,20],"Name","much faster");
figure(5); plotmf(fis_thrust,"input",1) %[output:0663df05]
% output membership functions:
fis_thrust = addMF(fis_thrust,"thrust","constant",0 ,"Name","off");
fis_thrust = addMF(fis_thrust,"thrust","constant",20 ,"Name","low");
fis_thrust = addMF(fis_thrust,"thrust","constant",30 ,"Name","medium");
fis_thrust = addMF(fis_thrust,"thrust","constant",50 ,"Name","high");
fis_thrust = addMF(fis_thrust,"thrust","constant",100 ,"Name","maximum");
rules_thrust = [ ...
    1, 1, 1, 1; ... % if vel_error==much slower => thrust=off
    2, 2, 1, 1; ... % if vel_error==slower => thrust=low
    3, 3, 1, 1; ... % if vel_error==good => thrust=medium
    4, 4, 1, 1; ... % if vel_error==faster => thrust=high
    5, 5, 1, 1];    % if vel_error==much faster => thrust=maximum
fis_thrust = addRule(fis_thrust,rules_thrust);
figure(6); gensurf(fis_thrust) %[output:692e32ac]
writeFIS(fis_thrust,fis_thrust.Name)

%[text] ## 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":6.7}
%---
%[output:5ea58e2b]
%   data: {"dataType":"textualVariable","outputData":{"name":"F0","value":"1.6065e+04"}}
%---
%[output:6e6f8f2a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ff","value":"1.6250e+04"}}
%---
%[output:9a0670ac]
%   data: {"dataType":"textualVariable","outputData":{"name":"P_eq_0","value":"36.1114"}}
%---
%[output:19bbcd52]
%   data: {"dataType":"textualVariable","outputData":{"name":"P_eq_80","value":"30.3336"}}
%---
%[output:045f08f6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"K","rows":1,"type":"double","value":[["0.0694","2.2222"]]}}
%---
%[output:082154b6]
%   data: {"dataType":"textualVariable","outputData":{"name":"V","value":"0.0694"}}
%---
%[output:4fef88b6]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"t","rows":1,"type":"double","value":[["0","150","200","250","500"]]}}
%---
%[output:7b7fa337]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAACYAAAAmCAYAAACoPemuAAAAAXNSR0IArs4c6QAABfFJREFUWEftl3lsjGsYxc8oFUvFnnKjCFFCpAhSS4gttiKERAmNiCW21r5H7MaWKBGUWINKLG0aYt9FKaWlC0EqiLh2ouWPufk9N99kyvSmcyO59495k0mm873Lec85z3m+ujwej0f\/w+EKAgtQlSBjARKmIGNBxgJlIND5QY\/9FsYOHDigffv2aerUqZo\/f74OHz6syMjIYntfu3ZNkyZN8vusJBCBrHElJyd7Fi5caHstX75cQ4cOlT9gERERWrBggdLS0kR75TNu3DjNmjWr1GQEBGzIkCGe1atX2+Zz584V32\/evPkLY7Vq1dKoUaOUl5enMmXKaNCgQVqzZk2pQTExIGAJCQmeFStWqEKFClq7dq0aNGig79+\/G7CuXbtq3rx53sPr1q2rtm3bKj09Xa9evbLfmzZtquPHj5vUw4YN05EjR7zzt23bpvPnz+vUqVO2\/4YNG+wyWGTChAn68eOHzQ0PD9fFixe9duECLrfb7XHkABgDACNHjtTAgQP15MkTO8ztdmvXrl0GnOfVqlVTdna2Jk+ebHJu3bpVEydOtPWLFy829hMSEgQ4BhdCje7du6tSpUr2OXr0qGrWrKlevXrZnNOnTxs4A4bH8BUDAM+ePbODATlnzhy7IeZftmyZkpOTVa5cOdWpU0chISH69OmTsZGYmKjUtBRFRRTqz6I\/FBIapkuXLqmgoECdOnXSt2\/f1Lx5c8XHxys2NlaNGjVS\/fr1bb+KFStqypQpun\/\/vlq3bq2NGzfqzp07JQP7uSqnT5+uCxcu2M3btGmjGzdu6MGDB7py5YoVQXZmhuJiI5WX5VJ6foHOnDmj6tWr29zKlSurWbNm6tOnj5YuXWog+\/btq0WLFhlomH306JF599ChQ8rNzS1Zyp+BISNycVCrVq309OlTO\/zYsWO6d++exUpMdJRmuxPVs2dPkx4GkKlly5b2W35+vvnxy5cvfoumR48eJQPzNb9vjgF0y5YtioqKMmBQT\/WeO3dOd+\/eFZHjxE3nzp3N0F26dNGqVas0ZswYdejQwWIIievVq2c+qlGjhpcxkCIjRWIeK01c7Nmzx2hnU2IDA1+\/fl0vX760qsNjSBsdHW3fMfmOHTusQPAZUo0YMcLiBnabNGlilwKEIyW+glnWez3mL2B3796tsLAwq7zU1FS9efPGPFJYWKgqVarYoWXLltXYsWOLVeX+\/fvVuHFjk+7z588mGb\/17t1b\/fr1M0CENescjw0fPlwnTpww9rt16\/Y3Y\/7+GaFysrKyVLVqVd26dcuqEGO+ffvWqrGoqEhfv361Q7n14MGDLdvIJfKN5wyXy2UVTmY5kjsMbd682btHaGioypcvLzyWlJSknJycX4EREzzEX5mZmRYXVJeTcWQWRkdWJMZr5BYsINn69estRmCIXoq8rJ8xY4b69+9v+z5+\/NikZowePdq8uHLlymJneBl79+6dxo8fbywxkOP169eiK3BLns2cOVPt27e3Ijh48KDlEIYmKD98+KCOHTva2oyMDE2bNk3Pnz83G8AgVU1Fc9GTJ0\/aGsaAAQMUExNjmQlgnvMpBuz9+\/dGPc2aICWVYYgFTh8FsMMYqc7Nb9++rYYNG9pFzp49640CLkNboxh27txpwGAVe6SkpAgyfBnzPecXj8EOwD5+\/KirV6\/aIbxJ1K5d2xgCOCGJp5zqo7EToJQ7oPET7SkuLs6ybsmSJdaaWrRoYc+Rf+\/evQaMbtCuXTuzA3\/DHB+\/wGDnxYsXJhuMUZkYmLBkg\/8MGIxhWqeHQjHNmnjgpuvWrTPz+0rJWwaVhuHxGG2K73iJi\/w2KX2B+RbGpk2bzMgcSNPnQMxMD0UyPERAwjoxQgAjH0VC88bYvOKwHttweeZxCb\/mdxzLZOR7+PChtRPkBBiVAysAKW1cIDt7XL582drSv4oLX2BISXvgNRqjMmgphC5\/84wU51AGrza8ZRCoDCTdvn27dy7sETW0MNZRncjL4EXUOYd9sQe5F\/z3ze97zj\/8GGQsyFigDAQ6\/y9ofGl2JZLYoQAAAABJRU5ErkJggg==","height":30,"width":30}}
%---
%[output:0b37f5e8]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAACYAAAAmCAYAAACoPemuAAAAAXNSR0IArs4c6QAABXxJREFUWEftV2lIVVsU\/m5WzqkZL7NuOaeYV+vHiwaDNExQVFATQRDLARx+aDlgBUU22IAYFJQNIg6of0QoE\/FpluAAopaBGC8Vx6i0KMuy7uNbcC8O+eqWvdePu+HAPefsvfba6xvWuQq1Wq3GbzgU+sR0REVfMR0LBn3F9BXTtQK6ztdz7Kcq1tzcjN27d0uMhIQE5OXl4ezZs2hsbMSTJ09QX18PT0\/Pf92jt7cXN2\/ehJubG\/Lz81FeXo6Kigr4+Phg165dc9Zq9nv48OGCd4qKigr1sWPHwJZpZ2cHb29vHD9+XAJwk\/DwcFhaWqKnpwcBAQFobW2Vubzi4+ORnp4+Z7NTp05heHgY27dvl8TOnTuHrKwsXLlyRbfEQkND1Vw8OTmJwMBAZGRkIC0t7auJbdmyBSMjI1i2bBlCQkKQm5u7oHpLlhgAnb4uVqxYgU+fPs1JaPXq1Xj16tWCJDds2IChoaFFod+zZw+amppAKPv7+5GdnY3BwUGZr1CpVGoGJo\/MzMzw4cMHbNu2DW1tbTA3N5d7JmJsbAwLCwsYGhpiYGAAHh4eSE1Nxfnz5yXYly9fhJecd+vWLTx\/\/lz4SDRiYmLw9u1buUpLS+Hn54egoCB0dnZiamoKd+7cQW1tLa5fv461a9fi0KFDUKSkpKgVCoUsIJxWVlYgHDk5OXjz5o0k5+DgINzi4Maa53FxcaipqRFhWJmbwt\/XBzabbTE1CS3HkpOTwYuc5IZXr16VOLOFlpSUJJV79+4drK2tcf\/+fSgIJW9cXV3R0tIiAjh58iROnDghSmJCGzduxN27d\/H58+dFYSHvYvM34++\/jOD4x584cOCAcHX\/\/v2ibh6+sLAQkZGREuPly5dSte7ubuzdu1egNDU1xYsXLwQ9SWz2boTqzJkzYhPBwcEifQ5uvHPnTuGD5p7wfW0olUoUFxfj9OnT8PLyErgXG6QRISf0RkZGeP36taxVREZGqletWiWyp+pmZmakQiQzF\/GeC+vq6oRvPDXnEuLp6Wns27cPRUVFAhUDkpslJSUCP\/lDi7l8+bJwcHbFmGhXVxd8fX3lwKyYk5MTHj16JCJQKJVKNf2IUHV0dGgPRu8aHx+Ho6MjbGxsBFITExPxJC7kxuQklUc4KAZ6Hgeh6OvrE2gOHz4sHCOUtKPKykqtFRFmCokcIxLOzs4iCBqxQEm1UXm2trYykb\/d3d0xOjoqQVg9KpaqMjAwEK4RLvLxwYMHWLlyJT5+\/Cjq4mGYOO\/Xr1+PgoICREVFYWxsTKtKeiCLUVVVJc+4bmJiQgT3\/v17yWEBxzZt2iTE5AIOVomQcHAjuvq3BqtDaEmJiIgIsQ\/GnD9Ifiah6QqkAA\/BofPXxdOnT0GbWLNmjZS+oaFB26ookOjoaNy+fVvEwt9lZWW4du2aWFB7ezuqq6u1gprfzmYnrlNi5Af7amxsrHCCfkZDXLdunfwmJfiekLBifEYbon\/xMKRDYmKieBXbGS8K7GtDp8RIcgYNCwsTz7t06RIeP34svKPqmASJfvToUeEMeXXw4EFs3bpVejDX0vV5TxGxK1CJP52YJgAVevHiRQlOE2VjJ6940Tro4kyKHYFGrakQOcfkaRGZmZly\/ZLEqCzC5uLiIjDRVu7du4cdO3aIx\/2SxC5cuIAbN25IkWiShCQlJUWqw2Fvby\/9lb2T0NGxeXpWisqlAf9vUFJtJC8PwR5HU2YyNF4+J\/n9\/f0FuiNHjggXKZwlJ\/98jrGSFAQNk8mxasuXL5eex++2\/8wu5ifGivF\/gObTnOTnoFWwgbNFzaYE4X\/27BlUKpV422JW8UMG+y3XX6r3OvnYUm36PXH0iX1PlX64V+oa\/Gfm\/7ZQ\/gMhRXU6lq2gLQAAAABJRU5ErkJggg==","height":30,"width":30}}
%---
%[output:78405e0b]
%   data: {"dataType":"matrix","outputData":{"columns":5,"name":"rules","rows":13,"type":"double","value":[["1","4","3","1","1"],["1","3","4","1","1"],["1","2","4","1","1"],["1","1","5","1","1"],["2","1","5","1","1"],["2","2","4","1","1"],["2","3","3","1","1"],["2","4","3","1","1"],["3","1","4","1","1"],["3","2","3","1","1"]]}}
%---
%[output:7182b99a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAACYAAAAmCAYAAACoPemuAAAAAXNSR0IArs4c6QAAA4VJREFUWEftWEsodGEYfmahWCkbkyyEbJGFZCMSEU0uKZeyEWWDcimXhcvEECthQUqjXJKssLCzQIoSFi4JK2YWFlg5f89b5zTnGMz3j\/n7F+ctzfmc93znmed93uf9cGiapuE\/DIcNTLEqNmOKhMFmzGZMlQHVfFtjyoy9vr5qMTEx8pzf70dZWRmen59lzWk1MjKC6upqWdfU1ODk5ESuMzIysLKyovq+kPMdgcA2NjbgdrvlhQQ3MTGBubk5xMXFwev1YmhoCP39\/bK5fl1XVxfyy1QSTcDa29txeXkJAmT09vYKS9nZ2fJ5cXGB7e1tJCQkIDc3F4mJiRFjzQD29vaGhoYGxMbGYn5+HlwTWE5OjpSSQBj7+\/vyaV2rsBFKrglYRUUFrq+v4XA45Nnk5GTU19eD5bIyRAYfHh4MoKG8TCXHAEbhE1hmZiampqaEMa5ZNjL4E7C7uzvMzMyguLgYeXl5KhiC5poYCywdgbW0tOD+\/h7r6+vSrV+VkhI4PDyU3OXlZbhcrt8Dxp3Gx8eRlJQkmtKB0TLYmY2NjabSWRkkOMbS0lLYoLiBqZS1tbWiL1rD8fExOjs70dbWJqD+uV34fD6NPsWg8EtLS8VYGR8fH+JrusGmp6fj\/f1d7kVHR+P09PRX2Am2ySeDpdOvrq6GbbBsJlbg9vbW+KJOpxMLCwtITU0VLN3d3djc3JRr6nJsbMzAGDGDpV5p1vHx8djZ2QHH3tbWlkwRxt7eHjo6OjA5OSlr\/To\/P19GY0QMlhs3NzeLFM7OzlBZWYnd3V2ZEikpKQZbBwcHMmUItqqqSu6RNYL+ZLA3NzcGnX9rsFdXV+jq6kJUVBRaW1tFFoODg+KHWVlZsj+BMGhF1vXi4qK5K8Mx2EABExiH\/MvLC87Pzw2NNTU1SafrQHSGdL2x+QhUgKWlpWnsxL6+PgwPDxuzkT7Gcjw+PmJtbe1bg7V2FYENDAzIsx6PRwCSQeqMU4WHgsDSBQUWeOyxGiyB0ddmZ2d\/NFgrYz09PRgdHZUOpI5YyoKCArAzyZpSKTms6WEcK+EYLMVPK+CPDox+WFRUJAdM+iLv6aWzltYkft7khuXl5Xh6ehICSkpKhHo9VE6wPNsdHR2JRVBvPC7RlHUf+9EuIvVPlWAGOz09jcLCQuOLfmuwkQIWbMyo\/M7+802FLebajNmMqTKgmv8HS6P1NZLV0TwAAAAASUVORK5CYII=","height":30,"width":30}}
%---
%[output:62f3885a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAACYAAAAmCAYAAACoPemuAAAAAXNSR0IArs4c6QAABfFJREFUWEftl3lsjGsYxc8oFUvFnnKjCFFCpAhSS4gttiKERAmNiCW21r5H7MaWKBGUWINKLG0aYt9FKaWlC0EqiLh2ouWPufk9N99kyvSmcyO59495k0mm873Lec85z3m+ujwej0f\/w+EKAgtQlSBjARKmIGNBxgJlIND5QY\/9FsYOHDigffv2aerUqZo\/f74OHz6syMjIYntfu3ZNkyZN8vusJBCBrHElJyd7Fi5caHstX75cQ4cOlT9gERERWrBggdLS0kR75TNu3DjNmjWr1GQEBGzIkCGe1atX2+Zz584V32\/evPkLY7Vq1dKoUaOUl5enMmXKaNCgQVqzZk2pQTExIGAJCQmeFStWqEKFClq7dq0aNGig79+\/G7CuXbtq3rx53sPr1q2rtm3bKj09Xa9evbLfmzZtquPHj5vUw4YN05EjR7zzt23bpvPnz+vUqVO2\/4YNG+wyWGTChAn68eOHzQ0PD9fFixe9duECLrfb7XHkABgDACNHjtTAgQP15MkTO8ztdmvXrl0GnOfVqlVTdna2Jk+ebHJu3bpVEydOtPWLFy829hMSEgQ4BhdCje7du6tSpUr2OXr0qGrWrKlevXrZnNOnTxs4A4bH8BUDAM+ePbODATlnzhy7IeZftmyZkpOTVa5cOdWpU0chISH69OmTsZGYmKjUtBRFRRTqz6I\/FBIapkuXLqmgoECdOnXSt2\/f1Lx5c8XHxys2NlaNGjVS\/fr1bb+KFStqypQpun\/\/vlq3bq2NGzfqzp07JQP7uSqnT5+uCxcu2M3btGmjGzdu6MGDB7py5YoVQXZmhuJiI5WX5VJ6foHOnDmj6tWr29zKlSurWbNm6tOnj5YuXWog+\/btq0WLFhlomH306JF599ChQ8rNzS1Zyp+BISNycVCrVq309OlTO\/zYsWO6d++exUpMdJRmuxPVs2dPkx4GkKlly5b2W35+vvnxy5cvfoumR48eJQPzNb9vjgF0y5YtioqKMmBQT\/WeO3dOd+\/eFZHjxE3nzp3N0F26dNGqVas0ZswYdejQwWIIievVq2c+qlGjhpcxkCIjRWIeK01c7Nmzx2hnU2IDA1+\/fl0vX760qsNjSBsdHW3fMfmOHTusQPAZUo0YMcLiBnabNGlilwKEIyW+glnWez3mL2B3796tsLAwq7zU1FS9efPGPFJYWKgqVarYoWXLltXYsWOLVeX+\/fvVuHFjk+7z588mGb\/17t1b\/fr1M0CENescjw0fPlwnTpww9rt16\/Y3Y\/7+GaFysrKyVLVqVd26dcuqEGO+ffvWqrGoqEhfv361Q7n14MGDLdvIJfKN5wyXy2UVTmY5kjsMbd682btHaGioypcvLzyWlJSknJycX4EREzzEX5mZmRYXVJeTcWQWRkdWJMZr5BYsINn69estRmCIXoq8rJ8xY4b69+9v+z5+\/NikZowePdq8uHLlymJneBl79+6dxo8fbywxkOP169eiK3BLns2cOVPt27e3Ijh48KDlEIYmKD98+KCOHTva2oyMDE2bNk3Pnz83G8AgVU1Fc9GTJ0\/aGsaAAQMUExNjmQlgnvMpBuz9+\/dGPc2aICWVYYgFTh8FsMMYqc7Nb9++rYYNG9pFzp49640CLkNboxh27txpwGAVe6SkpAgyfBnzPecXj8EOwD5+\/KirV6\/aIbxJ1K5d2xgCOCGJp5zqo7EToJQ7oPET7SkuLs6ybsmSJdaaWrRoYc+Rf+\/evQaMbtCuXTuzA3\/DHB+\/wGDnxYsXJhuMUZkYmLBkg\/8MGIxhWqeHQjHNmnjgpuvWrTPz+0rJWwaVhuHxGG2K73iJi\/w2KX2B+RbGpk2bzMgcSNPnQMxMD0UyPERAwjoxQgAjH0VC88bYvOKwHttweeZxCb\/mdxzLZOR7+PChtRPkBBiVAysAKW1cIDt7XL582drSv4oLX2BISXvgNRqjMmgphC5\/84wU51AGrza8ZRCoDCTdvn27dy7sETW0MNZRncjL4EXUOYd9sQe5F\/z3ze97zj\/8GGQsyFigDAQ6\/y9ofGl2JZLYoQAAAABJRU5ErkJggg==","height":30,"width":30}}
%---
%[output:43e43514]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAACYAAAAmCAYAAACoPemuAAAAAXNSR0IArs4c6QAABH5JREFUWEftV2sopVsYfnaNolEu4yiTlEhIShMS5Zgfck+YNEluucVxqZHbzDQ5M4aQk1Buk0siJBF+yLj\/cE1IlPjhUjqFUTb\/9ul569ttzXTau2k4nfaqnb32t771Put5n\/d5F5VGo9HgPzhURmAGZsXImIGEwciYkTFDGTB0vVFjv5SxwcFBvH37VmJ8\/PgRr169MjSe3uvvpfLi4gKZmZnY2trSbqAAODg4QEZGBk5OTuSZvb09Wltb4ezsrHcwQxaq1Gq1xszMTN5hcAI7Pj6GSqVCd3c3fH195VlTU5MAub29lbm5uTnS0tKQk5NjSDy9194D1tnZiebmZoyMjAi42tpatLS0wNraGq9fv8b29jb6+\/thamqK2NhYuLu7o6+vT+9ghiy8B4wMfPv2DUNDQ8JMeXk54uPjhbWXL1\/i7u4O09PTIMNBQUHC6tevXw2Jp\/daLTACiYmJweHhofZlOzs7pKenIyEhAf7+\/rCwsMDExIQ8DwsLk0MsLS3JPK7gT6ifuWDiXbzewf9toRYYhU9gXl5eqK+vF8Y4f\/78OTo6OgSYpaUlxsfHZb\/w8HBcXV0JsMTERExe\/SbAwiz\/xtBf734anMrFxUXDILQBVqCSOu5cWFgoFUqboKZ0U8fU8vI7MzMjIAju9pkLcv\/Ixe9OVj8PTKlKRVN+fn7iT5yzQtVqNXp6epCSkoK9vT3Rn6QuLg6urq5SDL9i3EtlVFQUIiMjUVxcjLOzM0RHRwtTnNMumNKbmxvB8fTp04ezi6mpKRQVFWm9ysbGBmNjY2IX9DgWAgFzUHttbW0PY7AMWFNTg\/b2dgnu6OiIo6Mj+U49WVlZieA5QkNDMTk5Kd8DAgKwsLAgGuRobGwUM6Y+PT09UVJSgjdv3sihqOcXL16goqJC1paWlkpBKWsV3\/zudqEAy8vLw+bmJqqrq4Ux\/s5BRsvKyjA3N4euri7Z8P3793BwcJBiqaurw\/X1NUJCQuT3z58\/Y35+HqmpqYiIiAD3JftkmyMpKQmBgYGorKy8F0MLTOmTtAQO9sDz83N8+vRJWwg8Nc2WeqPjDwwMoLe3VzoFC4FMEdzy8jLy8\/ORnJwsBszDkEEyxYOS6eHhYYmjq2sC5nN+7gG7vLwU7dDxTUxMQI1xU77AdFRVVQlghTHSzpOvra1pgbF3rq+vS8PndwJg2lg4BEbgq6urGB0dBcnQZUw3znepVGyDrr64uKjVl62trTBE4Ay0srIiQAlsf38fbm5uYswEzbaVnZ0tjLGgPnz4ID3Xw8NDnjP9vCAQWEFBAXx8fCTtnNMB+PkhMLJzenoqaSNjOzs7yMrKQnBwsGzwaMCYSsVoSRkpzs3NxZMnT+SkvHVQ\/I+SSl1guoXR0NCAL1++iHbYqqgditnb21tSRg1tbGyIJtlrKX6mj0Xi5OQkwp6dnZX3KRt2Ga6jHn8ofqWtcDHTt7u7K6XOdBIYOwIrj0Ae1C50gTGVNEbF+PiMVx9eFBUjpFES+IMZrALwsf8a\/680NANGxv43jP0Dor93oKWZKsMAAAAASUVORK5CYII=","height":30,"width":30}}
%---
%[output:0663df05]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAACYAAAAmCAYAAACoPemuAAAAAXNSR0IArs4c6QAABYJJREFUWEftV1tIVVsUHbss85WZYah5TmSpFaJm3HyUmQoFYfkhEkZaRAXiIyvQ8EmJkX5EEKhJBYUfJaGVmuIlvVZgWB9qfYhKSinmKxM13+cyJuxDevVyTpTcC0447H32XmetMcccc6x1FJ1Op8N\/MJRlYEZWZZkxIwnDMmPLjBnLgLHj\/z8aKygoQGpqKrKysnDu3Dm8ePECwcHBkjC\/37hxA2ZmZgYRMDAwgOPHj6Oqqgru7u7YsWMHdu7cibS0NLx+\/Vrm5r0a6nh\/f\/9\/duWPwKKiohAXF4empiaEhobOmcQQZEVFRXjw4AF4NTc3R2JiIhwdHXHp0iX9\/aLAgoKCdJ8\/f5Z1Nm3ahIiICD1j4eHhOHbsGGZnZ7Fnzx7U1taCWTG4xebn5yMoKGhRjFevXkVXV5ewzDAKGNfYsGED\/Pz88PTpU0OIWHBMbm4uvn\/\/jvT09AXf29raYs2aNVLSt2\/for+\/f8FxW7duRWBgIBSNRqP7+vUrJiYmZODatWsxMzMDPtu7dy9evXqln8DCwgI+Pj6ijZUrV0p57t69C5Z8bGwMu3fvRmVlJfbv348PHz6AYLZs2YLS0lJJnJWxt7dHTEwMrl+\/jm\/fvsl3a2trBAQEoLi4WNZlKK6urrrVq1djenoaHz9+xPr16zE5OQlfX1+sWrUKZWVlICAGn7Os27dvx\/v376EoCurq6pCZmYnnz5\/D29sbDx8+hKenJ7RarSRLIefl5YmuCgsLYWNjg8jISNy+fVvkwPnZCJ2dnTKeVRseHobi7e2t44LsmIaGBll8fHxcupIAQkJCJKMVK1ZgZGREEmCW1Bq\/zw8nrSN6e\/plEVNTU+loNsDFixdRXV0tDJLtN2\/eyFqsjtrxXl5eyMjIENYUaszOzg4uLi7o7u6eA8zJyQmHDx8GNcgJCIRjeWUZGJcvX8a7d+\/wZ3U1HNwsMNQ1i5HhMXnHZMgww9LSEuvWrcPo6Ki+XAuJjMDJpGJra6tj9hRlb2+vlG8+Y9Sdmh0BOjg4yORDQ0O4efMmDhw4AGZrr7FByB9HUPlXBe7cuYMTJ07IXBcuXMC9e\/fw5csX0SXXam5uFimQuUePHkn3Zmdny1iWVtm2bZtucHBQsuNCGo1G2Dh16pSItqOjQ0rCYHkIjAZLoFNTU2ItlAF14+bmJqDJCpmmNJ49eyZN4eHhIWD5nqZLv+Tv2QTl5eXYvHkzrKyscP\/+fSFI0Wq1ur6+PukqExMTuLq6oqenBxs3bpSuevnypYBiGWgHLC\/BMhGCo9iZJTNm0Nm5QH19vQBubW2Fs7Oz6KqlpUXmoebIJgkgg7wePXoUT548kTn4TDTG8vEHBPhjEBiz4kCCVYPj+VwNehcTu3XrlpRuMe3s27cPnz59ErtZLGjYdIZ\/3cTJEN2aE5J++gxLx+DWQpbUP1lkUL2\/du0arly5IgzzGT8pKSk4efLkooDmv9AD4\/ZB0XFfI1MM3tPRWS6WmMHFuIjaJNRcTU2N6I9bGt\/Rs2gx9COWnleW6\/Hjx1JWQ0IPTGWHolTj4MGDc4DOn5AdRXaoHQZ1mZycLCyzkwmKLk9LocbOnj1rMGs\/fR5jJ9OEabhkjKWkq0dHRyMnJwfcTQ4dOgQmR6DcJ7kHsgsNiZ8GRr2xrRsbG6VzuTDLdPr0aSQlJQlDCQkJen2yU3ft2vV7gbW1tYlhki1aB89rjNjYWNnGzp8\/vzTA2N5cVN3XeDbjyaOkpETcu729XYyZGuLWQ19b8lJSW2fOnJHTxfygLYSFhaGiokJ8kScLamzJxE8LITCWkacBCprmHB8fL6Ukc7\/MLgzpFI4hY\/xToupKNVU5ESiKfH6pwRoKbKnG\/bRd\/G6Ay8CMZXiZMWMZ+xu3mnuZtKqMCgAAAABJRU5ErkJggg==","height":30,"width":30}}
%---
%[output:692e32ac]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAACYAAAAmCAYAAACoPemuAAAAAXNSR0IArs4c6QAAA6ZJREFUWEftV0sobHEc\/kYJIZNJU0LGOyI2psmKhalxF55J3nmWUqNQuI2FWFDugoXnwrvMlo1QyoKdx0Kh1JXidj0WZpQyt9+vzumcqZtzhrnu4vxrFv85v\/n\/v\/m+3+M7Oo\/H48F\/uHQaMJWqaIypJAwaYxpjahlQG6\/lmGrGXC6XJyQkRPzd6OgoZmdneb+wsACz2Sw+W19fx8DAAO+HhoZQUVGh9j7F8TopsK2tLXR3d8PlcokH9PX1oaGhARcXF2htbcX19TU\/i4mJwfT0NJKSkhRfpiZQBoxAHBwcgJghFvv7+1FZWcmsTU5OMhC3283nh4WFoampCR0dHWruUxwrAqML29vb8fz8jMXFRT6AgFksFpasqqoKJycnWFtbQ3BwMMrKypCeno7V1VXFl6kJlAGrra1FbGwsxsfH+QzKt\/j4eAZWUFCAl5cXbG9vM5v5+fnQ6XTY2dlRc5\/iWBmw0tJSXF5e8oW0EhISUFNTg+rqauTl5SEiIgKbm5v8zGaz4enpCfv7+7ynP\/U75RvMublwWE2KAfwtUAR2f38PApaYmIi5uTl47wmYXq\/HxsYGn1VUVITHx0cGRqAODw\/x02KHTf8Lzh\/fPw+Yt3RCzt3e3mJlZYVzSiodSUvmd3d3V2TMbUj5FFB0oKwqqSLHxsYwNTWF8\/NzDA4Ospz0XWNjI87OzuB0OhlIeXk50tLSuBj8sWRStrW1cX8S8shkMnFLoOSndkESU9XSCg0N9W+7kHp+yisCd3x8zJc3Nzdzw6VFDbalpQU3Nze8j46OxszMjP8arJqXEelIyszM5L4mrLe3NwQEBPB2YmKCmzH9QeEljHqjdLy9J79P7oKY7e3tRVRUFHJychAXFyfLza6uLmRnZ+Pu7g5ZWVlc4TTKKHcjIyPfw8TPfQJGsg4PD3Ojraurw97eHh9GspPUxcXFSE1NZVAlJSWyWKWs+QSMqpekoWFP7MzPz4uj6\/T0lCvYaDSivr6eGe3p6YHBYEBhYaFiR+ITMGmuEVOUR\/RZWlpiAORCwsPDQaYgOTlZJrtSq+QTMJqhxMzV1RW3EZIwKCgIDw8PcDgc7NX+CTDvViE1kDQhOjs7uZ2QN8vIyMDy8vLXSOldViTZ0dEROxEa7iMjIzwVrFYr55i0UPyW\/MQQ+bTAwEC8vr6y1bbb7SwtWSYyjjTKvqRdCOAEp0HNlRytYMm\/rMEq6pAfDPKpKj94p6Kfa8AU0SQJ0hhTy9gfzWVFrbg5hhsAAAAASUVORK5CYII=","height":30,"width":30}}
%---
