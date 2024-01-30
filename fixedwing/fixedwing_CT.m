function dxdt = fixedwing_CT(x, u)
%% Continuous-time nonlinear dynamic model of a pendulum on a cart
%
% 9 states (x): 
%   plane velocity (U,V,W)
%   plane anguler velocity (P,Q,R)
%   plane eular angle (phi,theta,psi) 
% 4 inputs: (u)
%   thrust (delta_t)
%   aileron angle (delta_a)
%   elevator angle (delta_e)
%   rudder angle (delta_r)
% Copyright 2018 The MathWorks, Inc.

%#codegen

%% parameters
m = 1;  % plane mass
rho = 1.225; %air density
b = 1; %wing span
c_bar = 1; %mean aerodynamic chord
S = 1; %wing area
g = 9.81;   % gravity of earth
J = [1 0 0; % moment inertia
     0 1 0;
     0 0 1];
%% Obtain x, u and y
% x
U = x(1);
V = x(2);
W = x(3);
P = x(4);
Q = x(5);
R = x(6);
phi = x(7);
theta = x(8);
psi = x(9);
% u
delta_t = u(1);
delta_a = u(2);
delta_e = u(3);
delta_r = u(4);
%% force and moment caculation
V_t = sqrt(U^2+V^2+W^2); %total velocity(assume no wind)
q_bar = rho*V_t^2/2;
alpha = atan2(W,U); %attack angle
beta = asin(V/V_t); %heading angle
C_L = alpha+beta; %lift coefficient, depend on alpha,beta
C_D = alpha+beta+delta_r+delta_e; %drag coefficient, depend on alpha,beta,delta_r,delta_e
C_C = alpha+beta+delta_r+delta_a; %shear coefficient, depend on alpha,beta,delta_r,delta_a
C_l = beta+delta_a+delta_r; %row coefficient, depend on beta,delta_a,delta_r
C_m = alpha+delta_e; %pitch coefficient, depend on alpha, delta_e
C_n = beta+delta_a+delta_r; %yaw coefficient, depend on beta,delta_a,delta_r
F_D = q_bar*S*C_D; %drag force
F_L = q_bar*S*C_L; %lift force
F_C = q_bar*S*C_C; %shear force
F_x = F_D+delta_t; %force along body frame x axe, f(alhpa,q_bar,delta_e,delta_t)
F_y = F_C; %force along body frame y axe, f(beta,p_hat,r_hat)
F_z = F_L+delta_t; %force along body frame z axe, f(alhpa,q_bar,delta_e,delta_t)
L = q_bar*S*b*C_l;   %rolling moment, f(beta,p_hat,r_hat,delta_a)
M = q_bar*S*c_bar*C_m;   %pitching moment, f(alpha,q_hat,delta_e)
N = q_bar*S*b*C_n;   %yawing moment, f(beta,p_hat,r_hat,delta_a)
Gamma = J(1,1)*J(3,3)-(J(1,3))^2;
%% Compute dxdt
dxdt = [R*V-Q*W-g*sin(theta)+F_x/m;...
        -R*U+P*W+g*sin(phi)*cos(theta)+F_y/m;...
        Q*U-P*V+g*cos(phi)*sin(theta)+F_z/m;...
        (J(1,3)*(J(1,1)-J(2,2)+J(3,3))*P*Q-(J(3,3)*(J(3,3)-J(2,2))+J(1,3)^2)*Q*R+J(3,3)*L+J(1,3)*N)/Gamma;
        ((J(3,3)-J(1,1))*P*R-J(1,3)*(P^2-R^2)+M)/J(2,2);
        (((J(1,1)-J(2,2))*J(1,1)+J(1,3)^2)*P*Q-J(1,3)*(J(1,1)-J(2,2)+J(3,3))*Q*R+J(1,3)*L+J(1,1)*N)/Gamma;
        P+tan(theta)*(Q*sin(phi)+R*cos(phi));
        Q*cos(phi)-R*sin(phi);
        (Q*sin(phi)+R*cos(phi))/cos(theta)];