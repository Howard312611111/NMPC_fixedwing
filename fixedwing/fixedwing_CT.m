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
g = 9.81;   % gravity of earth
J = [1 0 0; % moment inertia
     0 1 0;
     0 0 1];
%% Obtain x, u and y
% x
U = x(1)
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
F_x = 1; %force along body frame x axe, f(alhpa,q_hat,delta_e,delta_t)
F_y = 1; %force along body frame y axe, f(beta,p_hat,r_hat)
F_z = 1; %force along body frame z axe, f(alhpa,q_hat,delta_e,delta_t)
L = 1;   %roll moment, f(beta,p_hat,r_hat,delta_a)
M = 1;   %pitch moment, f(alpha,q_hat,delta_e)
N = 1;   %yaw moment, f(beta,p_hat,r_hat,delta_a)
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