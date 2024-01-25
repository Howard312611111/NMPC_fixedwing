function xk1 = fixedwing_DT(xk, uk, Ts)
%% Discrete-time nonlinear dynamic model of a pendulum on a cart at time k
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
%
% xk1 is the states at time k+1.
%
% Copyright 2018 The MathWorks, Inc.

%#codegen

% Repeat application of Euler method sampled at Ts/M.
M = 10;
delta = Ts/M;
xk1 = xk;
for ct=1:M
    xk1 = xk1 + delta*fixedwing_CT(xk1,uk);
end
% Note that we choose the Euler method (first oder Runge-Kutta method)
% because it is more efficient for plant with non-stiff ODEs.  You can
% choose other ODE solvers such as ode23, ode45 for better accuracy or
% ode15s and ode23s for stiff ODEs.  Those solvers are available from
% MATLAB.