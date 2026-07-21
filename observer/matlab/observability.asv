clear; clc;

%% 2. Physical Parameters (Numeric Constants)
params.m = 12809.162180730276; 
params.Ix = 4423.849628639999; 
params.Iz = 35308.58917090647;
params.C_alpha = 350e2;
params.Cr = 200000.0; 
params.Kr = 800000; 
params.g = -9.81;
params.CoM_z = 2.0832863727326365;
params.wheel_radius = 0.5645;
params.h_com = params.CoM_z - params.wheel_radius;
params.velocity_limit = 1.5;

params.L = [3.65, 1.75, 2.0, 3.39];
params.sum_L = sum(params.L);
params.distance_com = 0.0;

%% 3. Symbolic Variables Definition
syms.phi   = sym('phi', 'real');
syms.wx    = sym('wx', 'real');
syms.vy    = sym('vy', 'real');
syms.wz    = sym('wz', 'real');
syms.vx    = sym('vx', 'real');
syms.phi_u = sym('phi_u', 'real'); 
syms.Fx    = sym('Fx', 'real');
syms.Mz    = sym('Mz', 'real');

%% 4. Symbolic Expressions
% Uses the symbolic vx from the struct
syms.vx_lim = piecewise(abs(syms.vx) < params.velocity_limit, piecewise(syms.vx >= 0, params.velocity_limit, -params.velocity_limit), syms.vx);


Fx = syms.Fx;
Mz = syms.Mz;

[real_state, dot_x_real, J_x_real, h_real, J_h_real] = compute_truck_real_state(params, syms);
[obs_state, dot_x_obs, J_x_obs, h_obs, J_h_obs] = compute_truck_observed_state(params, syms);


% f(x)
f = dot_x_real;

% h(x)
g = h_real;

% Jacobian of f (constant throughout the recursion)
Jf = J_x_real;

maxOrder = 6;
% 2. Preallocate a symbolic matrix of size 6 x maxOrder
Lie = sym(zeros(6, maxOrder)); 

% 3. Run the loop
for k = 1:maxOrder
    % Jacobian of current vector field g
    Jg = jacobian(g, real_state);
    
    % Lie bracket [f,g]
    g = Jg*f - Jf*g;
    
    % Store the 6-element column vector in the k-th column
    Lie(:, k) = g; 
end


vector_36x1 = Lie(:);
