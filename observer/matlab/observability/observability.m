clear; clc;


T = readtable('csv_for_observability_matrix/simulation_data_curve.csv');
data = table2array(T);

% Check for NaN values
nan_mask = isnan(data);

if any(nan_mask, 'all')
    fprintf('WARNING: NaN values found in the CSV file!\n\n');

    % Total number of NaNs
    fprintf('Total NaN values: %d\n\n', sum(nan_mask, 'all'));

    % Find rows and columns containing NaNs
    [rows, cols] = find(nan_mask);

    fprintf('NaN locations:\n');

    for k = 1:length(rows)
        fprintf('Row %d, Column %s\n', ...
            rows(k), T.Properties.VariableNames{cols(k)});
    end

else
    fprintf('No NaN values found in the CSV file.\n');
end

 
% Extract signals as vectors
time = T.time;
vy_real = T.state_real_VY;
vx_real = T.state_real_VX;
phi_real = T.state_real_PHI;
wx_real = T.state_real_WX;
wz_real = T.state_real_WZ;
Mz_real = T.Mz;
phi_u_real = T.state_real_PHI_U;


%% 2. Physical Parameters (Numeric Constants)
params.m = 12809.162180730276; 
params.Ix = 4423.849628639999; 
params.Iz = 35308.58917090647;
params.C_alpha = 90000;
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

numbers = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
%% 4. Symbolic Expressions
% Uses the symbolic vx from the struct
%syms.vx_lim = piecewise(abs(syms.vx) < params.velocity_limit, piecewise(syms.vx >= 0, params.velocity_limit, -params.velocity_limit), syms.vx);
syms.vx_lim  = syms.vx;

Fx = syms.Fx;
Mz = syms.Mz;

[real_state, dot_x_real, J_x_real, h_real, J_h_real] = model_for_obs(params, syms);

% f(x)
f = dot_x_real([1:end-2, end]);

% h remove the vx from the output
h = h_real(1:end-1); 

% Jacobian of f (constant throughout the recursion)
Jf = J_x_real;

maxOrder = 6;
% 2. Preallocate a symbolic matrix of size 6 x maxOrder
% Devi avere n-p+1 Lie derivative (inclusa la 0 lie derivative)

% Jacobian of current vector field h
J_h = jacobian(h, real_state);

% Lie derivative L_f(h)
Lie_derivative = J_h*f;

PHI = [h; Lie_derivative];
 
dPhi = jacobian(PHI, real_state);

N = length(vx_real);
rank_vector = zeros(N,1);

dPhi_func = matlabFunction(dPhi, 'Vars', ...
    {syms.Mz, syms.phi_u, syms.vx, syms.vy, syms.phi, syms.wx, syms.wz});

for k = 1:N
    % Evaluate matrix numerically at time step k
    M_k = dPhi_func(Mz_real(k), phi_u_real(k), vx_real(k), ...
                    vy_real(k), phi_real(k), wx_real(k), ...
                    wz_real(k) ...
                    );

    % Compute numerical rank
    rank_vector(k) = rank(M_k);
end

Gramian = dPhi' * dPhi;

rank(Gramian)
%obs_expression = simplify(det(Gramian));
%unobservable_states = solve(obs_expression == 0, real_state);