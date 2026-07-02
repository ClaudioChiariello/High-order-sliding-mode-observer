function [dot_x_num, J_x_num, h_num, J_h_num] = generate_python(x_test_plant, x_test_obs, Fx_test, Mz_test)
% Use this to generatea pythonPackage
% compiler.build.pythonPackage('generate_python.m', 'PackageName', 'my_vehicle_package')    

% 1. Inizializzazione delle variabili simboliche
    syms x y phi vx vy wx wz real
    syms Fx Mz real
    syms phi_u real
    psi = sym('psi', 'real'); 
    state_plant = [x; y; phi; psi; vx; vy; wx; wz];
    state_obs   = [phi; wx; vy; wz; vx; phi_u]; 

    % Parametri Fisici
    m = 1500; Ix = 2500; Iz = 3500; C_alpha = 300e2;
    Cr = 200000.0; Kr = 800000; g = -9.81; h_com = 1;

    % 2. Calcolo della cinematica
    dx = vx * cos(psi) - vy * sin(psi); 
    dy = vx * sin(psi) + vy * cos(psi);
    dphi = wx;
    dpsi = wz;   
    
    L = [3.65, 1.75, 2.0, 3.39];
    sum_L = sum(L);
    delta_1 = (sum_L / 2) * (wz / vx);
    delta_2 = (2*L(2) + L(3) + L(4)) / (2*L(1) + L(3) + L(4)) * delta_1;
    
    alpha = sym(zeros(4, 1));
    alpha(1) = delta_1 - (vy + L(1) * wz) / vx;
    alpha(2) = delta_2 - (vy + L(2) * wz) / vx;
    alpha(3) = - (vy - L(3) * wz) / vx;
    alpha(4) = - (vy - L(4) * wz) / vx;
    F = C_alpha * alpha;

    F_lateral_total = F(1) * cos(delta_1) + F(2) * cos(delta_2) + F(3) + F(4);
    tire_yaw_moment = (F(1) * cos(delta_1) * L(1) + F(2) * cos(delta_2) * L(2) - F(3) * L(3) - F(4) * L(4));

    dvx = Fx / m + vy * wz;
    dvy = -vx * wz + (F_lateral_total) / m;
    dot_wx = (m * dvy * h_com + m * g * sin(phi) - Cr * wx - Kr * phi) / Ix;
    dot_wz = (Mz + tire_yaw_moment) / Iz;

    dot_x = [dx; dy; dphi; dpsi; dvx; dvy; dot_wx; dot_wz];

    % 3. Modellazione h(x)
    distance_from_CoM = [0.5; 0.5; 0.5];
    dot_omega = [dot_wx; 0; dot_wz];
    omega = [wx; 0; wz];
    transport_term = cross(dot_omega, distance_from_CoM) + cross(omega, cross(omega, distance_from_CoM));
    acc_y_measured = dvy - transport_term(2); 

    h = [phi; acc_y_measured; wx; wz; dvx; dot_wx];

    % 4. Calcolo delle Matrici Jacobiane Simboliche
    J_x = jacobian(dot_x, state_plant);
    J_h = jacobian(h, state_obs);

    % 5. Sostituzione Numerica (Substitute input values directly)
    % This evaluates the symbolic expressions into numeric matrices before returning to Python
    dot_x_num = double(subs(dot_x, [state_plant; Fx; Mz], [x_test_plant; Fx_test; Mz_test]));
    J_x_num   = double(subs(J_x,   [state_plant; Fx; Mz], [x_test_plant; Fx_test; Mz_test]));
    h_num     = double(subs(h,     [state_obs; Fx; Mz],   [x_test_obs; Fx_test; Mz_test]));
    J_h_num   = double(subs(J_h,   [state_obs; Fx; Mz],   [x_test_obs; Fx_test; Mz_test]));
end