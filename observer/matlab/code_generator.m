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

vars_model_real_input = {real_state, Fx, Mz};  
vars_model_obs_input = {obs_state, Fx, Mz};  

%% 3. NUMERIC FUNCTION EXPORT
fprintf('--> Exporting symbolic matrices to numeric MATLAB function...\n');
just_six_states = 1;
 
matlabFunction(dot_x_real, J_x_real, h_real, J_h_real, 'File', 'symbolic_matlab_function_generated/vehicle_dynamics_real', ...
    'Vars', vars_model_real_input, ...  
    'Outputs', {'dot_x_num', 'J_x_num', 'h_num', 'J_h_num'});
 
matlabFunction(dot_x_real, J_x_real, h_real, J_h_real, 'File', 'symbolic_matlab_function_generated/vehicle_dynamics_obs', ...
    'Vars', vars_model_obs_input, ...  
    'Outputs', {'dot_x_num', 'J_x_num', 'h_num', 'J_h_num'});

fprintf('--> Generated localized 6-DOF vehicle_dynamics_numeric.m successfully!\n');

%% 4. AUTOMATED CODEGEN PIPELINE
fprintf('--> Running MATLAB Code Generation (codegen)...\n');
cfg = coder.config('lib');
cfg.SupportNonFinite = false; 
cfg.GenerateExampleMain = 'DoNotGenerate';
cfg.TargetLangStandard = 'C99 (ISO)';       
cfg.TargetLang = 'C';

addpath("symbolic_matlab_function_generated");

codegen symbolic_matlab_function_generated/vehicle_dynamics_real -args {zeros(6, 1), 0.0, 0.0} -config cfg -report
codegen symbolic_matlab_function_generated/vehicle_dynamics_obs -args {zeros(6, 1), 0.0, 0.0} -config cfg -report


target_files = {'vehicle_dynamics_real', 'vehicle_dynamics_obs'};

for i = 1:length(target_files)
    current_target = target_files{i};
    target_dir = sprintf('codegen/lib/%s', current_target);
    
    fprintf('\nProcessing and compiling target: %s...\n', current_target);
    
    % 1. Commenta l''inclusione di tmwtypes.h
    comment_cmd = sprintf('cd %s && sed -i ''s/#include "tmwtypes.h"/\\/\\* #include "tmwtypes.h" \\*\\//g'' rtwtypes.h', target_dir);
    
    % 2. Inietta la definizione di boolean_T mancante su Linux
    typedef_cmd = sprintf('cd %s && sed -i ''/tmwtypes.h/a typedef unsigned char boolean_T;'' rtwtypes.h', target_dir);
    
    % 3. Comando GCC dinamico (FIXED: Compila il file C specifico e genera il rispettivo .so unico)
    gcc_cmd = sprintf('cd %s && gcc -O3 -fPIC -shared -I. %s.c -o lib%s.so -lm', target_dir, current_target, current_target);
    cleanup_cmd = sprintf('find %s -type f ! -name "*.so" -delete && find %s -type d -empty -delete', target_dir, target_dir);
    % Esecuzione sequenziale dei comandi di sistema per il target corrente
    system(comment_cmd);
    system(typedef_cmd);
    [status, cmdout] = system(gcc_cmd);
    
    if status == 0
        fprintf('SUCCESS: Compiled lib%s.so inside %s/!\n', current_target, target_dir);
        system(cleanup_cmd);
    else
        error('Compilation failed for %s. Compiler output:\n%s', current_target, cmdout);
    end
end

 