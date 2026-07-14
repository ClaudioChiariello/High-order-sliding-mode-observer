function [observer_state, dot_x, J_x, h, J_h] = compute_truck_observed_state(p, s)
   
    % Definizione del vettore di stato dell'osservatore (usando direttamente la struttura)
    observer_state = [s.phi; s.wx; s.vy; s.wz; s.vx; s.phi_u]; 
    
    %% 2. Calcolo Cinematico e Angoli di Slip
    delta_1 = (p.sum_L / 2) * (s.wz / s.vx_lim);
    delta_2 = (2*p.L(2) + p.L(3) + p.L(4)) / (2*p.L(1) + p.L(3) + p.L(4)) * delta_1;
    
    alpha = sym(zeros(4, 1));
    alpha(1) = delta_1 - (s.vy + p.L(1) * s.wz) / s.vx_lim;
    alpha(2) = delta_2 - (s.vy + p.L(2) * s.wz) / s.vx_lim;
    alpha(3) = - (s.vy - p.L(3) * s.wz) / s.vx_lim;
    alpha(4) = - (s.vy - p.L(4) * s.wz) / s.vx_lim;
    
    %% 3. Equazioni delle Forze e dei Momenti sugli Pneumatici
    F_sat = p.C_alpha * alpha;
    F_lateral_total = F_sat(1) * cos(delta_1) + F_sat(2) * cos(delta_2) + F_sat(3) + F_sat(4);
    tire_yaw_moment = (F_sat(1) * cos(delta_1) * p.L(1) + F_sat(2) * cos(delta_2) * p.L(2) - F_sat(3) * p.L(3) - F_sat(4) * p.L(4));
     
    
    %% 4. Derivate di Stato (Dinamica di Sistema)
    dvx    = s.Fx / p.m + s.vy * s.wz;
    dvy    = -s.vx * s.wz + (F_lateral_total) / p.m;
    dot_wz = (s.Mz + tire_yaw_moment) / p.Iz;
    dphi   = s.wx;
    dphi_u = 0;
    dot_wx = (p.m * (-s.vx * s.wz + (F_lateral_total) / p.m) * p.h_com + p.m * p.g * sin(s.phi - s.phi_u) - p.Cr *(dphi-dphi_u)  - p.Kr * (s.phi - s.phi_u)) / p.Ix;

    % Vettore finale delle derivate
    dot_x = [dphi; dot_wx; dvy; dot_wz; dvx; dphi_u];
    
    %% 5. Modello di Uscita / Matrice di Misura h(x)

    distance_from_CoM = [p.distance_com; p.distance_com; p.distance_com];
    dot_omega = [dot_wx; 0; dot_wz];
    omega     = [s.wx; 0; s.wz];
    
    % Calcolo del termine di trasporto (accelerazione relativa dell'IMU rispetto al CoM)
    transport_term = cross(dot_omega, distance_from_CoM) + cross(omega, cross(omega, distance_from_CoM));
    
    % Isoliamo l'accelerazione lungo l'asse Y (indice 2) e aggiungiamo il termine centripeto
    acc_y_measured = dvy - transport_term(2); 
    acc_y_measured = acc_y_measured + s.vx * s.wz; 
    
    % Vettore delle uscite h(x)
    h = [s.phi; acc_y_measured; s.wx; dot_wx; s.wz; s.vx];
    
    %% 6. Calcolo Jacobiani Analitici rispetto a observer_state
    J_x = jacobian(dot_x, observer_state);
    J_h = jacobian(h, observer_state);
end