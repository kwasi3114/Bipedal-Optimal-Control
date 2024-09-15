function accel_ref_x  = LQR_Bipedal_X(Xref, x)
    % Define state space matrices
    A = [0, 1; 0, 0];
    B = [0; 1];
    C = [1, 0];
    D = 0;

    % Define Q and R matrices for LQR
    Q = diag([1, 1]);  % Weighting matrix for states [x, xdot]
    R = 1;             % Weighting matrix for input u

    % Solve the Riccati equation to find the optimal state feedback gain matrix K
    %P = dare(A, B, Q, R);
    P = [1, 0; 0, 2];
    K = -inv(R)*(B')*P;

    % Calculate the error between the reference state and the current state
    error = Xref - x;

    % Calculate the control input (reference acceleration)
    u = -K * error;
    accel_ref_x = u(2);
end
