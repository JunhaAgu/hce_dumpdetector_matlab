function [hidden_state_optimal, r] = runBA_huber(hidden_state, observations, K)

N = observations(1); % the number of image
M = observations(2); % the number of points

x_iter = hidden_state; % initial guess of state

lam_init = 1;
lam = lam_init;
iter_opt = 1;
iter_max = 500;
huber_parameter = 4;
residual_norm = zeros(iter_max, 1); % for debugging

while true

    % 1. Extract 3D position of landmarks and twist from x_iter
    twist_kw = reshape(x_iter(1:N*6), 6, []); % twist [6xN]
    Pts_world = reshape(x_iter(N*6+1:end), 3, []); % 3D landmarks [3xM]



    % 2. Compute Jacobian and residual
    r = zeros(2*((length(observations) - 2 - N) / 3), 1); % [2(k1+...+kn)x1]
    J = zeros(length(r), 6*N+3*M); % [2(k1+...+kn)x(6N+3M)]

    % Fill Jacobian and residual for each frame individually:
    observation_i = 3;
    error_i = 1;
    for iter_frame = 1:N
        ki = observations(observation_i); % ki: ith img 안에서 찍힌 keypoints 수

        T_kw = se3Exp(twist_kw(:, iter_frame)); % T [4x4] (camera frame -> world frame)
        pts_observation = reshape(observations(observation_i+1:observation_i+2*ki), 2, []); % keypoints in ith frame [2xki]
        idx_landmark = observations(observation_i+2*ki+1 : observation_i+3*ki); % index between keypoints and landmark [1xki]
       
        
        % Landmarks observed in ith frame.
        Pts_ki_camera = T_kw(1:3,1:3)*Pts_world(:, idx_landmark) + T_kw(1:3,4); % [3xki]

        % Transforming the observed landmarks into the camera frame for projection.
        pts_reprojection = K*Pts_ki_camera./Pts_ki_camera(3, :);
        pts_reprojection = pts_reprojection(1:2,:);


        % Compute residual
        reprojection_err = pts_observation - pts_reprojection;
        r(error_i:error_i+2*ki-1, 1) = reprojection_err(:);
        
        


        
        
        % Compute jacobian
        for iter_ki = 1:ki
            % Landmark
            Pts = Pts_ki_camera(:, iter_ki);

            % Each error is affected by its pose (2x6)
            J(error_i+2*(iter_ki-1):error_i+2*iter_ki-1, 6*(iter_frame-1)+1:6*iter_frame) = ...
                [K(1)/Pts(3), 0, -K(1)*Pts(1)/(Pts(3)^2), ...
                -K(1)*Pts(1)*Pts(2)/(Pts(3)^2), K(1)*(1+(Pts(1)/Pts(3))^2), -K(1)*Pts(2)/Pts(3);...
                0, K(5)/Pts(3), -K(5)*Pts(2)/(Pts(3)^2), ...
                -K(5)*(1+(Pts(2)/Pts(3))^2), K(5)*Pts(1)*Pts(2)/(Pts(3)^2), K(5)*Pts(1)/Pts(3)];

            % Each error is affected by of the corresponding landmark (2x3)
            J(error_i+2*(iter_ki-1):error_i+2*iter_ki-1, 1+6*N+3*(idx_landmark(iter_ki)-1):6*N+3*idx_landmark(iter_ki)) = ...
                J(error_i+2*(iter_ki-1):error_i+2*iter_ki-1, 6*(iter_frame-1)+1:6*iter_frame-3)*T_kw(1:3,1:3);            

        end

        observation_i = observation_i + 1 + 3*ki; % ith img 속, 관측되는 keypoint 개수 나타내는 항
        error_i = error_i + 2*ki; % jacobian에서 ith img에 해당하는 error 행 시작
    end

    % Breaking criteria
    if iter_opt == 1
        residual_norm(1) = sqrt(sum(r.^2)/length(r));
        fprintf('iter: %d, total err: %0.6f, mean err: %0.6f,  err rate: %0.6f,  lam: %0.6f\n',...
            iter_opt, residual_norm(1), mean(sqrt(sum(reshape(r.^2, 2, [])))), 0, lam);
    else
        residual_norm(iter_opt) = sqrt(sum(r.^2)/length(r));
        
        r_prev = residual_norm(iter_opt-1);
        r_curr = residual_norm(iter_opt);
        r_rate = abs(r_curr - r_prev)/r_prev ;
        fprintf('iter: %d, total err: %0.6f, mean err: %0.6f,  err rate: %0.6f,  lam: %0.6f\n',...
            iter_opt, r_curr, mean(sqrt(sum(reshape(r.^2, 2, [])))), r_rate, lam);

        % Breaking criteria
        if(abs(r_rate) <= 1e-4) || (iter_opt == iter_max)
            hidden_state_optimal = x_iter;
            break;
        end
        
        % dynamically change damping parameter
        if(r_curr <= r_prev)
          lam = lam*0.75;
        else
          lam = 2*lam_init;
          disp('warning')
        end
        if(lam < 1e-4)
          lam = 1e-4;
        end
        if(lam > 50)
          lam = 50;
        end
    end
    
    
    % Huber weight
    ind_over = find(abs(r) >= huber_parameter);
    W  = ones(length(r),1);
    W(ind_over) = huber_parameter./abs(r(ind_over));
    
    
    JW  = bsxfun(@times, J, W); % 0.15 초
    
    % compute Hessian w/ damping of LM method
    H = JW'*J; % Hessian
    H = H + lam*diag(diag(H));
    
    % update state w/ schur complement
    b = JW'*r;
    b_xi = b(1:6*N,1);
    b_p = b(6*N+1:end,1);
    H_xixi = H(1:6*N,1:6*N);
    H_xip = H(1:6*N, 6*N+1:end);
    H_pp = H(6*N+1:end,6*N+1:end);
    
    H_pp_inv = H_pp;
    for iter_invHpp = 1:M
        H_pp_inv(3*iter_invHpp-2:3*iter_invHpp, 3*iter_invHpp-2:3*iter_invHpp) = ...
            H_pp(3*iter_invHpp-2:3*iter_invHpp, 3*iter_invHpp-2:3*iter_invHpp)^-1;
    end
    inv_term = H_xixi-H_xip*H_pp_inv*H_xip';
%     cond(inv_term)
    del_x_xi = inv_term\(-b_xi+H_xip*H_pp_inv*b_p);
    del_x_p = H_pp_inv*(-b_p+H_xip'*del_x_xi);
    del_x = [del_x_xi;del_x_p]; % 0.015 초
    
%     del_x = H\(-b);
    
    
    x_iter = x_iter - del_x;
    
    iter_opt = iter_opt + 1;
end



end