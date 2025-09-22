clear all;

% Plot settings
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');

%% Prompt user to choose which model to identify
model = -1; % Initialize the choice variable

while model ~= 0 && model ~= 1 && model ~= 2
    model = input(['Enter 0 to identify the kinematic model,\n' ...
                   '      1 to identify the extended kinematic model, or \n' ...
                   '      2 to identify the dynamic model: ']);
    if model == 0
        disp('Identifying the kinematic model');
    elseif model == 1
        disp('Identifying the extended kinematic model');
    elseif model == 2
        disp('Identifying the dynamic model');
    else
        disp('Invalid entry. Please enter either 0, 1 or 2.');
    end
end

%% Data extraction and parameter definitions
% Extract training data
data = readtable(['training_data.csv']);
timestamp = data.Timestamp;

% World-frame pose
odom_x = data.X;
odom_y = data.Y;
yaw = data.Yaw;

% Body-frame velocities
odom_vx = data.VX;
odom_vy = data.VY;
odom_az = data.VYaw;

% PWM inputs
lf_omega = data.Left_Front_Input;
lr_omega = data.Left_Rear_Input;
rf_omega = data.Right_Front_Input;
rr_omega = data.Right_Rear_Input;

u = [lf_omega rf_omega rr_omega lr_omega];

tfinal = seconds(table2array(data(end, 1) - data(1, 1))); % Total duration
% Extract sampling time from dataset
t = linspace(0, tfinal, size(u,1));
dt  = t(1,2);

x = [odom_x odom_y yaw];
nx = size(x,2);
nu = size(u,2);

dx_body = [odom_vx odom_vy odom_az];
dx_world = zeros(size(dx_body,1),nx);
dx_world(:,3) = dx_body(:,3); % Angular velocity remains the same

% Translate body-frame velocities to world-frame
for k = 1:length(odom_x)
    R_T = [ cos(yaw(k)), -sin(yaw(k));
           sin(yaw(k)), cos(yaw(k)) ];
    dx_world(k,1:2) = (R_T * [ dx_body(k,1); dx_body(k,2) ])';
end

polyorder = 1;
% Sparse parameter
if model == 0
    lambda_vec = [1e-4,1e-4,1e-4]; % Pure kinematic model
elseif model == 1
    lambda_vec = [1e-4,1e-4,1e-5]; % Extended kinematic model
    polyorder = 2; % Polynomial order
else
    lambda_vec = [1e-5,1e-5,1e-5]; % Dynamic model
end

%% Total Variation Regularised Differentiation
dx = zeros(size(dx_body,1),nx);
for i=1:nx
    dx(:,i) = TVRegDiff(x(1:end-1,i), 100, 0.01,[], 'small', 1e-6, dt, 0, 0);
end

% TVR against translated measured velocities plot
figure;
tiledlayout(3,1);

nexttile;
plot(t,  dx_world(:,1),'--','LineWidth',1.2);
title('Measured $\&$ TVR Velocities','FontSize',14);
ylabel('$v_x$ (m/s)','FontSize',12);
grid on;
hold on;
plot(t, dx(:,1),'LineWidth',1.2);
hold off;
box off;

nexttile;
plot(t,  dx_world(:,2),'--','LineWidth',1.2);
ylabel('$v_y$ (m/s)','FontSize',12);
grid on;
hold on;
plot(t, dx(:,2),'LineWidth',1.2);
hold off;
box off;

nexttile;
plot(t, dx_world(:,3),'--','LineWidth',1.2);
ylabel('$\Omega$ (rad/s)','FontSize',12);
xlabel('Time (s)','FontSize',12);
grid on;
hold on;
plot(t, dx(:,3),'LineWidth',1.2);
hold off;
box off;
lgd = legend('Measured','TVR','Orientation','horizontal');
lgd.Location = 'southoutside';
lgd.Box = 'off';
lgd.Color = 'none';
lgd.FontSize = 10;
% exportgraphics(gcf,'Results/training/tvr_vel.pdf','ContentType','vector','BackgroundColor','none');

% Obtain second order derivatives for the dynamic model only
if model == 2
    ddx = zeros(size(dx_body,1),nx);
    for i=1:nx
        ddx(:,i) = TVRegDiff(dx(1:end-1,i), 100, 0.01,[], 'small', 1e-6, dt, 0, 0);
    end
end

%% SINDYc Algorithm

if model == 0 || model == 1
    lib_vars = [x u]; % Variables within the library
    % Convert to string for displaying the library
    str_vars = [ arrayfun(@(i) sprintf('x%d', i), 1:size(x,2), 'UniformOutput', false), ...
        arrayfun(@(i) sprintf('u%d', i), 1:size(u,2), 'UniformOutput', false)];
else
    lib_vars = [x dx_body u];
    str_vars = [ arrayfun(@(i) sprintf('x%d', i), 1:size(x,2), 'UniformOutput', false), ... 
        arrayfun(@(i) sprintf('x%ddot_b', i), 1:size(dx,2), 'UniformOutput', false), ...
        arrayfun(@(i) sprintf('u%d', i), 1:size(u,2), 'UniformOutput', false)];
end

% Sparse regression setup
n = size(lib_vars,2);
clear Theta Xi
Theta = eval_lib(lib_vars,n,polyorder); % Compute library

% Construct preliminary library with all terms for all state derivatives
allVars = disp_lib(str_vars, [], n, polyorder); 

Xi = zeros(size(Theta,2), 3); % Sparse coefficient matrix

if model == 0 || model == 1
    % Mask library terms per state derivative used for the pure kinematic model
    keep_x1d = false(1,size(Theta,2));
    keep_x2d = false(1,size(Theta,2));
    keep_x3d = false(1,size(Theta,2));
    
    for k = 1:numel(allVars)
      name = allVars{k};
      if contains(name, {'sin(x3)', 'cos(x3)'})
        keep_x1d(k) = true;
      end
     if contains(name, {'sin(x3)', 'cos(x3)'})
        keep_x2d(k) = true;
     end
       if contains(name, {'u'}) && ~contains(name, {'sin','cos'})
          keep_x3d(k) = true;
      end
    end
    if model == 1 % Unrestrict x and y state derivatives for extended kinematic
        keep_x1d = true(1,size(Theta,2));
        keep_x2d = true(1,size(Theta,2));
    end
        
    % STLSQ to determine sparse coefficients
    Xi(keep_x1d, 1) = STLSQ(Theta(:,keep_x1d), dx(:,1), lambda_vec(1), 10, 1e-8);
    Xi(keep_x2d, 2) = STLSQ(Theta(:,keep_x2d), dx(:,2), lambda_vec(2), 10, 1e-8);
    Xi(keep_x3d, 3) = STLSQ(Theta(:,keep_x3d), dx(:,3), lambda_vec(3), 10, 1e-8);

else % Library for dynamic model
    keep_x1dd = false(1,size(Theta,2));
    keep_x2dd = false(1,size(Theta,2));
    keep_x3dd = false(1,size(Theta,2));

    for k = 1:numel(allVars)
        name = allVars{k};
        if contains(name, {'sin(x3) * u', 'cos(x3) * u', 'cos(x3) * x1dot_b', 'sin(x3) * x2dot_b' })
            keep_x1dd(k) = true;
        end
        if  contains(name, {'sin(x3) * u', 'cos(x3) * u', 'cos(x3) * x2dot_b', 'sin(x3) * x1dot_b' })
            keep_x2dd(k) = true;
        end
        if ~contains(name, {'sin', 'cos', 'x1dot_b', 'x2dot_b'})
            keep_x3dd(k) = true;
        end
    end

    Xi(keep_x1dd, 1) = STLSQ(Theta(:,keep_x1dd), ddx(:,1), lambda_vec(1), 10, 1e-8);
    Xi(keep_x2dd, 2) = STLSQ(Theta(:,keep_x2dd), ddx(:,2), lambda_vec(2), 10, 1e-8);
    Xi(keep_x3dd, 3) = STLSQ(Theta(:,keep_x3dd), ddx(:,3), lambda_vec(3), 10, 1e-8);
end

% Extract coefficients
[~,sindyc_eqns] = disp_lib(str_vars, Xi, n, polyorder);
extract_eqns(sindyc_eqns,model); % Generate symbolic representation of dynamics



%% Functions
function extract_eqns(sindyc_eqns,model)
    
    lib_terms = sindyc_eqns(1, 2:end); % Extract library terms
    eqns = cell(1, length(lib_terms)); % Initialise cell
    
    for i = 2:length(lib_terms)+1 % Iterate over columns
        eqn = '';
        for j = 2:size(sindyc_eqns, 1) % Iterate over rows
            coeff = sindyc_eqns{j, i};

            if coeff ~= 0 % Ignore coefficients that are 0
                termName = sindyc_eqns{j, 1}; % Extract term name
                
                % Format the term to handle negations, and multiplications
                if coeff == 1
                    termStr = termName;
                elseif coeff == -1
                    termStr = ['-', termName];
                else
                    termStr = [num2str(coeff), '*', termName];
                end
                
                if isempty(eqn)
                    eqn = termStr;
                else
                    if coeff > 0 % Append string
                        eqn = [eqn, ' + ', termStr];
                    else
                        eqn = [eqn, ' ', termStr];
                    end
                end
            end
        end
        eqns{i-1} = eqn;
    end
    
    % Display system equations
    if model == 0 || model == 1
        for i = 1:3
            fprintf('%s = %s;\n', lib_terms{i}, eqns{i});
        end
    else
        for i = 1:3
            fprintf('d_%s = %s;\n', lib_terms{i}, eqns{i});
        end
    end
end


function Theta = eval_lib(lib_vars,n,polyorder)
    
    ind = 1;
    
    % Include PWM signals u
    for i=4:n
        Theta(:,ind) = lib_vars(:,i);
        ind = ind+1;
    end
    
    % Polynomials of order 2 for PWM signals only
    if(polyorder>=2)
        for i=4:n
            jvars = i;
            for j=jvars:n
                Theta(:,ind) = lib_vars(:,i).*lib_vars(:,j);
                ind = ind+1;
            end
        end
    end
    
    % Polynomials of order 3 for PWM signals only
    if(polyorder>=3)
        for i=4:n
            Theta(:,ind) = lib_vars(:,i).*lib_vars(:,i).*lib_vars(:,i);
            ind = ind+1;
        end
    end
    
    sin_terms = sin(lib_vars(:,3));  % Compute sine terms
    cos_terms = cos(lib_vars(:,3));  % Compute cosine terms
    % Trig terms of the heading angle multiplied with PWM signals u
    for j=4:n
        Theta(:,ind) = sin_terms .* lib_vars(:,j); % sin(x_i) * u_j
        ind = ind + 1;
        Theta(:,ind) = cos_terms .* lib_vars(:,j); % cos(x_i) * u_j
        ind = ind + 1;
    end

end

function [lib_terms,Theta] = disp_lib(lib_vars,Xi,n,polyorder)

    % Similar to eval_lib function, but terms are converted to strings for
    % displying the library
    ind = 1;
    
    for i=4:n
        lib_terms(ind,1) = lib_vars(i);
        ind = ind+1;
    end
    
    if(polyorder>=2)
        for i=4:n
            jvars = i;
            for j=jvars:n
                lib_terms{ind,1} = [lib_vars{i},'*', lib_vars{j}];
                ind = ind+1;
            end
        end
    end
    
    if(polyorder>=3)
        for i=4:n
            lib_terms{ind,1} = [lib_vars{i},'*', lib_vars{i},'*', lib_vars{i}];
            ind = ind+1;
        end
    end
    
    for j=4:n
        lib_terms{ind,1} = ['sin(', lib_vars{3}, ') * ', lib_vars{j}];
        ind = ind + 1;
        lib_terms{ind,1} = ['cos(', lib_vars{3}, ') * ', lib_vars{j}];
        ind = ind + 1;
    end
    
    % Reorganise output
    Theta(1) = {''};
    for k=1:3
        Theta{1,1+k} = [lib_vars{k},'dot'];
    end
    for k=1:size(Xi,1)
        Theta(k+1,1) = lib_terms(k);
        for j=1:3
            Theta{k+1,1+j} = Xi(k,j);
        end
    end
    Theta
end

function Xi = STLSQ(Theta, dx, lambda, max_iters, tol)

    lambda_2 = 1e-5;  % Small ridge regularisation term
    Theta_norm = vecnorm(Theta);  % Compute column-wise norm
    Theta = Theta ./ Theta_norm;  % Normalise Theta

    % Initial least-squares guess
    Xi = (Theta' * Theta + lambda_2 * eye(size(Theta,2))) \ (Theta' * dx);
    
    %% Sweep lambda values (while tuning)
    % lambda_vals = logspace(-8, -4, 3);
    % lambda_vals = perms(lambda_vals);
    % errors = zeros(size(lambda_vals,1),1);

    % for i = 1:length(lambda_vals)
       % Xi = (Theta' * Theta + lambda_2 * eye(size(Theta,2))) \ (Theta' * dx);
        % lambda = lambda_vals(i,:)';

        for k = 1:max_iters
            Xi_old = Xi;
    
            for ind = 1:size(lambda,2)
                % Threshold using true coefficient magnitudes
                smallinds = abs(Xi(:,ind) ./ Theta_norm') < lambda(ind);
                Xi(smallinds,ind) = 0;
    
                biginds = ~smallinds;
                if any(biginds)
                    Xi(biginds,ind) = (Theta(:,biginds)' * Theta(:,biginds) + lambda_2 * eye(sum(biginds))) \ ...
                                      (Theta(:,biginds)' * dx(:,ind));
                end
            end
           
            % Check for convergence
            if norm(Xi - Xi_old, 'fro') < tol
                fprintf('Converged in %d iterations.\n', k);
                break;
            end
        end
        %% Continutation for sweeping lambda
        % errors(i) = norm(Theta * Xi - dx); % Compute error
    % end
    % figure;
    % semilogx(1:length(lambda_vals), errors, '-o');
    % xlabel('\lambda');
    % ylabel('Error');
    % title('Tuning Sparse Regression Parameter');
    % grid on;

    % Scale Xi back
    Xi = Xi ./ Theta_norm';
end
