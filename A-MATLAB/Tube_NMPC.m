clear all;
close all;
clc;

tic % Start timer

% Plot settings
set(groot,'defaultAxesFontSize',11);
set(groot,'defaultAxesLabelFontSizeMultiplier',1.05);
set(groot,'defaultAxesTitleFontSizeMultiplier',1.05);
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendFontSize',12);

col_ref = [0 0 0];
col_nom = [0 0.4470 0.7410];
col_real= [0.85 0.10 0.10];
col_tube= [0.20 0.80 0.20];

%% Reference Trajectory

% Prompt user to choose which trajectory to simulate
traj = -1; % Initialise the choice variable

while traj ~= 0 && traj ~= 1 && traj ~= 2
    traj = input(['Enter 0 to simulate a rectangular path with constant heading,\n' ...
                  ' 1 to simulate a circular path with constant heading, or \n ' ...
                  ' 2 to simulate a circular path with changing heading: ']);
    if traj == 0
         disp('Simulating a rectangular path with constant heading');
         duration = 25;
         a = 0.3; % Half-width
         b = 0.2; % Half-height
         xrefFUN = @(t) rectTrajectory(t, duration, a, b);
        
     elseif traj == 1
         duration = 30;
         disp('Simulating a circular path with constant heading');
         xrefFUN = @(t) [0.4*cos(2*pi*t'/duration) 0.4*sin(2*pi*t'/duration) zeros(size(t'))];
         
     elseif traj == 2
         duration = 30;
         disp('Simulating a circular path with changing heading');
         xrefFUN = @(t) [0.6*cos(2*pi*t'/duration) 0.6*sin(2*pi*t'/duration) ...
           unwrap(atan2(0.6 * (2*pi/duration) * cos(2*pi*t'/duration), -0.6 * (2*pi/duration) * sin(2*pi*t'/duration)))];
     
     else
        disp('Invalid entry. Please enter either 0, 1 or 2.');
     end
end

Ts = 0.1; % Sampling time
tsim = 0:Ts:duration;
Nt = floor((duration/Ts)+1); % Number of samples in simulation
xref = xrefFUN(tsim);
x0 = [xref(1,1), xref(1,2), xref(1,3)];

%% Tube-based NMPC Parameters
Np = 10; % Prediction horizon
Nu = Np; % Control horizon
nx = 3;
nu = 4;

% NMPC Weights
Q = 1000*diag([1 1 1]); % State 
Qf = ([3511.06992126953	233.248480516098	-90.1364787673133; ...
         233.248480516098	7447.46289178297	-30.9808550672083; ...
         -90.1364787673133	-30.9808550672083	1411.99502643508]); % Terminal
Rdu = 0.001*diag([1 1 1 1]); % Input-rate
Ru = 0.001*diag([1 1 1 1]); % Input

% Input & input-rate constraints
LB = -100*ones(nu,1);  
UB = -LB;
LBdu = -30*ones(Nt,nu); 
UBdu = -LBdu;

% Initial tightened constraints
LB_tightened = 0.9*LB; 
UB_tightened = -LB_tightened;

% Initial bounds for optimisation including for x0_hat
LBz = [-inf(nx,1); repmat(LB_tightened, Np, 1)];
UBz = [ inf(nx,1); repmat(UB_tightened, Np, 1)];

% Parameters NMPC
options = optimoptions('fmincon','Algorithm','sqp','Display','none', ...
 'MaxIterations',50, 'OptimalityTolerance',1e-3, 'StepTolerance',1e-3);

% Ancillary control parameters
Q_lqr = diag([1000, 1000, 1000]);
R_lqr = 0.001 * eye(nu);
w_max = [0.04, 0.03, 0.09];

%% Initialise parameters
u_nom = zeros(Nu,nu);
u_nomHistory = zeros(Nt,nu);
u_actualHistory = zeros(Nt,nu);

x0_nomHistory = zeros(Nt,nx);
x_nomHistory = zeros(Nt,nx);
x_nomHistory(1,:) = x0;
x_refHistory = zeros(Nt,nx);

tHistory = zeros(Nt,1);

LB_tightHistory = zeros(Nt,nu);
UB_tightHistory = zeros(Nt,nu);

% Ancillary control
[A,B,C,D] = lin_sys;
Phi = zeros(nx,nx,Nt); % Closed-loop discrete system
P = zeros(nx,nx,Nt); % DLQR Solution
e = zeros(Nt,nx); % Error matrix

% Initialise states
u_nomHistory(1,:) = u_nom(1,:);
x_refHistory(1,:) = x0;
x_nom(1,:) = x0;
x_actual(1,:) = x0;
tHistory(1) = 0;

rpi = zeros(nx,Nt+1); % Initialise error tube with {0}
z_opt = [zeros(nx,1);zeros(Nu*nu,1)]; % Augmented vector [x0_nom,u_nom]

%% Nominal NMPC & Ancillary DLQR
for i = 1:Nt

 % Set references over optimisation horizon
 if i < Nt-Np+1
    xref_loc = xref(i:i+Np-1,:);
 else % Pad last Np samples
    xref_loc = [xref(i:end,:); ones(Np-size(xref(i:end,:),1),1)*xref(end,:)]; %last Np rows must have repeated last entry
 end

 % Set up cost function & constraints
 Jz = @(z) Obj_fn_z(z, nx, nu, Ts, Np, xref_loc, u_nomHistory(i,:), Q, Qf, Rdu, Ru);
 conz = @(z) Constraint_fn_z(z, nx, nu, Np, u_nomHistory(i,:), LBdu(i,:), UBdu(i,:), x_actual(i,:), rpi(:,i));
 
 % Solve OCP
 z_opt = fmincon(Jz, z_opt, [],[],[],[], LBz(:), UBz(:), conz, options);

 [x0_hat_opt, u_nom] = unpack_decision(z_opt, nx, nu, Np);

 % Nominal dynamics (no disturbance)
 x_nom(i+1,:) = propagate_dynamics(x0_hat_opt, u_nom(1,:), Ts, [0 0 0]);

 % Linearise around initial optimised state to compute DLQR gain
 A_sub = double(subs(A, {'x1','x2','x3','u1','u2','u3','u4'}, ...
                {x0_hat_opt(1), x0_hat_opt(2), x0_hat_opt(3), u_nom(1,1), u_nom(1,2), u_nom(1,3), u_nom(1,4)}));
 
 B_sub = double(subs(B, {'x1','x2','x3','u1','u2','u3','u4'}, ...
                {x0_hat_opt(1), x0_hat_opt(2), x0_hat_opt(3), u_nom(1,1), u_nom(1,2), u_nom(1,3), u_nom(1,4)}));

 % Discretise system using 1st order forward-Euler approximation
 A_d = eye(nx) + Ts*A_sub;
 B_d = Ts*B_sub;

 % DLQR
 [K,P,~] = dlqr(A_d, B_d, Q_lqr, R_lqr);
 % disp(P); % Used once to obtain the terminal weights of the NMPC

 % Calculate error
 e(i,:) = x_actual(i,:) - x0_hat_opt;
 u_real = u_nom(1,:)' - K * e(i,:)';

 if any(u_real > UB) || any(u_real < LB) % Feasibility check
 warning('Total applied control input exceeds bounds %d', i);
 end
 
 % Component-wise uniform disturbance bounded by |w_max|
 disturbance = [(2*rand()-1)*w_max(1), (2*rand()-1)*w_max(2), (2*rand()-1)*w_max(3)];
 % Simulate fictitous "actual" system
 x_actual(i+1,:) = propagate_dynamics(x_actual(i,:), u_real, Ts, disturbance);

 % RPI set computation
 Phi(:,:,i) = A_d - B_d * K;

 rpi(:,1) = abs(e(1,:)'); % RPI set is the error dynamics in the first step
 rpi(:,i+1) = abs(Phi(:,:,i))*rpi(:,i) + w_max(:)*Ts; % One-step propagation 
                                                      % based on worst-case disturbance
 du_bound = abs(K)*rpi(:,i); % Calculate new tightened bounds

 if any(LB(:)+du_bound > UB(:)-1e-9) % Feasibility check
 warning('Tightening makes input infeasible at stage %d', i-1);
 end

 LB_tightened = LB + du_bound;
 UB_tightened = UB - du_bound;
 % Update bounds for next step
 LBz = [-inf(nx,1); repmat(LB_tightened, Np, 1)];
 UBz = [ inf(nx,1); repmat(UB_tightened, Np, 1)];

 % Store for plots
 LB_tightHistory(i+1,:) = LB_tightened';
 UB_tightHistory(i+1,:) = UB_tightened';
 u_actualHistory(i,:) = u_real;
 x_nomHistory(i,:) = x_nom(i,:);
 x0_nomHistory(i,:) = x0_hat_opt;
 u_nomHistory(i,:) = u_nom(1,:);
 tHistory(i,:) = i*Ts;
 x_refHistory(i,:) = xref_loc(1,:);
end

t = toc; % Stop timer
fprintf('Elapsed time: %.6f s\n', t);

%% Plots

% XY Plot with tube
figure;
tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
nexttile(tl, [2 1]); hold on; grid on;
title('Tube-based NMPC World Frame Pose','FontSize',12);
xlabel('x (m)','FontSize',12); ylabel('y (m)','FontSize',12);

plot(xref(:,1), xref(:,2), '-', 'Color', col_ref, 'LineWidth', 1.6);
plot(x_nom(:,1), x_nom(:,2), '-', 'Color', col_nom, 'LineWidth', 1.6);
plot(x_actual(:,1),x_actual(:,2),'--','Color', col_real,'LineWidth', 1.4);

exceed_xy = 0;
R = eye(2);
step = 3; % Draw tube every 3 steps

% Visualise RPI set
for i = 1:step:(size(x_nom,1)-1)
 rx = rpi(1,i);
 ry = rpi(2,i);
 C = x_nom(i,1:2)';  % Center

 corners = [ -rx -ry;
           rx -ry;
           rx ry;
          -rx ry ]';

 box = R*corners + C;
 patch('XData', box(1,[1 2 3 4]), 'YData', box(2,[1 2 3 4]), ...
        'FaceColor', col_tube, 'FaceAlpha', 0.20, 'EdgeColor', 'none', ...
        'DisplayName','Box tube');

 % Check that all actual points are within set
 e_loc = e(i,1:2)';
 inside = all(abs(e_loc) <= [rx; ry] + 1e-12);
 if ~inside
    exceed_xy = exceed_xy + 1;
 end

 plot(x_actual(i,1), x_actual(i,2), 'x', 'Color', col_real, 'LineWidth', 1.0, 'MarkerSize', 6);
end

legend({'Reference','Nominal','Actual','Box tube','Actual sample'}, 'Location','southoutside','Orientation','horizontal','Box','off','FontSize',10.5);

% Yaw
nexttile(tl); hold on; grid on;
xlabel('Time step','FontSize',12); ylabel('Yaw (rad)','FontSize',13);
exceed_yaw = 0;

Nt = size(x_nom,1)-1;
t_yaw = tHistory(1:Nt);

% RPI set
upper = x_nom(1:Nt,3) + rpi(3,1:Nt)';
lower = x_nom(1:Nt,3) - rpi(3,1:Nt)';

% Check if all points are within RPI set
inside = all(abs(e(:,3)) <= rpi(3,:) + 1e-12);
if ~inside
    exceed_yaw = exceed_yaw + 1;
end

plot(t_yaw, xref(1:Nt,3), '-', 'Color', col_ref, 'LineWidth', 1.6, 'DisplayName','Reference');
plot(t_yaw, x_nom(1:Nt,3), '-', 'Color', col_nom, 'LineWidth', 1.6, 'DisplayName','Nominal');
plot(t_yaw, x_actual(1:Nt,3),'--','Color', col_real,'LineWidth', 1.4,'DisplayName','Actual');

tt = [t_yaw.' fliplr(t_yaw.')];
yy = [upper.' fliplr(lower.')];
fill(tt, yy, col_tube, 'FaceAlpha', 0.20, 'EdgeColor','none', 'DisplayName','Box tube');

xlim([t_yaw(1) t_yaw(end)]);

fprintf('XY box tube violations: %d\n', exceed_xy);
fprintf('Yaw box tube violations: %d\n', exceed_yaw);

% Error Plots
ex = x_actual(1:end-1,1) - xref(:,1);
ey = x_actual(1:end-1,2) - xref(:,2);

% Wrap yaw error to [-pi,pi] for clarity
dth = x_actual(1:end-1,3) - xref(:,3);
eth = atan2(sin(dth), cos(dth));

rmse = @(z) sqrt(mean(z.^2));
rmse_x = rmse(ex);
rmse_y = rmse(ey);
rmse_th = rmse(eth);

figure;
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

% x error
nexttile; hold on; grid on;
plot(tHistory, ex, 'LineWidth', 1.3, 'Color', col_nom);
yline(0,'-','Color',[0.4 0.4 0.4],'LineWidth',0.8,'HandleVisibility','off');
ylabel('$e_x$ (m)','FontSize',13);
xlim([tHistory(1) tHistory(end-1)]);
title(sprintf('State RMSE w.r.t Reference: $e_x$ = %.3gm, $e_y$ = %.3gm, $e_{\\theta}$ = %.3grad', ...
                rmse_x, rmse_y, rmse_th), 'Interpreter','latex','FontSize',12);

% y error
nexttile; hold on; grid on;
plot(tHistory, ey, 'LineWidth', 1.3, 'Color', [0.85 0.325 0.098]);
yline(0,'-','Color',[0.4 0.4 0.4],'LineWidth',0.8,'HandleVisibility','off');
ylabel('$e_y$ (m)','FontSize',13);
xlim([tHistory(1) tHistory(end-1)]);

% yaw error
nexttile; hold on; grid on;
plot(tHistory, eth, 'LineWidth', 1.3, 'Color', [0.466 0.674 0.188]);
yline(0,'-','Color',[0.4 0.4 0.4],'LineWidth',0.8,'HandleVisibility','off');
ylabel('$e_\theta$ (rad)','FontSize',13); xlabel('Time step','FontSize',13);
xlim([tHistory(1) tHistory(end-1)]);

names = {'Left Front','Left Rear','Right Front','Right Rear'};

figure; hold on; grid on;
ax = gca;
co = ax.ColorOrder;

% Bounds as dashed lines in same color
for k = 1:4
 c = co(mod(k-1,size(co,1))+1, :);
 plot(tHistory, LB_tightHistory(1:end-1,k), '--', 'Color', c, 'LineWidth', 0.5, ...
  'HandleVisibility','off');
 plot(tHistory, UB_tightHistory(1:end-1,k), '--', 'Color', c, 'LineWidth', 0.5, ...
  'HandleVisibility','off');
end

% Control inputs in solid lines
h = gobjects(1,4);
for k = 1:4
    c = co(mod(k-1,size(co,1))+1, :);
    h(k) = plot(tHistory, u_actualHistory(:,k), '-', 'Color', c, ...
                'LineWidth', 1.6, 'DisplayName', names{k});
end

xlim([tHistory(1) tHistory(end-1)]);
xlabel('Time step','FontSize',12);
ylabel('Wheel PWM Signal','FontSize',12);
title('Total Applied Control Input PWM Signals','FontSize',12);
legend(h, 'Location','southoutside','Orientation','horizontal','Box','off','FontSize',12);
hold off;
% exportgraphics(gcf,'Results/Simulation/sim_inputs_ml.pdf','ContentType','vector','BackgroundColor','none');

%% Functions
% RK4 integration returning a discrete map x(k+1) = f(x(k),u(k))
function x_next = propagate_dynamics(x, u, Ts, disturbance)

 f1 = SINDYc_model(0, x, u,disturbance)';
 f2 = SINDYc_model(0, x + 0.5*Ts*f1, u,disturbance)';
 f3 = SINDYc_model(0, x + 0.5*Ts*f2, u,disturbance)';
 f4 = SINDYc_model(0, x + Ts*f3, u,disturbance)';
 x_next = x + Ts*(f1 + 2*f2 + 2*f3 + f4)/6;
end

% Continuous-time model identified from SINDYc (Extended Kinematic Model)
function x_next = SINDYc_model(t, x, u,w)

 x3 = x(3);
 u1 = u(1);
 u2 = u(2);
 u3 = u(3);
 u4 = u(4);
 
 x1dot = -0.00016598*u1 -0.00044484*u2 + 0.00028542*u3 + 0.00021938*u4 + 0.00052793*sin(x3) * u1 + 0.0022203*cos(x3) * u1 -0.00066344*sin(x3) * u2 + 0.0023484*cos(x3) * u2 + 0.00087885*sin(x3) * u3 + 0.0013426*cos(x3) * u3 -0.00075726*sin(x3) * u4 + 0.00081618*cos(x3) * u4;
 x2dot = 0.0004803*u1 + 0.000149*u2 -0.00029443*u3 -0.00045783*u4 + 0.0013242*sin(x3) * u1 -0.00085305*cos(x3) * u1 + 0.0016074*sin(x3) * u2 + 0.00068381*cos(x3) * u2 + 0.0023134*sin(x3) * u3 -0.00074272*cos(x3) * u3 + 0.0021253*sin(x3) * u4 + 0.00088697*cos(x3) * u4;
 x3dot = -0.0043797*u1 + 0.008181*u2 + 0.0046648*u3 -0.0078721*u4 -7.1607e-05*u1*u1 -7.2158e-05*u1*u2 + 0.00011008*u1*u3 + 1.6506e-05*u1*u4 + 5.1765e-05*u2*u3 -2.1046e-05*u2*u4 -3.0088e-05*u3*u3 + 2.7602e-05*u4*u4;

 % Body-frame disturbance is translated to world-frame
 G = [cos(x3), -sin(x3), 0;
  sin(x3), cos(x3), 0;
  0, 0, 1];
 x_next = [x1dot; x2dot; x3dot] + G * w';
end

% Linearise system
function [A,B,C,D] = lin_sys

 syms x1 x2 x3
 syms u [4,1]
 x = [x1;x2;x3];
 u1 = u(1);
 u2 = u(2);
 u3 = u(3);
 u4 = u(4);
 
 x1dot = -0.00016598*u1 -0.00044484*u2 + 0.00028542*u3 + 0.00021938*u4 + 0.00052793*sin(x3) * u1 + 0.0022203*cos(x3) * u1 -0.00066344*sin(x3) * u2 + 0.0023484*cos(x3) * u2 + 0.00087885*sin(x3) * u3 + 0.0013426*cos(x3) * u3 -0.00075726*sin(x3) * u4 + 0.00081618*cos(x3) * u4;
 x2dot = 0.0004803*u1 + 0.000149*u2 -0.00029443*u3 -0.00045783*u4 + 0.0013242*sin(x3) * u1 -0.00085305*cos(x3) * u1 + 0.0016074*sin(x3) * u2 + 0.00068381*cos(x3) * u2 + 0.0023134*sin(x3) * u3 -0.00074272*cos(x3) * u3 + 0.0021253*sin(x3) * u4 + 0.00088697*cos(x3) * u4;
 x3dot = -0.0043797*u1 + 0.008181*u2 + 0.0046648*u3 -0.0078721*u4 -7.1607e-05*u1*u1 -7.2158e-05*u1*u2 + 0.00011008*u1*u3 + 1.6506e-05*u1*u4 + 5.1765e-05*u2*u3 -2.1046e-05*u2*u4 -3.0088e-05*u3*u3 + 2.7602e-05*u4*u4;

 xdot = [x1dot; x2dot; x3dot];
 
 A = jacobian(xdot,x);
 B = jacobian(xdot,u);
 C = eye(3);
 D = zeros(3,4);
end

% NMPC cost function
function J = Obj_fn_z(z, nx, nu, Ts, Np, xref, u0, Q, Qf, R, Ru)
 
 [x0_hat, U] = unpack_decision(z, nx, nu, Np);

 xk = zeros(Np, nx);
 x = x0_hat;

 % RK4 integration for prediction horizon
 for k = 1:Np
 u = U(k,:);
 f1 = SINDYc_model(0, x,  u,[0 0 0])'; 
 f2 = SINDYc_model(0, x + 0.5*Ts*f1, u,[0 0 0])'; 
 f3 = SINDYc_model(0, x + 0.5*Ts*f2, u,[0 0 0])'; 
 f4 = SINDYc_model(0, x + Ts*f3, u,[0 0 0])'; 
 x = x + Ts*(f1 + 2*f2 + 2*f3 + f4)/6;
 xk(k,:) = x;
 end

 % Cost function: stage + input rate + control effort + terminal cost
 J = 0;
 uk = U(1,:);
 for k = 1:Np
 xk1 = xk(k,:);
 yaw_err = wrapToPi(xk1(3) - xref(k,3));
 e = [xk1(1:2)-xref(k,1:2), yaw_err];
 J = J + e*Q*e';

 if k==1
  J = J + (uk - u0)*R*(uk - u0)' + uk*Ru*uk';
 else
  J = J + (uk - U(k-1,:))*R*(uk - U(k-1,:))' + uk*Ru*uk';
 end

 if k < Np
  uk = U(k+1,:);
 else
  J = J + e*Qf*e';
 end
 end
end

% Constraints function
function [c, ceq] = Constraint_fn_z(z, nx, nu, N, uold, LBdu, UBdu, x_meas, rpi_k)

 [x0_hat, U] = unpack_decision(z, nx, nu, N);

 % Initial nominal state at the centre of tube with |x_meas - x0_hat| <= r_k
 dx0 = (x_meas(:) - x0_hat(:));
 c_x0 = [ dx0 - rpi_k;
  -dx0 - rpi_k];

 % Input-rate constraints across horizon
 c_rate = zeros(2*N, nu);
 duk = U(1,:) - uold; % For the first step, check the last applied nominal input
 for ct = 1:N
 c_rate(2*ct-1,:) = -duk + LBdu;
 c_rate(2*ct,:) = duk - UBdu;
 if ct < N
  duk = U(ct+1,:) - U(ct,:);
 end
 end

 c = [c_x0; c_rate(:)];
 ceq = [];
end

% Unpack augmented optimised state vector
function [x0_hat, U] = unpack_decision(z, nx, nu, Np)
 x0_hat = z(1:nx).';
 U = reshape(z(nx+1:end), Np, nu);
end

% Rectangular path with constant heading and padding per segment
function X = rectTrajectory(t, duration, a, b)

 Tseg = duration/4; % Time per side
 pad = 0.4;
 Tmove = Tseg - pad; % Moving time on a side

 tau = mod(t', duration); % Time within cycle
 seg = floor(tau / Tseg); % 0:right, 1:up, 2:left, 3:down
 tau_seg = tau - seg*Tseg; % Time within current segment

 % Increase position during moving, stay at 1 during padding
 u = min(1, tau_seg / Tmove);

 % Piecewise-linear sides with dwell at the end of each
 x = (-a + 2*a.*u).*(seg==0) + a.*(seg==1) + ...
 (a - 2*a.*u).*(seg==2) - a.*(seg==3);

 y = -b.*(seg==0) + (-b + 2*b.*u).*(seg==1) + ...
  b.*(seg==2) + (b - 2*b.*u).*(seg==3);

 theta = zeros(size(t')); % constant heading
 X = [x, y, theta];
end