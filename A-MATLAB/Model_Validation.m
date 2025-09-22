clc;
clear all;
close all;

% Plot settings
set(groot,'defaultAxesFontSize',11);
set(groot,'defaultAxesLabelFontSizeMultiplier',1.05);
set(groot,'defaultAxesTitleFontSizeMultiplier',1.05);
set(groot,'defaultLegendInterpreter','latex');
set(groot,'defaultTextInterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendFontSize',12);

col_actual = [0 0 0];                  % black
col_ext    = [0.000 0.447 0.741];      % blue
col_kin    = [0.850 0.325 0.098];      % orange
col_dyn    = [0.466 0.674 0.188];      % green
col_dyn_ext = [0.6 0.6 0.6];

%% Prompt user to choose which trajectory to validate
traj = -1; % Initialize the choice variable

while traj ~= 0 && traj ~= 1
    traj = input(['Enter 0 to verify against rectangular path with constant heading, or\n' ...
                  '      1 to verify against rectangular path with changing heading: ']);
    if traj == 0
        disp('Verify against a rectangular path with constant heading');
        data = readtable(['validation_const_rect.csv']);
    elseif traj == 1
        disp('Verify against a rectangular path with changing heading');
        data = readtable(['validation_turn_rect.csv']);
    else
        disp('Invalid entry. Please enter either 0, or 1.');
    end
end

%% Extract data

timestamp = data.Timestamp;
odom_x = data.X;
odom_y = data.Y;
yaw = data.Yaw;
odom_vx = data.VX;
odom_vy = data.VY;
odom_az = data.VYaw;
lf_omega = data.Left_Front_Input;
lr_omega = data.Left_Rear_Input;
rf_omega = data.Right_Front_Input;
rr_omega = data.Right_Rear_Input;

u = [lf_omega rf_omega rr_omega lr_omega];
dx_body = [odom_vx odom_vy odom_az];
x = [odom_x odom_y yaw];

% Initial world-frame velocities
v_x0_world = cos(yaw(1))*odom_vx(1) - sin(yaw(1))*odom_vy(1);
v_y0_world = sin(yaw(1))*odom_vx(1) + cos(yaw(1))*odom_vy(1);

tfinal = seconds(table2array(data(end, 1) - data(1, 1)));
t_valid = linspace(0, tfinal, size(u,1));
dt  = t_valid(1,2);

% Interpolate PWM input for solving ODE
u1_interp = @(t) interp1(t_valid, u(:, 1), t, 'previous');
u2_interp = @(t) interp1(t_valid, u(:, 2), t, 'previous');
u3_interp = @(t) interp1(t_valid, u(:, 3), t, 'previous');
u4_interp = @(t) interp1(t_valid, u(:, 4), t, 'previous');

% Initial conditions
x0 = [x(1,1); x(1,2); x(1,3)];
v0 = [v_x0_world; v_y0_world; dx_body(1,3)];
z0 = [x0; v0]; % For dynamic model

% Time span for the ODE solver
tspan = 0:dt:tfinal;

[t_kin, x_kin] = ode45(@(t, x) kin_ode(t, x, u1_interp(t), u2_interp(t), u3_interp(t), u4_interp(t)), tspan, x0);
[t_ext, x_ext] = ode45(@(t, x) ext_kin_ode(t, x, u1_interp(t), u2_interp(t), u3_interp(t), u4_interp(t)), tspan, x0);
[t_dyn, z_dyn] = ode45(@(t, z) dyn_ode(t, z, u1_interp(t), u2_interp(t), u3_interp(t), u4_interp(t)), tspan, z0);
[t_dyn_ext, z_dyn_ext] = ode45(@(t, z) ext_dyn_ode(t, z, u1_interp(t), u2_interp(t), u3_interp(t), u4_interp(t)), tspan, z0);

% Translate to body-frame velocities for comparison
v_kin = zeros(size(x_kin));
for k = 1:length(t_kin)
    v_kin(k,:) = kin_ode(t_kin(k), x_kin(k,:)', ...
                                u1_interp(t_kin(k)), ...
                                u2_interp(t_kin(k)), ...
                                u3_interp(t_kin(k)), ...
                                u4_interp(t_kin(k)));
    vx_world = v_kin(k,1);
    vy_world = v_kin(k,2);
    theta = x_kin(k,3);

    v_kin(k,1) = cos(theta)*vx_world + sin(theta)*vy_world;
    v_kin(k,2) = -sin(theta)*vx_world + cos(theta)*vy_world;
end

v_ext = zeros(size(x));
for k = 1:length(t_ext)
    v_ext(k,:) = ext_kin_ode(t_ext(k), x_ext(k,:)', ...
                                u1_interp(t_ext(k)), ...
                                u2_interp(t_ext(k)), ...
                                u3_interp(t_ext(k)), ...
                                u4_interp(t_ext(k)));
    vx_world = v_ext(k,1);
    vy_world = v_ext(k,2);
    theta = x_ext(k,3);

    v_ext(k,1) = cos(theta)*vx_world + sin(theta)*vy_world;
    v_ext(k,2) = -sin(theta)*vx_world + cos(theta)*vy_world;
end

v_dyn = zeros(size(x_ext));
for k = 1:length(t_dyn)
    vx_world = z_dyn(k,4);
    vy_world = z_dyn(k,5);
    theta = z_dyn(k,3);

    v_dyn(k,1) = cos(theta)*vx_world + sin(theta)*vy_world;
    v_dyn(k,2) = -sin(theta)*vx_world + cos(theta)*vy_world;
end

v_dyn_ext = zeros(size(x_ext));
for k = 1:length(t_dyn_ext)
    vx_world = z_dyn_ext(k,4);
    vy_world = z_dyn_ext(k,5);
    theta = z_dyn_ext(k,3);

    v_dyn_ext(k,1) = cos(theta)*vx_world + sin(theta)*vy_world;
    v_dyn_ext(k,2) = -sin(theta)*vx_world + cos(theta)*vy_world;
end

x_kin1 = x_kin(:, 1);
x_kin2 = x_kin(:, 2);
x_kin3 = x_kin(:, 3);

x_ext1 = x_ext(:, 1);
x_ext2 = x_ext(:, 2);
x_ext3 = x_ext(:, 3);

x_dyn1 = z_dyn(:, 1);
x_dyn2 = z_dyn(:, 2);
x_dyn3 = z_dyn(:, 3);
v_dyn(:,3) = z_dyn(:, 6);

x_dyn_ext1 = z_dyn_ext(:, 1);
x_dyn_ext2 = z_dyn_ext(:, 2);
x_dyn_ext3 = z_dyn_ext(:, 3);
v_dyn_ext(:,3) = z_dyn_ext(:, 6);

%% RMSE
rmse_vx_kin = rmse(odom_vx, v_kin(:,1));
rmse_vx_ext = rmse(odom_vx, v_ext(:,1));
rmse_vx_dyn = rmse(odom_vx, v_dyn(:,1));

rmse_vy_kin = rmse(odom_vy, v_kin(:,2));
rmse_vy_ext = rmse(odom_vy, v_ext(:,2));
rmse_vy_dyn = rmse(odom_vy, v_dyn(:,2));

rmse_az_kin = rmse(odom_az, v_kin(:,3));
rmse_az_ext = rmse(odom_az, v_ext(:,3));
rmse_az_dyn = rmse(odom_az, v_dyn(:,3));

rmse_x_kin = rmse(odom_x, x_kin1);
rmse_x_ext = rmse(odom_x, x_ext1);
rmse_x_dyn = rmse(odom_x, x_dyn1);

rmse_y_kin = rmse(odom_y, x_kin2);
rmse_y_ext = rmse(odom_y, x_ext2);
rmse_y_dyn = rmse(odom_y, x_dyn2);

rmse_z_kin = rmse(yaw, x_kin3);
rmse_z_ext = rmse(yaw, x_ext3);
rmse_z_dyn = rmse(yaw, x_dyn3);

%% Plots
figure;
sgtitle('Measured vs SINDYc Pose');
subplot(2,1,1);
hold on;
plot(odom_x, odom_y, '-', 'Color', col_actual, 'LineWidth',1.4, 'DisplayName','Measured');
plot(x_kin1, x_kin2, '--', 'Color', col_kin, 'LineWidth',1.6, 'DisplayName','Kinematic');
plot(x_ext1, x_ext2, ':', 'Color', col_ext, 'LineWidth',1.8, 'DisplayName','Extended Kinematic');
plot(x_dyn1, x_dyn2, '-.', 'Color', col_dyn, 'LineWidth',1.3, 'DisplayName','Dynamic');
% plot(x_dyn_ext1, x_dyn_ext2, 'Color', col_dyn_ext, 'LineWidth',1, 'DisplayName','Extended Dynamic');
hold off;
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('XY Position');
grid on;

subplot(2,1,2);
hold on;
plot(t_valid, yaw, '-', 'Color', col_actual, 'LineWidth',1.4, 'DisplayName','Measured');
plot(t_kin,x_kin3, '--', 'Color', col_kin, 'LineWidth',1.6, 'DisplayName','Kinematic');
plot(t_ext,x_ext3, ':', 'Color', col_ext, 'LineWidth',1.8, 'DisplayName','Extended Kinematic');
plot(t_dyn, x_dyn3, '-.', 'Color', col_dyn, 'LineWidth',1.3, 'DisplayName','Dynamic');
% plot(t_dyn_ext, x_dyn_ext3, 'Color', col_dyn_ext, 'LineWidth',1, 'DisplayName','Extended Dynamic');
hold off;
xlabel('Time (s)');
ylabel('Yaw Angle (rad)');
title('Yaw Angle');
legend('Location','southoutside','Orientation','horizontal','Box','off');
grid on;
% exportgraphics(gcf,'Results/ID/sindyc_turn_pose.pdf','ContentType','vector','BackgroundColor','none');

figure;
tiledlayout(3,1);
sgtitle('Measured vs SINDYc Velocity','Interpreter','latex');
nexttile;
hold on;
plot(t_valid, dx_body(:,1), '-', 'Color', col_actual, 'LineWidth',1.4, 'DisplayName','Measured');
plot(t_kin,v_kin(:,1), ':', 'Color', col_kin, 'LineWidth',1.8, 'DisplayName','Kinematic');
plot(t_ext,v_ext(:,1), '--', 'Color', col_ext, 'LineWidth',1.6, 'DisplayName','Extended Kinematic');
plot(t_dyn, v_dyn(:,1), '-.', 'Color', col_dyn, 'LineWidth',1.3, 'DisplayName','Dynamic');
% plot(t_dyn_ext, v_dyn_ext(:,1), 'Color', col_dyn_ext, 'LineWidth',1, 'DisplayName','Extended Dynamic');
hold off;
grid on;
ylabel('$v_x$ (m/s)','FontSize',12);

nexttile;
hold on;
plot(t_valid, dx_body(:,2), '-', 'Color', col_actual, 'LineWidth',1.4, 'DisplayName','Measured');
plot(t_kin,v_kin(:,2), ':', 'Color', col_kin, 'LineWidth',1.8, 'DisplayName','Kinematic');
plot(t_ext,v_ext(:,2), '--', 'Color', col_ext, 'LineWidth',1.6, 'DisplayName','Extended Kinematic');
plot(t_dyn, v_dyn(:,2), '-.', 'Color', col_dyn, 'LineWidth',1.3, 'DisplayName','Dynamic');
% plot(t_dyn_ext, v_dyn_ext(:,2), 'Color', col_dyn_ext, 'LineWidth',1, 'DisplayName','Extended Dynamic');
hold off;
grid on;
ylabel('$v_y$ (m/s)','FontSize',12);

nexttile;
hold on;
plot(t_valid, dx_body(:,3), '-', 'Color', col_actual, 'LineWidth',1.4, 'DisplayName','Measured');
plot(t_kin,v_kin(:,3), ':', 'Color', col_kin, 'LineWidth',1.8, 'DisplayName','Kinematic');
plot(t_ext,v_ext(:,3), '--', 'Color', col_ext, 'LineWidth',1.6, 'DisplayName','Extended Kinematic');
plot(t_dyn, v_dyn(:,3), '-.', 'Color', col_dyn, 'LineWidth',1.3, 'DisplayName','Dynamic');
% plot(t_dyn_ext, v_dyn_ext(:,3), 'Color', col_dyn_ext, 'LineWidth',1, 'DisplayName','Extended Dynamic');
hold off;
grid on;
xlabel('Time (s)');
ylabel('$\Omega$ (rad/s)','FontSize',12);
legend('Location','southoutside','Orientation','horizontal','Box','off');
% exportgraphics(gcf,'Results/ID/sindyc_turn_vel.pdf','ContentType','vector','BackgroundColor','none');

%% Functions
% Pure kinematic model
function dxdt = kin_ode(t, x, u1, u2, u3, u4)

    x3 = x(3);

    x1dot = 0.00033047*sin(x3) * u1 + 0.0023453*cos(x3) * u1 -0.00087532*sin(x3) * u2 + 0.002374*cos(x3) * u2 + 0.0010968*sin(x3) * u3 + 0.0013889*cos(x3) * u3 -0.00061986*sin(x3) * u4 + 0.0008294*cos(x3) * u4;
    x2dot = 0.0015423*sin(x3) * u1 -0.0008759*cos(x3) * u1 + 0.0018272*sin(x3) * u2 + 0.00069308*cos(x3) * u2 + 0.0021491*sin(x3) * u3 -0.00078103*cos(x3) * u3 + 0.0018469*sin(x3) * u4 + 0.00087432*cos(x3) * u4;
    x3dot = -0.0067258*u1 + 0.0069079*u2 + 0.0065961*u3 -0.0066846*u4;

    dxdt = [x1dot; x2dot; x3dot];
end

% Extended kinematic model
function dxdt = ext_kin_ode(t, x, u1, u2, u3, u4)

    x3 = x(3);

    x1dot = -0.00016598*u1 -0.00044484*u2 + 0.00028542*u3 + 0.00021938*u4 + 0.00052793*sin(x3) * u1 + 0.0022203*cos(x3) * u1 -0.00066344*sin(x3) * u2 + 0.0023484*cos(x3) * u2 + 0.00087885*sin(x3) * u3 + 0.0013426*cos(x3) * u3 -0.00075726*sin(x3) * u4 + 0.00081618*cos(x3) * u4;
    x2dot = 0.0004803*u1 + 0.000149*u2 -0.00029443*u3 -0.00045783*u4 + 0.0013242*sin(x3) * u1 -0.00085305*cos(x3) * u1 + 0.0016074*sin(x3) * u2 + 0.00068381*cos(x3) * u2 + 0.0023134*sin(x3) * u3 -0.00074272*cos(x3) * u3 + 0.0021253*sin(x3) * u4 + 0.00088697*cos(x3) * u4;
    x3dot = -0.0043797*u1 + 0.008181*u2 + 0.0046648*u3 -0.0078721*u4 -7.1607e-05*u1*u1 -7.2158e-05*u1*u2 + 0.00011008*u1*u3 + 1.6506e-05*u1*u4 + 5.1765e-05*u2*u3 -2.1046e-05*u2*u4 -3.0088e-05*u3*u3 + 2.7602e-05*u4*u4;
    
    
    dxdt = [x1dot; x2dot; x3dot];
end

% Pure dynamic model
function dzdt = dyn_ode(t, z, u1, u2, u3, u4)

    x3 = z(3);
    x1dot = z(4);
    x2dot = z(5);
    x3dot = z(6);

    % transform world frame to body frame velocities
    R = [cos(x3)  -sin(x3);
         sin(x3)   cos(x3)];
    
    v_body  = R.' * [x1dot; x2dot];
    x1dot_b = v_body(1,:);
    x2dot_b = v_body(2,:);
    x3dot_b = x3dot;

    x1ddot = -1.167*cos(x3) * x1dot_b + 1.037*sin(x3) * x2dot_b + 0.0012988*sin(x3) * u1 + 0.002428*cos(x3) * u1 -0.00040258*sin(x3) * u2 + 0.001833*cos(x3) * u2 + 0.00091462*sin(x3) * u3 + 0.0026499*cos(x3) * u3 -0.0025439*sin(x3) * u4 + 0.0028136*cos(x3) * u4;
    x2ddot = -1.3093*sin(x3) * x1dot_b -2.2961*cos(x3) * x2dot_b + 0.0022445*sin(x3) * u1 -0.00090785*cos(x3) * u1 + 0.0032406*sin(x3) * u2 + 0.0013923*cos(x3) * u2 + 0.0027253*sin(x3) * u3 -0.0024104*cos(x3) * u3 + 0.0034616*sin(x3) * u4 + 0.0027898*cos(x3) * u4;
    x3ddot = -1.2711*x3dot_b -0.0078466*u1 + 0.011304*u2 + 0.010035*u3 -0.011928*u4;

    dzdt = [x1dot; x2dot; x3dot; x1ddot; x2ddot; x3ddot];
end

% Extended dynamic model
function dzdt = ext_dyn_ode(t, z, u1, u2, u3, u4)
    
    x3 = z(3);
    x1dot = z(4);
    x2dot = z(5);
    x3dot = z(6);
    
    % transform world frame to body frame velocities
    R = [cos(x3)  -sin(x3);
        sin(x3)   cos(x3)];
    
    v_body  = R.' * [x1dot; x2dot];
    x1dot_b = v_body(1,:);
    x2dot_b = v_body(2,:);
    x3dot_b = x3dot;
    
    x1ddot = -0.15961*x1dot_b + 0.1752*x2dot_b + 0.015965*x3dot_b + 0.00018454*u1 + 0.00071625*u2 -0.00018188*u3 + 0.00070445*u4 + 0.43108*sin(x3) * x1dot_b -0.96714*cos(x3) * x1dot_b + 0.96843*sin(x3) * x2dot_b + 0.90101*cos(x3) * x2dot_b -0.013313*sin(x3) * x3dot_b -0.053099*cos(x3) * x3dot_b + 0.00068643*sin(x3) * u1 + 0.0023883*cos(x3) * u1 -0.0012034*sin(x3) * u2 + 0.0010635*cos(x3) * u2 + 0.00020563*sin(x3) * u3 + 0.003729*cos(x3) * u3 -0.0036816*sin(x3) * u4 + 0.0014916*cos(x3) * u4;
    x2ddot = 0.28693*x1dot_b + 0.36097*x2dot_b -0.077902*x3dot_b -0.00024021*u1 -0.00015759*u2 -0.00026716*u3 -0.001389*u4 -1.5381*sin(x3) * x1dot_b -0.17919*cos(x3) * x1dot_b + 0.55514*sin(x3) * x2dot_b -2.7127*cos(x3) * x2dot_b + 0.073472*sin(x3) * x3dot_b + 0.15112*cos(x3) * x3dot_b + 0.0032342*sin(x3) * u1 + 0.0007047*cos(x3) * u1 + 0.0024338*sin(x3) * u2 + 0.0013083*cos(x3) * u2 + 0.0033999*sin(x3) * u3 -0.0036611*cos(x3) * u3 + 0.0040898*sin(x3) * u4 + 0.0040679*cos(x3) * u4;
    x3ddot = -0.28725*x1dot_b + 1.1423*x2dot_b -1.2562*x3dot_b -0.0064243*u1 + 0.011086*u2 + 0.011442*u3 -0.012438*u4;
    
    dzdt = [x1dot; x2dot; x3dot; x1ddot; x2ddot; x3ddot];
end
