close all;
clc;
clear all;

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
t_valid = linspace(0, tfinal, size(u,1));
dt  = t_valid(1,2);

x = [odom_x odom_y yaw];
nx = size(x,2);
nu = size(u,2);

dx_body = [odom_vx odom_vy odom_az];
dx_world = zeros(size(dx_body,1),nx);
dx_world(:,3) = dx_body(:,3); % Angular velocity remains the same


% Interpolate PWM signals for ODE
u1_interp = @(t) interp1(t_valid, u(:, 1), t, 'previous');
u2_interp = @(t) interp1(t_valid, u(:, 2), t, 'previous');
u3_interp = @(t) interp1(t_valid, u(:, 3), t, 'previous');
u4_interp = @(t) interp1(t_valid, u(:, 4), t, 'previous');

% Initial conditions
x0 = [x(1,1); x(1,2); x(1,3)];

% Time span for the ODE solver
tspan = 0:dt:tfinal;
[t_ext, x_ext] = ode45(@(t, x) ext_kin_ode(t, x, u1_interp(t), u2_interp(t), u3_interp(t), u4_interp(t)), tspan, x0);

% Translate to body-frame velocities for comparison
dx_ext = zeros(size(x));
for k = 1:length(t_ext)
    dx_ext(k,:) = ext_kin_ode(t_ext(k), x_ext(k,:)', ...
                                u1_interp(t_ext(k)), ...
                                u2_interp(t_ext(k)), ...
                                u3_interp(t_ext(k)), ...
                                u4_interp(t_ext(k)));

    vx_world = dx_ext(k,1);
    vy_world = dx_ext(k,2);
    X = x_ext(k,3);

    dx_ext(k,1) = cos(X)*vx_world + sin(X)*vy_world;
    dx_ext(k,2) = -sin(X)*vx_world + cos(X)*vy_world;

end

%% Estimate maximum continuous time disturbance bound
N = length(t_ext);

% Split Data for training and validation
i_tr = 1:round(0.6*N);
i_val = (i_tr(end)+1):N;

% Enforce same input-rate limit as in NMPC
du = [zeros(1,size(u,2)); diff(u)/dt]; % Finite-difference input rates
du_max = 30;

dx_corr = zeros(size(dx_body));
dly  = zeros(1,3);
lag = zeros(1,3);
gain = ones(1,3);
bias = zeros(1,3);

% Shift the calculated delay on the dataset
shift = @(x,d) (d>=0) * [zeros(d,1); x(1:end-d)];

for i = 1:nx
    % Find delay from training data & shift
    dly(i) = finddelay(dx_ext(i_tr,i), dx_body(i_tr,i));
    dx_ext_dly = shift(dx_ext(:,i), dly(i));

    % 1st order lag fit
    y_tr = dx_body(i_tr,i);
    x_tr = dx_ext_dly(i_tr);
    den = sum((y_tr(1:end-1) - x_tr(2:end)).^2) + eps;
    num = sum((y_tr(2:end)   - x_tr(2:end)) .* (y_tr(1:end-1) - x_tr(2:end)));
    lag(i) = max(0, min(0.9999, num/den));

    % Apply lag
    dx_ext_lag = zeros(N,1); dx_ext_lag(1) = dx_ext_dly(1);
    for k = 2:N
        dx_ext_lag(k) = lag(i)*dx_ext_lag(k-1) + (1-lag(i))*dx_ext_dly(k);
    end

    % Gain and bias fit
    x_fit = dx_ext_lag(i_tr);
    y_fit = dx_body(i_tr,i);
    x_mean = mean(x_fit);
    y_mean = mean(y_fit);

    % Centre around zero
    x_z_mean = x_fit - x_mean;
    y_z_mean = y_fit - y_mean;

    gain(i) = (x_z_mean' * y_z_mean) / max(x_z_mean' * x_z_mean, eps); % Least-squares gradient
    bias(i) = y_mean - gain(i)*x_mean;

    % Corrected velocity
    dx_corr(:,i) = gain(i)*dx_ext_lag + bias(i);
end

w = dx_body(i_val,:) - dx_corr(i_val,:); % Disturbance estimate on validation

du_inf = max(abs(du(i_val,:)), [], 2); % Maximum from all PWM signals per time sample
A = [ones(size(du_inf)) du_inf]; % Linear regression
c = zeros(1,3); 
m = zeros(1,3);

for i = 1:nx
    X = A \ abs(w(:,i)); % Least-squares
    c(i) = max(0, X(1));
    m(i) = max(0, X(2));
end
w_max = m * du_max + c

%% Functions
% Extended kinematic model
function dxdt = ext_kin_ode(t, x, u1, u2, u3, u4)

    x3 = x(3);
    
    x1dot = -0.00016598*u1 -0.00044484*u2 + 0.00028542*u3 + 0.00021938*u4 + 0.00052793*sin(x3) * u1 + 0.0022203*cos(x3) * u1 -0.00066344*sin(x3) * u2 + 0.0023484*cos(x3) * u2 + 0.00087885*sin(x3) * u3 + 0.0013426*cos(x3) * u3 -0.00075726*sin(x3) * u4 + 0.00081618*cos(x3) * u4;
    x2dot = 0.0004803*u1 + 0.000149*u2 -0.00029443*u3 -0.00045783*u4 + 0.0013242*sin(x3) * u1 -0.00085305*cos(x3) * u1 + 0.0016074*sin(x3) * u2 + 0.00068381*cos(x3) * u2 + 0.0023134*sin(x3) * u3 -0.00074272*cos(x3) * u3 + 0.0021253*sin(x3) * u4 + 0.00088697*cos(x3) * u4;
    x3dot = -0.0043797*u1 + 0.008181*u2 + 0.0046648*u3 -0.0078721*u4 -7.1607e-05*u1*u1 -7.2158e-05*u1*u2 + 0.00011008*u1*u3 + 1.6506e-05*u1*u4 + 5.1765e-05*u2*u3 -2.1046e-05*u2*u4 -3.0088e-05*u3*u3 + 2.7602e-05*u4*u4;
    
    dxdt = [x1dot; x2dot; x3dot];
end
