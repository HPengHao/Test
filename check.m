clear;
close all;
filename = 'Test/one-motor/no-disturbance.csv';
train_data = csvread(filename, 2, 0);  
% states [x y z roll pitch yaw vx vy vz p q r]
% output = states
% input [m1 m2 m3 m4]
NX = 12;    
NY = 12;
NU = 4;

%reset start time
train_data(:,1) = train_data(:,1) - train_data(1,1);
%trim data (remove unnecessary parts with starting point (s) and end point (s)
    sp = 10; % starting skip (s)
    ep = 20; % end skip (s)
    isp = find(train_data(:,1) * 1e-6 > sp, 1);
    iep = find(train_data(:,1) > train_data(end,1) - ep * 1e6, 1);
    train_data = train_data(isp:iep, :);
    
raw_timestamps = train_data(:,1) * 1e-6;    %(s)    time vector
raw_states = train_data(:,2:13);            % array of state vector: 12 states
% converting: motor_input = (pwm - 1100)/900 
% motor signal [0..1] --> motor speed (PWM servo) [0...2000]
raw_motors = (train_data(:,14:17)-1100)/900;   %(((0.5595*1000)+1000)-1100)/900=0.51
raw_motors(raw_motors<0) = 0;
raw_motors(raw_motors>1) = 1;
raw_N = size(train_data,1);                 % size of samples

%%%Preprocessing ================================

%========= to NWU ==============
% raw_states(:,2) = -raw_states(:,2);
% raw_states(:,3) = -raw_states(:,3);
% raw_states(:,5) = -raw_states(:,5);
% raw_states(:,6) = -raw_states(:,6);
% raw_states(:,8) = -raw_states(:,8);
% raw_states(:,9) = -raw_states(:,9);
% raw_states(:,11) = -raw_states(:,11);
% raw_states(:,12) = -raw_states(:,12);

%========== to ENU ==============
% x <-> y, vx <-> vy
temp = raw_states(:,1); 
raw_states(:,1) = raw_states(:,2);
raw_states(:,2) = temp;
temp = raw_states(:,7); 
raw_states(:,7) = raw_states(:,8);
raw_states(:,8) = temp;

% z <-> -z, vz <-> -vz
raw_states(:,3) = -raw_states(:,3);
raw_states(:,9) = -raw_states(:,9);

% pitch <-> -pitch yaw = (-yaw + pi/2)
raw_states(:,5) = -raw_states(:,5);
raw_states(:,6) = mod(-raw_states(:,6)+pi/2, 2*pi);

% q <-> -q r <-> -r
raw_states(:,11) = -raw_states(:,11);
raw_states(:,12) = -raw_states(:,12);



%%resample (for uniform sampling time)
desiredFs = 400; %(default 400Hz)
Ts = 1/desiredFs;
[res_states, res_timestamps] = resample(raw_states,raw_timestamps,desiredFs);
[res_motors, res_timestamps] = resample(raw_motors,raw_timestamps,desiredFs);
N = size(res_timestamps,1);

%plot (original and sampeled)
% title_name = ["x(North)", "y(West)", "z(up)", "roll", "pitch", "yaw", "vx", "vy", "vz", "p", "q", "r"]; 
title_name = ["x(east)", "y(north)", "z(up)", "roll", "pitch", "yaw", "vx", "vy", "vz", "p", "q", "r"]; 

figure;
for n=1:NY
    subplot(NY/3, 3, n);
    plot(raw_timestamps,raw_states(:,n),'b.-');
    hold on;
    plot(res_timestamps, res_states(:,n),'r-');
    legend('Original','Resampled');
    title(title_name(n));
end

%%convert to column vectors
timestamps = res_timestamps';
states = res_states';
motors = res_motors';
%%%============================================

%% ==============================================================
%% parameters (default)
g = 9.80665;   % gravity acceleration constant (m/s^2)

arm_scale = deg2rad(5000);
yaw_scale = deg2rad(400);

thetas = [deg2rad(45), deg2rad(-135), deg2rad(-45), deg2rad(135)];

m = 1.5;
I_x = 0.015; %16365151e-9; %16365151e-9;      % Inertia (kg*m^2)
I_y = 0.015; %8354114e-9;    %8354114e-9;
I_z = 0.015; %24008439e-9;  %24008439e-9;


% alpha = 9.6;
throttle_hover = 0.51; %0.5595;%0.51; %0.7;    % (%)  (1559-1100)/900=0.51

K_T = 7.21077;   % = 7.2108

K_Q = 0.1047;          % = 6.9813 * 0.015 = 0.1187

%a=d, b=c
a =  0.128364;    %0.7071 * 87.2665 * 0.015 / 7.2108 = 0.1284
b =  0.128364;    
c = 0.128364;    
d =  0.128364;    

%====================================
% test the model implementation
t = timestamps;
x = zeros(NX,N);    x(1:12,1) = states(:,1);
dx = zeros(NX,N);
y = zeros(NY,N);
u = motors;
err = zeros(NX, N);
global frame_height;
frame_height = 0.1;
for n=1:N-1
    dt = t(n+1) - t(n);
    [dx(:,n),y(:,n)] = quadrotor_m(t(n), x(:,n), u(:,n), a,b,c,d, m, I_x, I_y, I_z, K_T, K_Q);
    x(:,n+1) = x(:,n) + dx(:,n) * dt; 
    x(6,n+1) = mod(x(6,n+1), 2*pi);
    %========= on ground check ==========
    if on_ground(x(3, n+1), frame_height)
        x(3, n+1) = frame_height; % z ;
        x(4:5,n+1) = 0; % roll = pitch = 0;
        x(7:8, n+1) = 0; % vx = vy = 0;
        x(10:12, n+1) = 0; %pqr = 0;
        if x(9, n+1) < 0
            x(9, n+1) = 0; %vz = 0;
        end
    end

%     sychronize for the first 10 seconds
    if n * dt < 10
        x(10:12,n+1) = states(10:12,n+1);
        x(4:6,n+1) = states(4:6,n+1);
    end
    
    %k-step ahead estiamtion (sync at every-k loop)
%     k = 3 * desiredFs;
%     if mod(n, k) == 0
%         x(:,n+1) = states(:,n+1);
%     end
    
    err(:, n) = abs(y(:, n) - states(:, n));
end

err_mean = mean(err, 2);
err_max = max(err, [], 2);
err_quants = quantile(err', [0.25, 0.75])';
err_thresh = err_quants(:, 2) + 1.5 * (err_quants(:, 2)-err_quants(:, 1));
figure;
for n=1:NY
    if NY > 1
        subplot(NY/3, 3, n);
    end
    yyaxis left
    plot(timestamps, states(n,:),'r.-');
    hold on;
    plot(t, y(n,:), 'b-');  
    yyaxis right
    area(t, err(n, :), 'FaceAlpha', 0.8, 'EdgeColor', 'none');
    legend('Resampled', 'Model', 'Error');
%     plot(t, err_mean(n, 1)*ones(1, length(t)), 'g');
%     plot(t, err_max(n, 1)*ones(1, length(t)), 'r');
%     plot(t, err_thresh(n, 1)*ones(1, length(t)), 'b');
%     legend('Resampled', 'Model', 'Error', 'Mean\_err', 'Max\_err', 'Thresh\_err');
    title(title_name(n));
end

