clear;
close all;





%nonlinear greybox model

a = 0.128364;
b = 0.128364;
c = 0.128364;
d = 0.128364;
m = 1.5;
I_x = 0.015; 
I_y = 0.015; 
I_z = 0.015; 
K_T = 7.21077;
K_Q = 0.10472;

%% Read test data
filename = 'Test/one-motor/atk-300.csv';
test_data = csvread(filename, 2, 0);  
test_data(:, 1) = test_data(:, 1)-test_data(1, 1); % reset start time
%trim data (remove unnecessary parts with starting point (s) and end point (s)
    sp = 1; % starting point (s)
    ep = 270; % end point (s)
    isp = find(test_data(:,1) * 1e-6 > sp, 1);
    iep = find(test_data(:,1) * 1e-6 > ep, 1);
    if isempty(iep)
        iep = size(test_data,1);
    end
    test_data = test_data(isp:iep, :);

NX = 12;    
NY = 12;
NU = 4;
raw_timestamps = test_data(:,1) * 1e-6;    %(s)    time vector
raw_states = test_data(:,2:13);            % array of state vector: 12 states
raw_motors = (test_data(:,14:17)-1100)/900;   %SITL: (((0.5595*1000)+1000)-1100)/900=0.51
N = size(test_data,1);                 % size of samples
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

timestamps = raw_timestamps';
states = raw_states';
motors = raw_motors';


title_name = ["x(east)", "y(north)", "z(up)", "roll", "pitch", "yaw", "vx", "vy", "vz", "p", "q", "r"]; 
% t, x, y, u
t = timestamps;
x = zeros(NX,N);    %x(1:12,1) = states(:,1);    
y = zeros(NY,N);    %model output
u = motors;
dx = zeros(NX,N);
para = zeros(1,N);
global frame_height;
frame_height = 0.1;
for n=1:N-1
    dt = t(n+1) - t(n);
    [dx(:,n),y(:,n)] = quadrotor_m( states(:,n), u(:,n), a,b,c,d, m, I_x, I_y, I_z, K_T, K_Q);
    x(:,n+1) = states(:,n) + dx(:,n) * dt; 
    %para(1:n+1)=(states(7,n+1)-x(7,n+1))*m/K_T/dt/(cos(states(4,n))*sin(states(5,n))*cos(states(6,n)) + sin(states(4,n))*sin(states(6,n)))*900;
    x(6,n+1) = mod(x(6,n+1), 2*pi); % wrap yaw to [0,2pi)
    para(1,n+1)=(states(7,n+1)-x(7,n+1))*m/K_T/dt/(cos(states(4,n))*sin(states(5,n))*cos(states(6,n)) + sin(states(4,n))*sin(states(6,n)))*900;
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
    if n * dt < 10
        x(10:12,n+1) = states(10:12,n+1);
        x(4:6,n+1) = states(4:6,n+1);
    end
    
    
end


figure;

    
    plot(timestamps, para);     %truth
    