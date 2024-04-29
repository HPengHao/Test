clear;
close all;
%filename = 'Test1/RVPLAYER/atk-300-rv-motor.csv';% 8是攻击参数与攻击补偿一样
%filename = 'Test1/one-motor/atk-300-c-2-motor.csv'
%filename = 'Test1/CI/atk-300-ci-2-motor.csv'
filename = 'Test1/atk/atk-4-motor.csv'
test_data = csvread(filename, 2, 0);

reference_motor = 1100;
refer_idx1 = find(test_data(:, 2) >= reference_motor, 1);
refer_idx2 = find(test_data(:, 7) >= reference_motor, 1);
reference_time1 = test_data(refer_idx1, 1);
reference_time2 = test_data(refer_idx2, 1);
isp1 = refer_idx1;
isp2 = refer_idx2;
iep1 = find(test_data(:, 2) >= reference_motor,1,'last') ;
iep2 = find(test_data(:, 7) >= reference_motor,1,'last') ;
raw_t1 = test_data(isp1:iep1, 1)* 1e-6;
raw_motors1 = test_data(isp1:iep1, 2:5);
raw_t2 = test_data(isp2:iep2, 6)* 1e-6;
raw_motors2 = test_data(isp2:iep2, 7:10);
N=2;

figure;
title_name = ["M1:M2-OUT", "M3:M4-OUT"];
for n=1:N
    subplot(N,1 , n);
    plot(raw_t2,raw_motors2(:,2*n-1),'b.-');
    hold on;
    plot(raw_t2, raw_motors2(:,2*n),'r-');
    title(title_name(n));
end
figure;
title_name = ["M1:M2", "M3:M4"];
for n=1:N
    subplot(N,1 , n);
    plot(raw_t1,raw_motors1(:,2*n-1),'b.-');
    hold on;
    plot(raw_t1, raw_motors1(:,2*n),'r-');
    title(title_name(n));
end