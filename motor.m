clear;
close all;
filename = 'Test/one-motor/no-disturbance-motor.csv';
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

N=4;
title_name = ["M1", "M2", "M3", "M4"];
figure;
for n=1:N
    subplot(N/2,N/2 , n);
    plot(raw_t1,raw_motors1(:,n),'b.-');
    hold on;
    plot(raw_t2, raw_motors2(:,n),'r-');
    legend('Record','Output');
    title(title_name(n));
end