clear;
close all;
filename = 'Test/one-motor/h-0.93-motor.csv';
test_data1 = csvread(filename, 2, 0);

reference_motor = 1100;
refer_idx11 = find(test_data1(:, 2) >= reference_motor, 1);
refer_idx21 = find(test_data1(:, 7) >= reference_motor, 1);
reference_time11 = test_data1(refer_idx11, 1);
reference_time21 = test_data1(refer_idx21, 1);
isp11 = refer_idx11;
isp21 = refer_idx21;
iep11 = find(test_data1(:, 2) >= reference_motor,1,'last') ;
iep21 = find(test_data1(:, 7) >= reference_motor,1,'last') ;
raw_t11 = test_data1(isp11:iep11, 1)* 1e-6;
raw_motors11 = test_data1(isp11:iep11, 2:5);
raw_t21 = test_data1(isp21:iep21, 6)* 1e-6;
raw_motors21 = test_data1(isp21:iep21, 7:10);

filename = 'Test/one-motor/no-disturbance-motor.csv';
test_data2 = csvread(filename, 2, 0);

refer_idx12 = find(test_data2(:, 2) >= reference_motor, 1);
refer_idx22 = find(test_data2(:, 7) >= reference_motor, 1);
reference_time12 = test_data2(refer_idx12, 1);
reference_time22 = test_data2(refer_idx22, 1);
isp12 = refer_idx12;
isp22 = refer_idx22;
iep12 = find(test_data2(:, 2) >= reference_motor,1,'last') ;
iep22 = find(test_data2(:, 7) >= reference_motor,1,'last') ;
raw_t12 = test_data2(isp12:iep12, 1)* 1e-6;
raw_motors12 = test_data2(isp12:iep12, 2:5);
raw_t22 = test_data2(isp22:iep22, 6)* 1e-6;
raw_motors22 = test_data2(isp22:iep22, 7:10);

%t1=raw_t12(1,1)-raw_t11(1,1);
%t2=raw_t22(1,1)-raw_t21(1,1);
N=4;
title_name = ["M1-RECORD", "M2-RECORD", "M3-RECORD", "M4-RECORD"];
figure;
for n=1:N
    subplot(N/2,N/2 , n);
    plot(raw_t11,raw_motors11(:,n),'b.-');
    hold on;
    plot(raw_t12-8.0126, raw_motors12(:,n),'r-');
    legend('Attack','None');
    title(title_name(n));
end

title_name = ["M1-OUT", "M2-OUT", "M3-OUT", "M4-OUT"];
figure;
for n=1:N
    subplot(N/2,N/2 , n);
    plot(raw_t21,raw_motors21(:,n),'b.-');
    hold on;
    plot(raw_t22-8.0001, raw_motors22(:,n),'r-');
    legend('Attack','None');
    title(title_name(n));
end