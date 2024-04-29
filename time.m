clear;
close all;

% 读取时间数据
time_data = csvread('time/time3.csv');%time1无方案  time2有方案无攻击 time3有方案有攻击

% 绘制时间开销图
figure;
plot(time_data, 'LineWidth', 2);
xlabel('主循环迭代');
ylabel('时间间隔/ms');
title('有防御方案有攻击的时间开销图');
grid on;