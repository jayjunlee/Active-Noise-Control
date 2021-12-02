% Desired signal generation using Wiener filter

clear all; clc; close all;

fs = 3000;
Ts = 1/fs;
t = 0:Ts:5-Ts;
f = 10;
L = 3000; % wiener filter length

x = square(2*pi*f*t);
% x = randn(1,length(t));
d = sin(2*pi*f*t);

figure(1); 
subplot(311);
plot(t,x,'b'); hold on;
plot(t,d,'r');
legend('x', 'd');
xlabel('time [s]'); ylabel('x, d');
ylim([-1 1]);

subplot(312);
[wopt, y, e, mse] = wiener(x,d,L);
plot(t,y,'k'); hold on;
plot(t,d,'r');
legend('y', 'd');
xlabel('time [s]'); ylabel('y, d');
ylim([-1 1]);

subplot(313);
plot(t,e);
xlabel('time [s]'); ylabel('e');
ylim([-1 1]);

