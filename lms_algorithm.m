% LMS algorithm
% example 1: h = 0.8^k

clear all; clc; close all;

N = 50000;
x = randn(1,N);         % input (reference) signal
x = x - mean(x);
tap = 20;
k = 0:tap-1;
h = 0.8.^k;             % impulse response function                
w = zeros(1,tap);       % LMS filter coefficients (20 taps)
alpha = 0.015;          % gradient descent step size
ww = zeros(20,N-tap-tap);


for n = tap:N-tap
    dd = conv(h,x);   
    yy = conv(w,x);
    d = dd(n+1-tap:n);  % desired signal
    y = yy(n+1-tap:n);  % LMS filtered signal
    e = d - y;
    w = w + alpha*e.*x(n+1-tap:n);  % LMS update equation
    k = n-tap+1;
    ww(:,k) = w';
end

figure(1); clf;
subplot(411);
plot(x); ylabel('x');
subplot(412);
d = conv(x,h);
plot(d(1:N)); ylabel('d');
subplot(413);
y = conv(x,w);
plot(y(1:N)); ylabel('y');
subplot(414);
plot(d, 'r'); hold on;
plot(y, 'b');
plot(d-y,'k'); ylabel('e');
legend('d','y','e');

figure(2); clf;
plot(ww(:,2000),'o-'); hold on;
plot(ww(:,5000),'o-');
plot(ww(:,10000),'o-');
plot(w,'*-');
plot(h,'--');
legend('w 2000', 'w 5000', 'w 10000', 'w final', 'y');

figure(3); clf;
plot(ww');
