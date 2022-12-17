clear all; clc; close all;

% Create some test data from a transfer function I thought of
%test_sys = tf(1, [10 1000 100000]);
%test_sysd = c2d(test_sys, 0.002);
%test_data = step(test_sysd);

% Import data
data = readtable('testdata.txt');

% Basic plot 
figure(1)
plot(data.t, data.u); hold on
plot(data.t, data.y); hold off

% Estimate transfer function
ts = data.t(2) - data.t(1);
tfdata = iddata(data.y, data.u, ts);
sys = tfest(tfdata, 2);

% Plot step response of estimated transfer function
figure(2)
step(sys)
[x, t] = step(sys);

% Plot root locus of estimated transfer function
figure(3)
rlocus(sys)

% Run FFT and plot power spectrum
N = size(x, 1);
f_nyq = round(1/ts*2);
ft = fft(x, N);
P2 = abs(ft/N);
P1 = P2(1:N/2+1);
P1(2:end-1) = 2*P1(2:end-1);

figure(4)
plot(abs(P1))

