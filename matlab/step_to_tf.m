%function [num, denum] = step_to_tf(u, y, ts, order)

% TESTING DATA
data = readtable('testdata.txt');
u = data.u;
y = data.y;
ts = data.t(2) - data.t(1);
fs = round(1/ts);
order = 2;

% u: Input vector
% y: Output vector
% ts: Sample time
% order: Transfer function order

% Normalize 
[nu, cu, su] = normalize(u);
[ny, cy, sy] = normalize(y);

% Deriative to get impulse response
dnu = diff(nu);
dny = diff(ny);

% FFT of impulse
L = size(dny, 1);
n = 2^nextpow2(L);
dny = [dny zeros(L - n)];
dnu = [dnu zeros(L - n)];
Y = fft(dny, n);
U = fft(dnu, n);

Puu = U .* conj(U);
Puy = Y .* conj(U);

H = Puy / Puu;
%figure(1)
%plot(abs(H))

%figure(2)
ts = data.t(2) - data.t(1);
tfdata = iddata(data.y, data.u, ts);
sys_est = tfest(tfdata, 2);
bode(sys_est)