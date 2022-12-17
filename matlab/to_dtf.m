function [num, denum] = to_dtf(u, y, ts, order)

tfdata = iddata(y, u, ts);
sys = tfest(tfdata, order);
[num, denum] = numden(sys);
