function [wopt, y, e, mse] = wiener(x,d,L)
    n = length(x);

    Rx = xcorr(x); % autocorrelation vector
    Rxx = toeplitz(Rx(n:n+L-1)); % autocorrelation matrix [n x n]
    % Rxx = [ Rxx(0)   Rxx(1)   . . . Rxx(n-1);
    %         Rxx(1)   Rxx(0)   . . . Rxx(n-2);
    %         Rxx(2)   Rxx(1)   . . . Rxx(n-3);
    %          ...      ...             ...
    %         Rxx(n-1) Rxx(n-2) . . . Rxx(0)   ]

    ppxd = xcorr(d,x);
    pxd = ppxd(n:n+L-1)'; % crosscorrelation vector [n x 1]
    % pxd = [ Rxd(0) Rxd(1) . . . Rxd(n-1) ]'

    wopt = inv(Rxx)*pxd; % wiener filter (vector) [n x 1]
    y = filter(wopt,1,x);

    e = d - y;
    mse = mean(e.^2);
end