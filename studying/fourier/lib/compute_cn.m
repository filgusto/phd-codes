function c_n = compute_cn(f, t, n)

    % creating the modified function for the Fourier transform
    f_cn = @(x) f(x) .* (exp(-1 * n * 2 * pi * x * 1i));

    % computing c_n
    c_n = integral(f_cn, t(1), t(end));
end

