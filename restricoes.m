function [value, isterminal, direction] = restricoes(~,x)
    h_atual = x(3); u = x(8); v = x(9); w = x(10);

    value = zeros(3,1);
    isterminal = ones(3,1);
    direction = [-1;-1;-1];

    % 1 - Colisão com o Solo
    value(1) = h_atual - Planeta.h0;

    % 2 - 