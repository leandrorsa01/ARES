function [value, isterminal, direction] = restricoes(~,x,Planeta)
    h = x(3); u = x(8); v = x(9); w = x(10);
    m_RP1 = x(14); m_LOX = x(15);
    aoa_max = 15 * (pi/180);

    value = zeros(3,1);
    isterminal = ones(3,1);
    direction = [-1;-1;-1];

    % 1 - Colisão com o Solo
    value(1) = h - Planeta.h0;

    % 2 - Ângulo de atáque máximo
    aoa = 0;
    if norm([u,v,w]) > 0
        aoa = atan2(v, u);
    end
    value(2) = aoa_max - aoa;

    % 3 - Combustível
    value(3) = min(m_RP1, m_LOX);
end
