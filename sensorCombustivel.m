function [value, isterminal, direction] = sensorCombustivel(~, x, mf)
    m_atual = x(5);
    value = m_atual - mf;
    isterminal = 1;
    direction = 0;
end