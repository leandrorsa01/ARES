function [value, isterminal, direction] = sensorCombustivel(~, x, mf1)
    m_atual = x(5);
    value = m_atual - mf1;
    isterminal = 1;
    direction = 0;
end