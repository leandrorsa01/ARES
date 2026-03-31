function [value, isterminal, direction] = sensorChao(~, x)
    value = x(2); 
    isterminal = 1; 
    direction = -1; 
end