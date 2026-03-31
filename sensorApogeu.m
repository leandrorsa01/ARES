function [value, isterminal, direction] = sensorApogeu(~, x)
    value = [x(4); x(2)];   
    
    isterminal = [1; 1]; 
    direction = [-1; -1];
end