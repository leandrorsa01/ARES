function [value, isterminal, direction] = sensorApogeu(~, x)
    % O Apogeu ocorre quando a velocidade vertical é nula, ou seja, 
    % quando o Path Angle (gamma) cruza o zero.
    value = x(4);   
    
    isterminal = 1; % 1 = PARAR o ode45 imediatamente!
    direction = -1; % Só deteta quando o gamma passa de positivo (a subir) para negativo (a descer)
end