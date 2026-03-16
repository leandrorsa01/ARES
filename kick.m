function [value, isterminal, direction] = kick(~, x, h_kick)
    h_atual = x(2);

    value = h_kick-h_atual;
    isterminal = 1;
    direction = -1;
end