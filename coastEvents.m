function [value, isterminal, direction] = coastEvents(~,x)
    v_atual=x(1);
    h_atual = x(2);

    value = [v_atual; h_atual - 5000];

    isterminal = [0; 1];

    direction = [-1; -1];
end