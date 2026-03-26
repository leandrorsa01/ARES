function [value, isterminal, direction] = coastEvents(~,x)
    v_atual=x(3);
    h_atual = x(2);
    gg_atual = x(4);

    value = [v_atual*sin(gg_atual); v_atual*sin(gg_atual); h_atual - 5000];

    isterminal = [0; 0; 1];

    direction = [-1; 1; -1];
end