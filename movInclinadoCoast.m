function dxdt = movInclinadoCoast(~, x, Planeta)
    h_atual = x(2);
    v_atual = x(3);
    gg_atual = x(4);
    g0 = Planeta.g0; Re = Planeta.Re;

    dxdt = zeros(5,1);

    dxdt(1) = v_atual*cos(gg_atual);
    dxdt(2) = v_atual*sin(gg_atual);
    dxdt(3) = -g0*sin(gg_atual)*(Re/(Re+h_atual))^2;
    dxdt(4) = cos(gg_atual)*(v_atual/(Re+h_atual) - (g0/v_atual)*(Re/(Re+h_atual))^2);
    dxdt(5) = 0;
end