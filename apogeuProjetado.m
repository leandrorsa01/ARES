function [value, isterminal, direction] = apogeuProjetado(~,x,Planeta, orbita_km)
    h_atual = x(2);
    v_atual = x(3);
    gg_atual = x(4);
    mu = Planeta.g0 * Planeta.Re^2;
    r = Planeta.Re + h_atual;

    energia = v_atual^2/2 - mu/r;
    H = r * v_atual * cos(gg_atual);

    a = -mu / (2 * energia);
    e = sqrt(1 + (2 * energia * H^2) / (mu^2));

    apogeu = a*(1+e) - Planeta.Re;
    value = apogeu - (orbita_km*1000);
    isterminal = 1;
    direction = 1;
end