function [value, isterminal, direction] = perigeuProjetado(~,x, Planeta, orbita_km)
    h_atual = x(2);
    v_atual = x(3);
    gg_atual = x(4);
    m_atual = x(5);
    mu = Planeta.g0 * Planeta.Re^2;
    r = Planeta.Re + h_atual;

    energia = v_atual^2/2 - mu/r;
    H = r * v_atual * cos(gg_atual);

    a = -mu / (2 * energia);
    e = sqrt(1 + (2 * energia * H^2) / (mu^2));

    perigeu = a*(1-e) - Planeta.Re;

    value = [perigeu - (orbita_km * 1000); h_atual; m_atual - 100]; 
    isterminal = [1; 1;1]; 
    direction = [1; -1;-1];
end