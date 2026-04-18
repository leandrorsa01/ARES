function [value, isterminal, direction] = superEvento(~,x,Planeta,evento,alvo)

    h_atual = x(2); v_atual = x(3); gg_atual = x(4); m_atual = x(5);

    % Por defeito:
    isterminal = 1;                 % Sim
    direction = -1;                 % Positivo -> Negativo

    switch evento
        case 'pitchOver'
            value = alvo - h_atual;
        case 'sensorCombustivel'
            value = m_atual - alvo;
        case 'apogeuProjetado'
            mu = Planeta.g0 * Planeta.Re^2;
            r = Planeta.Re + h_atual;

            energia = v_atual^2/2 - mu/r;
            H = r * v_atual * cos(gg_atual);

            a = -mu / (2 * energia);
            e = sqrt(1 + (2 * energia * H^2) / (mu^2));
            apogeu = a*(1+e) - Planeta.Re;

            value = [(alvo*1000) - apogeu; h_atual; m_atual - 10];
            isterminal = [1;1;1];
            direction = [-1; -1;-1];
        case 'sensorApogeu'
            value = [gg_atual; h_atual];
            isterminal = [1; 1];
            direction = [-1; -1];
        case 'perigeuProjetado'
            mu = Planeta.g0 * Planeta.Re^2;
            r = Planeta.Re + h_atual;

            energia = v_atual^2/2 - mu/r;
            H = r * v_atual * cos(gg_atual);

            a = -mu / (2 * energia);
            e = sqrt(1 + (2 * energia * H^2) / (mu^2));
            perigeu = a*(1-e) - Planeta.Re;

            value = [(alvo*1000) - perigeu; h_atual; m_atual - 100];
            isterminal = [1; 1;1];
            direction = [-1; -1;-1];
    end
end