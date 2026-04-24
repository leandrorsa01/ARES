function [value, isterminal, direction] = missao(~,x,Planeta,evento,alvo)
    h = x(3); m_RP1 = x(14); u = x(8); v = x(9); w = x(10);
    q0 = x(4); q1 = x(5); q2 = x(6); q3 = x(7);
    dh = 2*u*(q1*q3-q0*q2)+2*v*(q2*q3+q0*q1)+w*(q0^2-q1^2-q2^2+q3^2);
    v_t = norm([u, v, w]); v_hor = sqrt(v_t^2-dh^2);
    mu = Planeta.g0*Planeta.Re^2; r = Planeta.Re + h;
    energia = v_t^2/2 - mu/r; H = r*v_hor;
    a = -mu / (2 * energia); e = sqrt(1 + (2 * energia * H^2) / (mu^2));
    
    isterminal = 1; direction = -1;
    switch evento
        case 'pitchOver'
            value = alvo - h;
        case 'combustivel'
            value = m_RP1 - alvo;
        case 'apogeuProjetado'
            apogeu = a*(1+e) - Planeta.Re;
            value = (alvo*1000) - apogeu;
        case 'apogeuSensor'
            value = dh;
        case 'circularizacao'
            perigeu = a*(1-e) - Planeta.Re;
            value = (alvo*1000) - perigeu;
        otherwise
            error('ARES:EventoInvalido', 'Evento de missão desconhecido: %s', evento);
    end
end