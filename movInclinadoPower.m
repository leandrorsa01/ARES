function dxdt = movInclinadoPower(~, x, g0, Re, Isp, m_flux)
    h_atual = x(2);
    v_atual = x(3);
    gg_atual = x(4);
    m_atual = x(5);

    dxdt = zeros(5,1);

    dxdt(1) = v_atual*cos(gg_atual);
    dxdt(2) = v_atual*sin(gg_atual);
    dxdt(3) = g0*(m_flux*Isp/m_atual - sin(gg_atual)*(Re/(Re+h_atual))^2);
    dxdt(4) = cos(gg_atual)*(v_atual/(Re+h_atual) - (g0/v_atual)*(Re/(Re+h_atual))^2);
    dxdt(5) = -m_flux;
end