function dxdt = movVerticalPower(~, x, g0, Re, Isp, m_flux) 
    v_atual = x(1);
    h_atual = x(2);
    m_atual = x(3);

    dxdt = zeros(3,1);

    dxdt(1) = g0*(Re/(Re+h_atual))^2*((m_flux*Isp)/m_atual - 1);
    dxdt(2) = v_atual;
    dxdt(3) = -m_flux;
end