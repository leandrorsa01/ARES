function dxdt = movVerticalPower(~, x, g0, Re, Isp, m_flux) 
    h_atual = x(2);
    v_atual = x(3);
    m_atual = x(5);

    dxdt = zeros(5,1);

    dxdt(1) = 0;
    dxdt(2) = v_atual;
    dxdt(3) = g0*(m_flux*Isp/m_atual - (Re/(Re+h_atual))^2);
    dxdt(4) = 0;
    dxdt(5) = -m_flux;
end