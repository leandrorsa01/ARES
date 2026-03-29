function dxdt = movVerticalPower(~, x, Planeta, Veiculo) 
    h_atual = x(2);
    v_atual = x(3);
    m_atual = x(5);
    g0 = Planeta.g0; Re = Planeta.Re; [rho,~,~,~] = atmosfera_100km(h_atual);
    CD = Veiculo.CD; Aref = Veiculo.Aref;
    Isp = Veiculo.Isp1; m_flux = Veiculo.m_flux1;

    dxdt = zeros(5,1);

    dxdt(1) = 0;
    dxdt(2) = v_atual;
    dxdt(3) = g0*(m_flux*Isp/m_atual - (Re/(Re+h_atual))^2) - (CD*Aref*rho*v_atual^2 /(2*m_atual));
    dxdt(4) = 0;
    dxdt(5) = -m_flux;
end