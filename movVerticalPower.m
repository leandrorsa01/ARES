function dxdt = movVerticalPower(~, x, Planeta, Veiculo) 
    h_atual = x(2);
    v_atual = x(3);
    m_atual = x(5);

    dxdt = zeros(5,1);

    dxdt(1) = 0;
    dxdt(2) = v_atual;
    dxdt(3) = Planeta.g0*(Veiculo.m_flux1*Veiculo.Isp1/m_atual - (Planeta.Re/(Planeta.Re+h_atual))^2);
    dxdt(4) = 0;
    dxdt(5) = -Veiculo.m_flux1;
end