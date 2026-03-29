function dxdt = movInclinadoCoast(~, x, Planeta, Veiculo)
    h_atual = x(2);
    v_atual = x(3);
    gg_atual = x(4);
    m_atual = x(5);
    g0 = Planeta.g0; Re = Planeta.Re; [rho,~,~,~] = atmosfera_100km(h_atual);
    CD = Veiculo.CD; Aref = Veiculo.Aref;

    dxdt = zeros(5,1);

    dxdt(1) = v_atual*cos(gg_atual);
    dxdt(2) = v_atual*sin(gg_atual);
    dxdt(3) = -g0*sin(gg_atual)*(Re/(Re+h_atual))^2 - (CD*Aref*rho*v_atual^2 /(2*m_atual));
    dxdt(4) = cos(gg_atual)*(v_atual/(Re+h_atual) - (g0/v_atual)*(Re/(Re+h_atual))^2);
    dxdt(5) = 0;
end