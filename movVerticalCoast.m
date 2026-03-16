function dxdt = movVerticalCoast(~, x, g0, Re)
    v_atual = x(1);
    h_atual = x(2);

    dxdt = zeros(3,1);

    dxdt(1) = -g0*(Re/(Re+h_atual))^2;
    dxdt(2) = v_atual;
    dxdt(3) = 0;
end
