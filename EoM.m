function dx = EoM(~,x,Planeta,Veiculo,in,estagio)
    h = x(3); q0 = x(4); q1 = x(5); q2 = x(6); q3 = x(7); u = x(8);
    v = x(9); w = x(10); p = x(11); q = x(12); r = x(13); m_RP1 = x(14);
    m_LOX = x(15);
    g = Planeta.g0*(Planeta.Re/(Planeta.Re+h))^2;
    CP = Veiculo.CP; v_total = norm([u,v,w]);
    dy = in(1)*(pi/180); dz = in(2)*(pi/180);

    if estagio == 1
        Isp = Veiculo.Isp1; m_flux = Veiculo.m_flux1;
    elseif estagio == 2
        Isp = Veiculo.Isp2; m_flux = Veiculo.m_flux2;
    else
        error('EoM:noStage', 'Nenhum estágio identificado');
    end
    [m, CG, Ix, Iy, pos_motor] = calcInercial(m_RP1, m_LOX, Veiculo, estagio);

    % Aerodinámica
    if v_total > 0.01
        aoa = atan2(-v, u);
        bb  = asin(w / v_total);

        [rho, ~, ~, ~] = atmosfera_100km(h);
        q_dyn = 0.5 * rho * v_total^2;
        
        D = q_dyn * Veiculo.Aref * Veiculo.CD;
        
        Ly = q_dyn * Veiculo.Aref * Veiculo.C_Na * aoa;
        Lz = q_dyn * Veiculo.Aref * Veiculo.C_Na * bb;
        
        Ax = -D*cos(aoa) + Ly*sin(aoa);
        Ay =  D*sin(aoa) + Ly*cos(aoa);
        Az = -D*sin(bb)  - Lz*cos(bb);
    else
        Ax = 0; Ay = 0; Az = 0;
    end

    
    Tx = Planeta.g0*Isp*m_flux*in(3)*cos(dy)*cos(dz);
    Ty = Planeta.g0*Isp*m_flux*in(3)*cos(dy)*sin(dz);
    Tz = Planeta.g0*Isp*m_flux*in(3)*sin(dy);

    dx = zeros(15,1);

    % Cinemática de Translação
    dx(1) = u*(q0^2+q1^2-q2^2-q3^2)+2*v*(q1*q2-q0*q3)+2*w*(q1*q3+q0*q2);
    dx(2) = 2*u*(q1*q2+q0*q3)+v*(q0^2-q1^2+q2^2-q3^2)+2*w*(q2*q3-q0*q1);
    dx(3) = 2*u*(q1*q3-q0*q2)+2*v*(q2*q3+q0*q1)+w*(q0^2-q1^2-q2^2+q3^2);
    % Cinemática de Rotação
    dx(4) = -0.5*(p*q1+q*q2+r*q3);
    dx(5) = 0.5*(p*q0-q*q3+r*q2);
    dx(6) = 0.5*(p*q3+q*q0-r*q1);
    dx(7) = 0.5*(-p*q2+q*q1+r*q0);
    % Dinâmica de Translação
    dx(8)  = -q*w+r*v-2*g*(q1*q3-q0*q2)+(Tx/m)+(Ax/m);
    dx(9)  = -r*u+p*w-2*g*(q2*q3+q0*q1)+(Ty/m)+(Ay/m);
    dx(10) = -p*v+q*u-g*(q0^2-q1^2-q2^2+q3^2)+(Tz/m)+(Az/m);
    % Dinâmica de Rotação
    dx(11) = 0;
    dx(12) = (-(Ix-Iy)*p*r+(CG-pos_motor)*Tz-(CP-CG)*Az)/Iy;
    dx(13) = (-(Iy-Ix)*p*q-(CG-pos_motor)*Ty+(CP-CG)*Ay)/Iy;
    % Termodinâmica
    dx(14) = -m_flux*in(3)/3.5;
    dx(15) = -(m_flux - m_flux/3.5)*in(3);
end