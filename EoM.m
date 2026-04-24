function dx = EoM(~,x,Planeta,Veiculo,estagio)
    h = x(3); q0 = x(4); q1 = x(5); q2 = x(6); q3 = x(7); u = x(8);
    v = x(9); w = x(10); p = x(11); q = x(12); r = x(13); m_RP1 = x(14);
    m_LOX = x(15);
    dy = 0 * (pi/180); dz = 0 * (pi/180);
    g = Planeta.g0*(Planeta.Re/(Planeta.Re+h))^2;
    R = Veiculo.d / 2; CP = Veiculo.CP;

    if norm([u,v,w]) > 0
        aoa = atan2(v, u); bb = asin(w / norm([u,v,w]));
    else
        aoa = 0; bb = 0;
    end

    if estagio == 1
        Isp = Veiculo.Isp1; m_flux = Veiculo.m_flux1;
        m_RP1_S1 = m_RP1-Veiculo.m_RP1_S2;
        m_LOX_S1 = m_LOX-Veiculo.m_LOX_S2;

        % CG1
        x_RP1_S2 = Veiculo.base_RP1_S2 +(Veiculo.h_RP1_S2/2);
        x_LOX_S2 = Veiculo.base_LOX_S2 +(Veiculo.h_LOX_S2/2);
        m_S2     = Veiculo.m_RP1_S2 + Veiculo.m_LOX_S2 + Veiculo.ms2 + Veiculo.mPL;
        CG_S2    = (Veiculo.m_RP1_S2*x_RP1_S2+ ...
                    Veiculo.m_LOX_S2*x_LOX_S2+ ...
                    Veiculo.ms2*Veiculo.Xs_S2+ ...
                    Veiculo.mPL*Veiculo.X_PL)/m_S2;

        % CG2
        h_RP1_S1 = Veiculo.h_RP1_S1 * (m_RP1_S1 / Veiculo.m_RP1_S1);
        h_LOX_S1 = Veiculo.h_LOX_S1 * (m_LOX_S1 / Veiculo.m_LOX_S1);
        x_RP1_S1 = Veiculo.base_RP1_S1 + (h_RP1_S1 / 2);
        x_LOX_S1 = Veiculo.base_LOX_S1 + (h_LOX_S1 / 2);
        m_S1     = m_RP1_S1+m_LOX_S1+Veiculo.ms1;
        CG_S1    = (m_RP1_S1*x_RP1_S1+ ...
                    m_LOX_S1*x_LOX_S1+ ...
                    Veiculo.ms1*Veiculo.Xs_S1)/m_S1;
        
        % CG TOTAL
        m  = m_S1+m_S2;
        CG = (m_S1*CG_S1+ ...
              m_S2*CG_S2)/m;
        
        % Inércias 2
        Iy_RP1_S2 = (1/12) * Veiculo.m_RP1_S2 * (3*R^2 + Veiculo.h_RP1_S2^2);
        Iy_LOX_S2 = (1/12) * Veiculo.m_LOX_S2 * (3*R^2 + Veiculo.h_LOX_S2^2);
        Iy_ms2    = (1/12) * Veiculo.ms2 * (3*R^2 + Veiculo.h_S2^2);
        Iy_PL     = (1/12) * Veiculo.mPL * (3*0.6^2 + 1.9^2);

        Ix_RP1_S2 = 0.5 * Veiculo.m_RP1_S2 * R^2;
        Ix_LOX_S2 = 0.5 * Veiculo.m_LOX_S2 * R^2;
        Ix_ms2    = Veiculo.ms2 * R^2;
        Ix_PL     = 0.5 * Veiculo.mPL * 0.6^2;
        
        % Inércias 1
        Iy_RP1_S1 = (1/12) * m_RP1_S1 * (3*R^2 + h_RP1_S1^2);
        Iy_LOX_S1 = (1/12) * m_LOX_S1 * (3*R^2 + h_LOX_S1^2);
        Iy_ms1    = (1/12) * Veiculo.ms1 * (3*R^2 + Veiculo.h_S1^2);

        Ix_RP1_S1 = 0.5 * m_RP1_S1 * R^2;
        Ix_LOX_S1 = 0.5 * m_LOX_S1 * R^2;
        Ix_ms1    = Veiculo.ms1 * R^2;
        
        % Inércias Totais
        Iy = ( Iy_ms1    + Veiculo.ms1 * (CG - Veiculo.Xs_S1)^2 ) + ...
             ( Iy_RP1_S1 + m_RP1_S1    * (CG - x_RP1_S1)^2 ) + ...
             ( Iy_LOX_S1 + m_LOX_S1    * (CG - x_LOX_S1)^2 ) + ...
             ( Iy_ms2    + Veiculo.ms2 * (CG - Veiculo.Xs_S2)^2 ) + ...
             ( Iy_RP1_S2 + Veiculo.m_RP1_S2 * (CG - x_RP1_S2)^2 ) + ...
             ( Iy_LOX_S2 + Veiculo.m_LOX_S2 * (CG - x_LOX_S2)^2 ) + ...
             ( Iy_PL     + Veiculo.mPL * (CG - Veiculo.X_PL)^2 );
        Ix =   Ix_ms1    + Ix_RP1_S1 + Ix_LOX_S1 + Ix_ms2 + ...
               Ix_RP1_S2 + Ix_LOX_S2 + Ix_PL;

        pos_motor = 0;
    elseif estagio == 2
        Isp = Veiculo.Isp2; m_flux = Veiculo.m_flux2;
        m_RP1_S2 = m_RP1;
        m_LOX_S2 = m_LOX;

        % CG
        h_RP1_S2 = Veiculo.h_RP1_S2 * (m_RP1_S2 / Veiculo.m_RP1_S2);
        h_LOX_S2 = Veiculo.h_LOX_S1 * (m_LOX_S2 / Veiculo.m_LOX_S2);
        x_RP1_S2 = Veiculo.base_RP1_S2 + (h_RP1_S2 / 2);
        x_LOX_S2 = Veiculo.base_LOX_S2 + (h_LOX_S2 / 2);
        m     = m_RP1_S2 + m_LOX_S2 + Veiculo.ms2 + Veiculo.mPL;
        CG       = (m_RP1_S2*x_RP1_S2 + ...
                    m_LOX_S2*x_LOX_S2 + ...
                    Veiculo.ms2*Veiculo.Xs_S2 + ...
                    Veiculo.mPL*Veiculo.X_PL) / m;

        % Inércias
        Iy_RP1_S2 = (1/12) * m_RP1_S2 * (3*R^2 + h_RP1_S2^2);
        Iy_LOX_S2 = (1/12) * m_LOX_S2 * (3*R^2 + h_LOX_S2^2);
        Iy_ms2    = (1/12) * Veiculo.ms2 * (3*R^2 + Veiculo.h_S2^2);
        Iy_PL     = (1/12) * Veiculo.mPL * (3*0.6^2 + 1.9^2);

        Ix_RP1_S2 = 0.5 * m_RP1_S2 * R^2;
        Ix_LOX_S2 = 0.5 * m_LOX_S2 * R^2;
        Ix_ms2    = Veiculo.ms2 * R^2; 
        Ix_PL     = 0.5 * Veiculo.mPL * 0.6^2;

        Iy = ( Iy_ms2    + Veiculo.ms2 * (CG - Veiculo.Xs_S2)^2 ) + ...
             ( Iy_RP1_S2 + m_RP1_S2 * (CG - x_RP1_S2)^2 ) + ...
             ( Iy_LOX_S2 + m_LOX_S2 * (CG - x_LOX_S2)^2 ) + ...
             ( Iy_PL     + Veiculo.mPL * (CG - Veiculo.X_PL)^2 );
        Ix = Ix_ms2 + Ix_RP1_S2 + Ix_LOX_S2 + Ix_PL;

        pos_motor = Veiculo.base_RP1_S2 - 1;
    else
        error('ARES:noStage', 'Nenhum estágio identificado');
    end

    D = 0; L = 0;
    Ax = -D*cos(aoa)+L*sin(aoa);
    Ay = D*sin(aoa)+L*cos(aoa);
    Az = -D*sin(bb)-L*cos(bb);

    Tx = Planeta.g0*Isp*m_flux*cos(dy)*cos(dz);
    Ty = Planeta.g0*Isp*m_flux*cos(dy)*sin(dz);
    Tz = Planeta.g0*Isp*m_flux*sin(dy);

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
    dx(14) = -m_flux/3.5;
    dx(15) = -(m_flux - m_flux/3.5);
end