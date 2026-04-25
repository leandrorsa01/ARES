function [m, CG, Ix, Iy, pos_motor] = calcInercial(m_RP1, m_LOX, Veiculo, estagio)
    R = Veiculo.d / 2;
    
    if estagio == 1
        m_RP1_S1 = m_RP1 - Veiculo.m_RP1_S2;
        m_LOX_S1 = m_LOX - Veiculo.m_LOX_S2;
        
        % CG2
        x_RP1_S2 = Veiculo.base_RP1_S2 + (Veiculo.h_RP1_S2/2);
        x_LOX_S2 = Veiculo.base_LOX_S2 + (Veiculo.h_LOX_S2/2);
        m_S2     = Veiculo.m_RP1_S2 + Veiculo.m_LOX_S2 + Veiculo.ms2 + Veiculo.mPL;
        CG_S2    = (Veiculo.m_RP1_S2*x_RP1_S2 + ...
                    Veiculo.m_LOX_S2*x_LOX_S2 + ...
                    Veiculo.ms2*Veiculo.Xs_S2 + ...
                    Veiculo.mPL*Veiculo.X_PL)/m_S2;
                    
        % CG1
        h_RP1_S1 = Veiculo.h_RP1_S1 * (m_RP1_S1 / Veiculo.m_RP1_S1);
        h_LOX_S1 = Veiculo.h_LOX_S1 * (m_LOX_S1 / Veiculo.m_LOX_S1);
        x_RP1_S1 = Veiculo.base_RP1_S1 + (h_RP1_S1 / 2);
        x_LOX_S1 = Veiculo.base_LOX_S1 + (h_LOX_S1 / 2);
        m_S1     = m_RP1_S1 + m_LOX_S1 + Veiculo.ms1;
        CG_S1    = (m_RP1_S1*x_RP1_S1 + ...
                    m_LOX_S1*x_LOX_S1 + ...
                    Veiculo.ms1*Veiculo.Xs_S1)/m_S1;
        
        % CG TOTAL e Massa Total
        m  = m_S1 + m_S2;
        CG = (m_S1*CG_S1 + m_S2*CG_S2)/m;
        
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
             
        Ix = Ix_ms1 + Ix_RP1_S1 + Ix_LOX_S1 + Ix_ms2 + ...
             Ix_RP1_S2 + Ix_LOX_S2 + Ix_PL;
             
        pos_motor = 0;
        
    elseif estagio == 2
        m_RP1_S2 = m_RP1;
        m_LOX_S2 = m_LOX;
        
        % CG
        h_RP1_S2 = Veiculo.h_RP1_S2 * (m_RP1_S2 / Veiculo.m_RP1_S2);
        h_LOX_S2 = Veiculo.h_LOX_S2 * (m_LOX_S2 / Veiculo.m_LOX_S2);
        x_RP1_S2 = Veiculo.base_RP1_S2 + (h_RP1_S2 / 2);
        x_LOX_S2 = Veiculo.base_LOX_S2 + (h_LOX_S2 / 2);
        m        = m_RP1_S2 + m_LOX_S2 + Veiculo.ms2 + Veiculo.mPL;
        
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
        error('ARES:calcInercias', 'Nenhum estágio identificado no cálculo inercial');
    end
end