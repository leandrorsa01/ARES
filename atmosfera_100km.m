function [rho, P, T, a] = atmosfera_100km(h_geom)
    % h_geom: Altitude GEOMÉTRICA em metros (distância real ao solo)
    % Saídas: rho (kg/m3), P (Pa), T (K), a (Vel. Som m/s)

    % Se não passares argumentos, ele faz o gráfico até 100km
    if nargin == 0, h_geom = 0:100:100000; end

    % --- CONSTANTES FÍSICAS ---
    g = 9.80665; R = 287.05; gamma = 1.4;
    P_ref = 101325; T_ref = 288.15;
    Re = 6356766; % Raio da Terra (m) para conversão geopotencial

    % --- CAMADAS ISA EXPANDIDAS (0-100km+) ---
    % hb: Altitudes Geopotenciais base
    hb = [0, 11000, 20000, 32000, 47000, 51000, 71000, 84852, 91000];
    % Lb: Gradientes térmicos (K/m)
    Lb = [-0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002, 0, 0.000912];

    rho = zeros(size(h_geom)); P = zeros(size(h_geom)); 
    T = zeros(size(h_geom));   a = zeros(size(h_geom));

    for i = 1:length(h_geom)
        % 1. CONVERSÃO: Altitude Geométrica -> Geopotencial (H)
        
        H = (Re * h_geom(i)) / (Re + h_geom(i));
        
        Ti = T_ref; Pi = P_ref;
        
        % 2. LOOP PELAS CAMADAS
        for j = 1:length(hb)
            h_base = hb(j); L = Lb(j);
            h_top = H;
            
            % Define o topo da camada atual
            if j < length(hb) && H > hb(j+1)
                h_top = hb(j+1);
            end
            
            % Cálculo da Temperatura
            T_next = Ti + L * (h_top - h_base);
            
            % Cálculo da Pressão
            if L == 0
                Pi = Pi * exp(-g * (h_top - h_base) / (R * Ti));
            else
                Pi = Pi * (T_next / Ti)^(-g / (L * R));
            end
            
            Ti = T_next;
            if H <= h_top, break; end
        end
        
        % 3. RESULTADOS FINAIS
        T(i) = Ti;
        P(i) = Pi;
        rho(i) = Pi / (R * Ti);
        a(i) = sqrt(gamma * R * Ti);

        if h_geom(i) > 100000
            rho(i) = 0;
            P(i) = 0;
        end
    end

    % --- GRÁFICOS AUTOMÁTICOS ---
    if nargin == 0
        figure('Name', 'Atmosfera Nova-C (0-100km)', 'Color', 'w');
        subplot(2,1,1);
        semilogy(h_geom/1000, rho, 'LineWidth', 2); % Escala logarítmica para ver os 100km
        grid on; ylabel('Densidade \rho (kg/m^3)');
        title('Perfil de Densidade (Escala Logarítmica)');
        
        subplot(2,1,2);
        plot(h_geom/1000, T, 'r', 'LineWidth', 2);
        grid on; xlabel('Altitude Geométrica (km)'); ylabel('Temperatura (K)');
        title('Perfil de Temperatura');
    end
end