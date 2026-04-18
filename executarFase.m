function [t, x, t_evento, x_evento, i_evento] = executarFase(funcao_dinamica, t_inicio, duracao, x_inicial, funcao_evento)
    tspan = [t_inicio, t_inicio + duracao];
    
    if nargin < 5 || isempty(funcao_evento)
        % SEM EVENTOS
        options = odeset('RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
        
        [t, x] = ode45(funcao_dinamica, tspan, x_inicial, options);
        
        t_evento = []; 
        x_evento = []; 
        i_evento = [];
    else
        % COM EVENTOS
        options = odeset('Events', funcao_evento, 'RelTol', 1e-6, 'AbsTol', 1e-9, 'MaxStep', 0.1);
        
        [t, x, t_evento, x_evento, i_evento] = ode45(funcao_dinamica, tspan, x_inicial, options);
    end
end