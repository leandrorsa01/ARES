function [value, isterminal, direction] = gestorEventos(t,x,Planeta,evento,alvo)

    [val_Restricao, term_Restricao, dir_Restricao] = restricoes(t,x,Planeta);

    if nargin < 4 || strcmpi(evento, 'nenhum')
        val_missao = 1;     
        term_missao = 0;    
        dir_missao = 0;     
    else
        [val_missao, term_missao, dir_missao] = missao(t, x, Planeta, evento, alvo);
    end

    value = [val_Restricao; val_missao];
    isterminal = [term_Restricao; term_missao];
    direction = [dir_Restricao; dir_missao];
end