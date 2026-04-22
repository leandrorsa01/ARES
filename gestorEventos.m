function [value, isterminal, direction] = gestorEventos(t,x,Planeta,Veiculo,evento,alvo)

    [val_Restricao, term_Restricao, dir_Restricao] = restricoes(t,x);

    [val_missao, term_missao, dir_missao] = missao(t,x,Planeta,Veiculo,evento,alvo);

    value = [val_Restricao; val_missao];
    isterminal = [term_Restricao; term_missao];
    direction = [dir_Restricao; dir_missao];
end