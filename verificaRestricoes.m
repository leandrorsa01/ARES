function verificaRestricoes(ie)
    if isempty(ie)
        return;
    end
    
    if any(ie == 1)
        error('ARES:Colisao', 'FALHA CRÍTICA: O foguetão colidiu com o solo!');
    elseif any(ie == 2)
        error('ARES:Aerodinamica', 'FALHA CRÍTICA: Ângulo de ataque máximo excedido (>15°). O foguetão capotou!');
    elseif any(ie == 3)
        error('ARES:Propulsao', 'FALHA CRÍTICA: Esgotamento prematuro de combustível!');
    end
end