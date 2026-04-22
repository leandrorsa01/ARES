function Veiculo = loadVeiculo_V2(iteracao)
    try
        tabela = readtable("Veiculos.xlsx", 'VariableNamingRule', 'preserve');
    catch
        error('ARES:FicheiroNaoEncontrado', 'Não foi possível encontrar o ficheiro');
    end

    nomes_variaveis = tabela{:, 1};
    coluna_dados = iteracao + 2;

    if coluna_dados > width(tabela)
        error('ARES:IteracaoInvalida', 'A iteração %d não existe no Excel.', iteracao);
    end

    valores = tabela{:, coluna_dados};

    Veiculo = struct();
    for i = 1:length(nomes_variaveis)
        propriedade = char(nomes_variaveis(i)); 
        Veiculo.(propriedade) = valores(i); 
    end

    if ~isfield(Veiculo, 'C_Na') || isnan(Veiculo.C_Na)
        error('ARES:ModeloIncompativel', ...
            'ERRO CRÍTICO: A iteração V%d não suporta simulação 6-DOF!\nFaltam parâmetros dinâmicos (ex: C_Na, CP, etc.) no ficheiro Excel.', iteracao);
    end
end