function Veiculo = loadVeiculo(iteracao)
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
end