tabela_CD = readmatrix('CD.xlsx');
tabela_CN = readmatrix('CN.xlsx');

machVector = tabela_CD(:,1);
cdVector = tabela_CD(:,2);

alphaVector = deg2rad(tabela_CN(:,1));
cnVector = tabela_CN(:,2);