function [A, B] = jacobianos(x0, u0, Planeta, Veiculo, estagio)

    A = zeros(15,15);
    B = zeros(15,3);
    delta = 1e-5;
    
    dx0 = EoM(0,x0,Planeta,Veiculo,u0,estagio);
    
    for i = 1:15
        x_pert = x0;
        x_pert(i) = x_pert(i) + delta;
        dx_pert = EoM(0,x_pert,Planeta,Veiculo,u0,estagio);
    
        A(:,i) = (dx_pert-dx0) / delta;
    end
    
    for j = 1:3
        u_pert = u0;
        u_pert(j) = u_pert(j) + delta;
        dx_pert = EoM(0,x0,Planeta,Veiculo,u_pert,estagio);
    
        B(:,j) = (dx_pert-dx0) / delta;
    end
end