Liftoff.t = 0; Liftoff.h = 0; Liftoff.vel = 0; Liftoff.mach = 0; Liftoff.Q = 0;
Liftoff.gg = deg2rad(90); Liftoff.mass = 28820.9;

pitchOver.t = 11.88; pitchOver.h = 295; pitchOver.vel = 51.2; pitchOver.mach = 0.15; pitchOver.Q = 1560;
pitchOver.gg = deg2rad(-90); pitchOver.mass = 27235.4;

Mach08.t = 45.72; Mach08.h = 5121; Mach08.vel = 256.11; Mach08.mach = 0.8; Mach08.Q = 23840;
Mach08.gg = deg2rad(-72.46); Mach08.mass = 22719.1;

Mach12.t = 58.52; Mach12.h = 8783; Mach12.vel = 365.81; Mach12.mach = 1.2; Mach12.Q = 32070;
Mach12.gg = deg2rad(-63.86); Mach12.mass = 21010.8;

maxQ.t = 64.92; maxQ.h = 11021; maxQ.vel = 430.14; maxQ.mach = 1.46; maxQ.Q = 33660;
maxQ.gg = deg2rad(-59.59); maxQ.mass = 20156.7;

mid1.t = 99.87; mid1.h = 28050; mid1.vel = 938.19; mid1.mach = 3.12; mid1.Q = 10950;
mid1.gg = deg2rad(-40); mid1.mass = 15492.2;

mid2.t = 134.8; mid2.h = 53262; mid2.vel = 1809.45; mid2.mach = 5.54; mid2.Q = 1140;
mid2.gg = deg2rad(-27.93); mid2.mass = 10831.2;

meco.t = 169.79; meco.h = 88696; meco.vel = 3343.49; meco.mach = 12.20; meco.Q = 20;
meco.gg = deg2rad(-21.40); meco.mass = 6161.4;


pontos_nominais = {Liftoff, pitchOver, Mach08, Mach12, maxQ, mid1, mid2, meco};
x0_geral = zeros(15, length(pontos_nominais));
u0_geral = zeros(3, length(pontos_nominais));

RP1_racio = 0.2857;
LOX1_racio = 1 - RP1_racio;

for i = 1:length(pontos_nominais)
    ponto = pontos_nominais{i};
    
    x0_geral(1,i) = 0;
    x0_geral(2,i) = 0;
    x0_geral(3,i) = ponto.h;

    x0_geral(4,i) = cos(ponto.gg/2);
    x0_geral(5,i) = 0;
    x0_geral(6,i) = sin(ponto.gg/2);
    x0_geral(7,i) = 0;