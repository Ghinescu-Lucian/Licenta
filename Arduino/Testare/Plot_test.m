brat = readtable('Test1_Pitch_UNO1.txt');
T = brat{:,1}; 
T2 = brat{:,2};
plot (T,T2);
title('Roll test 2 stabilization');
hold on
tacam = readtable('Test1_Pitch_UNO2.txt');
T3 = tacam{:,1}; 
T4 = tacam{:,2};
plot (T3,T4);
hold off

