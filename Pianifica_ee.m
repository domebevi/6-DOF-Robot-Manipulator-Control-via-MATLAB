function [pe,ve] = Pianifica_ee(pei,pef,pei_dot,pef_dot,tf,deltat )

%%Inizializzazione variabili
t = 0:deltat:tf;
pei = pei';
pef = pef';
pei_dot = pei_dot';
pef_dot = pef_dot';


%%Calcolo coeff. equazioni traiettoria
A = [0 0 0 1;0 0 1 0;tf^3 tf^2 tf 1;3*tf^2 2*tf 1 0];
Bx = [pei(1) pei_dot(1) pef(1) pef_dot(1)];
By = [pei(2) pei_dot(2) pef(2) pef_dot(2)];
Bz = [pei(3) pei_dot(3) pef(3) pef_dot(3)];
A = inv(A);
A = A';
Cx = Bx*A;
Cy = By*A;
Cz = Bz*A;

%%Equazioni cubiche di traiettorie e velocità lungo x y z
sx = Cx(1)*t.^3 + Cx(2)*t.^2 + Cx(3)*t + Cx(4);
sx_dot = 3*Cx(1)*t.^2 + 2*Cx(2)*t+Cx(3);

sy = Cy(1)*t.^3 + Cy(2)*t.^2 + Cy(3)*t + Cy(4);
sy_dot = 3*Cy(1)*t.^2 + 2*Cy(2)*t+Cy(3);

sz = Cz(1)*t.^3 + Cz(2)*t.^2 + Cz(3)*t + Cz(4);
sz_dot = 3*Cz(1)*t.^2 + 2*Cz(2)*t+Cz(3);


pe = [sx; sy; sz]';
ve = [sx_dot; sy_dot; sz_dot]';
end

