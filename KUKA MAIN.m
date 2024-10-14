clear all
close all
clc

%% KUKA Model

L(1) = Link([0 0.865*1e3 0.4*1e3 pi/2]);
L(2) = Link([0 0 0.92*1e3 0]);
L(3) = Link([0 0 0 pi/2]); 
L(4) = Link([0 1.15*1e3 0 -pi/2]);
L(5) = Link([0 0 0 pi/2]);
L(6) = Link([0 0.470*1e3 0.23*1e3 0]);

KUKA = SerialLink(L, 'name', 'Manipolatore KUKA');

q0 = [-41.799 101.725 -41.451 3.248 -48 -3.641 ]*pi/180; 
T0 = KUKA.fkine(q0); 

px=1253.465;
py=-744.136;
pz=928.036;

%TRASFORMAZIONE COORDINATE MATLAB-MANIPOLATORE
p_mat = [T0.t];
p_kuk = [px py pz]';
teta = 0;
R = [cos(teta) -sin(teta) 0;sin(teta) -cos(teta) 0;0 0 1];
A = [R p_kuk-R*p_mat;0 0 0 1];

%% Input Params

xi=input("Inserisci coordinate iniziali x: ");
yi=input("Inserisci coordinate iniziali y: ");
zi=input("Inserisci coordinate iniziali z: ");
x=input("Inserisci coordinate finali x: ");
y=input("Inserisci coordinate finali y: ");
z=input("Inserisci coordinate finali z: ");
t=input("Inserisci tempo per l'operazione (in s): ");

%% End-effector Trajectory

PEi = inv(A)*[xi+px yi+py zi+pz 1]';            %posizione iniziale end-effector
PEi = PEi(1:3,:);
PEf = inv(A)*[x+px y+py z+pz 1]';               %posizione finale end-effector
PEf = PEf(1:3,:);
VEi = zeros(1,3);                               %velocità iniziale end-effector
VEf = zeros(1,3);                               %velocità finale end-effector
tf  = t;                                        %tempo finale operazione
deltat = 0.01;                                  %intervallo di campionamento

[Pe, Ve] = Pianifica_ee(PEi,PEf,VEi,VEf,tf,deltat); %Pe = posizioni dell'end-effector negli istanti di tempo, Ve = velocità dell'end-effector negli istanti di tempo

T0.t = PEi;
q0 = KUKA.ikunc(T0);

%% Initialization and execution of differential kinematics

[n,l] = size(Ve);                               %assegna ad n il numero delle righe di Ve (numero totale di istanti di tempo campione)
q = q0;                                         %le variabili q partono da q0
qnew = q;                                       %qnew variabile di appoggio per il ciclo
q13 = q(:,1:3);                                 %q13 varibili di giunti trocanta ai primi tre elementi (il manipolatore non deve utilizzare il polso per movimenti lineari)
q13_dot = zeros(1,3);                           %velocità dei giunti
q0_dot = zeros(1,3);                            %velocità iniziale dei giunti


for i = 1:1:n
    q = qnew;
    jacob13 = KUKA.jacob0(q);                   % Calcolo jacobiano
    q13_dot = pinv(jacob13(1:3,1:3))*Ve(i,:)';  % Inversione differenziale q_dot = J' x Ve
    q13 = q13 + q13_dot'*deltat;                % Integrazione discreta q(t+1) = q(t) + deltat*q_dot(t)'
    qnew(:,1:3) = q13; 
    mov(i,:) = qnew;
    T = KUKA.fkine(qnew); 
    PeEff_kuk(i,:) = A*[T.t;1];
end
 

%% Generating G-Code file

PeEff_kuk = PeEff_kuk(:,1:3);
PeEff_kuk = [PeEff_kuk(:,1)-px PeEff_kuk(:,2)-py PeEff_kuk(:,3)-pz];

G_Code(PeEff_kuk, Ve);

%% Simulation of generated movement
plot(KUKA,mov);

