function [] = G_Code(Pe,Ve)

%  for i = 1 : length(Pe)            % Calcolo del modulo della velocità dell'end effector
%        Ve_mod(i) = norm(Ve(i,:));
%        Ve_mod(i) = 2000;
%  end

% Ee = [Pe Ve_mod'];  % Matrice contenente posizione e modulo della velocità dell'end effector

%Ee = [Pe Ve_mod'];
%Ee1 = [Pe(1,1) Pe(1,2) Pe(1,3) 2000];

fileID = fopen('Kuka.txt', 'w+');     % Creazione del file di testo
formatSpec1 = 'G1 X%f Y%f Z%f\n';
%formatSpec0 = 'G0 X%f Y%f Z%f\n';

formatSpec0 = 'G0 X%f Y%f Z%f F2000\n';
fprintf(fileID, formatSpec0, Pe(1,:));


%fprintf(fileID, formatSpec, Ee1);

for n = 2 : length(Pe)
    fprintf(fileID, formatSpec1, Pe(n,:));
end
fclose('all');

end

