%   Universidad Nacional de Trujillo - Ingenieria Mecatronica
%   Tema    :   Escribir matrices en documentos csv
%   Autor   :   Pedro Rafael Vidal Arias
%   Fecha   :   06 de Marzo del 2020
%-------------------------------------------------------------------------

function [ W ] = saveAngles (M)

q1 = [M(:,1),rad2deg(M(:,2))];
q2 = [M(:,1),rad2deg(M(:,3))];
q3 = [M(:,1),rad2deg(M(:,4))];
q4 = [M(:,1),rad2deg(M(:,5))];
q5 = [M(:,1),rad2deg(M(:,6))];
q6 = [M(:,1),rad2deg(M(:,7))];
csvwrite('q1.csv',q1);
csvwrite('q2.csv',q2);
csvwrite('q3.csv',q3);
csvwrite('q4.csv',q4);
csvwrite('q5.csv',q5);
csvwrite('q6.csv',q6);
W = 'Angulos enviados';
end