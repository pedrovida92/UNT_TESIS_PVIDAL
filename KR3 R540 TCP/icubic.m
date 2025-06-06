function P = icubic(q,t,qd)
% Obtiene los coeficientes de los splines cubicos
% que interpolan los valores q en los instantes t
% con las velocidades de paso qd
% Las velocidades de paso pueden ser especificadas
% o en caso contrario se utiliza la expresión [6.6]
% Retorna un vector con una fila por cada tramo 
% con [ti,tf,a,b,c,d] siendo el polinomio:
% q(t)=a+b(t-ti)+c(t-ti)^2+d(t-ti)^3 para ti<t<tf

n = length(q);
if n~=length(t)
    error('ERROR en icubic: Las dimensiones de q y t deben ser iguales')
end

if nargin ~= 3  % qd no definida. La obtiene según [6.6]
    qd(1) = 0;
    qd(n) = 0;

    for i=2:n-1
        if (sign(q(i)-q(i-1))==sign(q(i+1)-q(i)))...
                ||q(i)==q(i+1)...
                ||q(i-1)==q(i)
            qd(i)=0.5*((q(i+1)-q(i))/(t(i+1)-t(i))+ ...
                +(q(i)-q(i-1))/(t(i)-t(i-1)));
        else
            qd(i)=0;
        end
    end
end

% obtiene los coeficientes de los polinomios
TT = zeros(2,n-1);
a = zeros(1,n-1);
b = zeros(1,n-1);
c = zeros(1,n-1);
d = zeros(1,n-1);

for i=1:n-1
    ti=t(i);
    tii=t(i+1);
    if tii<=ti
        error('ERROR en i_cubico. Los tiempos deben estar ordenados: t(i) debe ser < t(i+1)');
    end

    T=tii-ti;
    TT(:,i)=[ti;tii];
    a(i)=q(i);
    b(i)=qd(i);
    c(i)= 3/T^2*(q(i+1)-q(i)) - 1/T *(qd(i+1)+2*qd(i)); 
    d(i)=-2/T^3*(q(i+1)-q(i)) + 1/T^2*(qd(i+1) +qd(i));
end

P=[TT;a; b; c; d]';


end
