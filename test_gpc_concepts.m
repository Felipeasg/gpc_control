% 2.4.3 An Example book [1]
%
% [1] Eduardo Fernandez Camacho, Model Predictive Control in the process
% industry

clear;
close all;
clc;

A = [1 -0.8];
B = [0.4 0.6];


z = tf('z');

Az = 1 - 0.8*z^-1;
Bz = 0.4 + 0.6*z^-1;

Plant_Tf = Bz/Az;


delta_A = conv([1 -1],A);

% Teste resposta impulso (propria funcao transferencia)
% a resposta fica armazenada em q (e.g. plot(q)
% % n = B;
% % d = A;

% Teste resposta ao degrau (B/A)*(1/(1-z^-1)
% % n = [0 B];
% % d = conv(A,[1 -1]);


% Exemplo pg. 11 [1]
% [1] - Plinio Castrucci, Roberto Moura Sales; Controle Digital
% % n = conv([1 1],[1 0]);
% % d = conv([1 -0.36],[1 -1]);


% Encontrando parametros GPC
% Tenho que sempre garantir que o numero de elementos do numerador seja do
% mesmo tamanho do denominador.
% Neste caso para achar E e F eu uso a divisão 1/((1-z^-1)*A(z-1))
n = [1 0 0]; % Numerador
d = delta_A; % denominador


%[Ec,Fc]=poly_long_div(n,d,5);

predictionHorizon = 3;
[E,F]=poly_long_div_v2(n,d,predictionHorizon);

Gtemp = [];
for i=1:size(E,1)
    Gtemp(end+1, :) = conv(E(i,:),B);
end

% Posso encontrar a matrix G fazendo toeplitz e zerando a parte da matrix
% toeplitz acima da diagonal principal. Isso equivale a fazer um toeplitz
% da respota do modelo ao degrau
%G = tril(toeplitz(Gtemp(end,1:end-1)));

% Ou posso encontrar a matriz G iterativamente como sujerido no livro [1]
% obtendo ela pelos indices da convolução do polinomio E*B
G =[];
for i=1:size(Gtemp,1)
    G(end + 1, :) = zeros(1,predictionHorizon);
    for j = i:-1:1        
        G(i,(i-j)+1) = Gtemp(i,j);        
    end
end

Gl = [];
for i=1:size(G,1)
    Gl(end+1, :) = Gtemp(i, i+1);
end

% Como se trata de polinomios e eu não estou usando matematica simbolica a
% soma deles equivale a uma concatenação, no fim é como se eu quizesse
% encontrar a lei de controle algebrica do MPC (claro para uma situação
% linear sem restrições)
f = [Gl F];
 
% Future control movments - forced response (degree of freedom)
%delta_u_t = [1; 0.8; 0.1];
delta_u_t = zeros(predictionHorizon,1);

u = delta_u_t;

% Past values (free response)
delta_u_t_1 = 0;
%   y(t); y(t-1)
y = [0;     0];

% Model prediction to calc cost function
% QP in Lagrangian form
y_predicted = G*u + F*y + Gl*delta_u_t_1;

u_t = [];
u_t(end+1) = delta_u_t_1;
for j=1:size(delta_u_t,1)
    u_t(end+1) = u_t(end) + delta_u_t(j);
end

%plot(y_predicted);

lambda = 0.8;
control_law_first_part = (inv((G'*G + lambda*eye(size(G))))*G' * f);
control_law_sec_part = (-(inv((G'*G + lambda*eye(size(G))))*G'));

% delta_u que minimiza a função de custo
control_law = -1*[control_law_first_part(1,:)  control_law_sec_part(1,:)];

fprintf('control law\n');
fprintf('delta_u(t) = %f*delta_u(t-1) + %f*y(t) + %f*y(t-1) + %f*w(t+1) + %f*w(t+2) + %f*w(t+3)\n' ...
    ,control_law(1), control_law(2), control_law(3), ...
    control_law(4), control_law(5), control_law(6));

y = zeros(1,2);
u = zeros(1,3);


b0 = 0.4; b1 = 0.6;
a = 0.8;

plot_y = [];
plot_u = [];
for i=1:25    
    
    y(1) = b0*u(2) + b1*u(3) + a*y(2);
    
    
    u(1) = 0.442*u(2) + 0.558*u(3) - 1.371*y(1) + 0.805*y(2) + 0.133*1 + 0.286*1 + 0.147*1;
    
    y(2) = y(1);
    
    u(3) = u(2);
    u(2) = u(1);
    
    
    plot_y(end + 1) = y(1);
    plot_u(end + 1) = u(1);
    
end

plot([plot_y' plot_u']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot_y = [];
plot_u = [];
y = zeros(1,2);
u = zeros(1,3);
delta_u = zeros(1,2);

w = [1; 1; 1];
%control_law(1) = -0.558;

alpha = 0.8;

for i=1:25
    
    y(1) = b0*u(2) + b1*u(3) + a*y(2);
    
    delta_u(1) = control_law*[delta_u(2); y(1); y(2); w(1); w(2); w(3)];
    u(1) = u(2) + delta_u(1);
    

    
    plot_y(end + 1) = y(1);
    plot_u(end + 1) = u(1);
    
    y(2) = y(1);
        
    delta_u(2) = delta_u(1);    
        
    u(3) = u(2);
    u(2) = u(1);
% %     u(2) = u(3) + delta_u(2);
% %     
% %     u(3) = u(2);
% %     u(2) = u(3);
end

figure;
plot([plot_y' plot_u']);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

plot_y = [];
plot_u = [];
y = zeros(1,2);
u = zeros(1,3);
delta_u = zeros(1,2);

w = [1; 1; 1];
wf = [0;0;0];
%control_law(1) = -0.558;

alpha = 0.8;

for i=1:25
    
    y(1) = b0*u(2) + b1*u(3) + a*y(2);

    % smoth setpoint
    wf(1) = y(1);
    wf(2) = alpha*w(1) + (1-alpha)*w(2);
    wf(3) = alpha*w(2) + (1-alpha)*w(3);
    
    delta_u(1) = control_law*[delta_u(2); y(1); y(2); wf(1); wf(2); wf(3)];
    u(1) = u(2) + delta_u(1);
    

    
    plot_y(end + 1) = y(1);
    plot_u(end + 1) = u(1);
    
    y(2) = y(1);
        
    delta_u(2) = delta_u(1);    
        
    u(3) = u(2);
    u(2) = u(1);
% %     u(2) = u(3) + delta_u(2);
% %     
% %     u(3) = u(2);
% %     u(2) = u(3);
end

figure;
plot([plot_y' plot_u']);