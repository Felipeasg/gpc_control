function [E,F, q] = poly_long_div(num,den,PredictionHorizon)

n = num; % Numerador
d = den; % denominador

q = [];
t_d = [];
    
r = n;

E = {};
F = {};

% https://en.wikipedia.org/wiki/Polynomial_long_division
% Euclidian algorithm
for i=1:PredictionHorizon
    
    % Encontra indexes diferentes de zero, para descobrir qual é o elemento que
    % acompanha o termo de maior grau, se for expresso em função de z^-1 sempre sera
    % o indice mais a esquerda
    % ex. [0 0 1 2 3]
    %          ^
    %          '-  Indice de maior grau é o 3
    idx_r = find(r~=0); % polinomio do resto
    idx_d = find(d~=0); % polinomio do divisor
    
    % t é o elemento que acompanha o maior indice do resto dividido pelo
    % elemento que acompanha o maior indice do divisor. No caso de uma
    % transformada z em z^-1, o divisor sempre terá o termo que acompanha o
    % maior grau no primeiro elemento e o valor desse elemento será 1.
    % e.g. 1 + a1*z^-1 + a2*z^-2 + a3*z^-3 ...
    %      [1 a1 a2 a3 ... ]
    % Isso é feito para forcar a eliminação do termo de maior grau na
    % proxima operação para o calculo do resto ( r <- r - t*d )
    t = r(idx_r(1))/d(idx_d(1));
    
    %q = q + t;
    % Acumula os valores de t num array, que nada mais é que do que o
    % Quociente da divisão polinomial
    q(end + 1) = t;

    % Multiplica o valor que acompanha a maior potencia do resto dividido
    % pelo divisor, assim quando eu subtrarir o resto de t*d o termo de
    % maior grau do polinomio vai ser zerado
    t_d = [];
    %t_d = t*d;
    t_d = conv(t,d);
    
    % Caso o indice de maior grau do resto não esteja na posição 1 (idx_r >
    % 1) eu sou obrigado a adicionar um zero no começo do polinomio t_d
    % (que é t*d). Isto é pra garantir a nova posição do polinomio t_d.
    
    if(idx_r > 1)
        for i=1:idx_r-1;
            
            
            % Para adicionar o zero no começo do polinomio eu adiciono zero no fim
            % do array t_d (t_d(end+1) = 0;) e depois rotaciono uma vez
            % para a direita considerando as colunas ([0 1]) pois o vetor
            % t_d é uma tem 1 linha e n colunas (e.g para rotacionar 2
            % vezes na linha seria preciso usar [2 0].
            t_d(end+1) = 0;
            t_d = circshift(t_d,[0 1]);            
        end
        
        % Como eu atualizei o posicionamento do indicies do polinomio t_d
        % eu preciso fazer o tamanho do resto bater com o tamanho do
        % polinomio t_d para eles poderem ser subtraidos, assim eu só
        % adiciono um zero no final dele, e eles ficam do mesmo tamanho.
        r(end+1) = 0;
    end
    
    r = r - t_d;
    
    % GPC Parameters Ej(z^-1) and z^-j*Fj(z^-1)
    % Append na linha
    %E{end + 1} = q;
    %F{end + 1} = r;
    
    % append na coluna    
    E(end + 1, :) = {q};
    F(end + 1, :) = {r};
end


