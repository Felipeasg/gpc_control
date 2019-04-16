% 2.4.3 An Example book [1]
%
% [1] Eduardo Fernandez Camacho, Model Predictive Control in the process
% industry

clear;
close all;
clc;

% Model parameters
b0 = 0.4; b1 = 0.6;
a = -0.8;

d = 3;  % Model delay (used to calc GPC Matrices)
N = 3;  % Control Horizon

% control weigths (in this case constant in over all control horizon)
lambda = 0.8;

% Não funciona legal para casos com delay (algum bug), eu acho q é no
% momento de pegar o valor inicial da referencia. Sem delay o valor inicial
% é a ultima medição com delay eu acredito que é o valor predito da planta.
alpha = 0.8;
setpoint_filter_enabled = false;

noise_enabled = true;
noise_percent = 0.1;
        
% contraints 
delta_u_max = 0.8;
delta_u_min = -0.8;

u_max = 1.8;
u_min = -0.8;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

A = [1 a];
B = [b0 b1]; % sem o delay

% delta = (1 - z^-1)
delta_A = conv([1 -1],A);

predictionHorizon = N + d;

control_via_quadprog = true;
control_via_expression = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIND MATRIX OF GPC - G, Gl and F %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% E and F
[E,F]=poly_long_div_v2([1 0 0],delta_A,predictionHorizon);

Ed = E(d+1:end,:);
Fd = F(d+1:end,:);

%E = [zeros(size(E,1),1) E];
%F = [zeros(size(F,1),1) F];
% G Matrix
Gd = [];
for i=1:size(Ed,1)
    %EB = conv(E(i,:),conv(B,[0 1]));
    EdB = conv(Ed(i,:),B);
    
    Gd(end + 1, :) = zeros(1,predictionHorizon-d);
    
    for j = i:-1:1  
        %fprintf('Gd(%d,%d) = EdB(%d,%d)\n',i,(i-j)+1,1,j);
        Gd(i,(i-j)+1) = EdB(1,j);        
    end
end

% % G = [];
% % for i=1:size(E,1)
% %     %EB = conv(E(i,:),conv(B,[0 1]));
% %     EB = conv(E(i,:),B);
% %     
% %     G(end + 1, :) = zeros(1,predictionHorizon);
% %     
% %     for j = i:-1:1        
% %         G(i,(i-j)+1) = EB(1,j);        
% %     end
% % end

% Gl matrix
Gl = [];
% for i=1:size(Ed,1)
%     %EB = conv(E(i,:),conv(B,[0 1]));
%     EdB = conv(Ed(i,:),B);
%     idx_end = find(EdB ~= 0); % index of last non zero element
%     
%     Gl(end+1, :) = EdB(1, idx_end(end)-d:idx_end(end));
% end
for i=1:size(Ed,1)
    %EB = conv(E(i,:),conv(B,[0 1]));
    EdB = conv(Ed(i,:),B);
    shiftedEdB = circshift(EdB, [0 -i]);
    
    Gl(end+1, :) = shiftedEdB(1:d+1);
end


% change some plant parameters to diverge the plant from model
b0p = 0.4; b1p = 0.6; b2p = 0;
ap = -0.8;

I = eye(size(Gd));


y = [
     0; % y(t)
     0; % y(t-1)
     0; % y(t-2)
];

% delta_u = [
%     0;  % delta_u(t)
%     0;  % delta_u(t-1)
%     0;  % delta_u(t-2)
%     0    
% ];
delta_u = zeros(d+2,1);

u = [
    0; % u(t)
    0; % u(t-1)
    0; % u(t-2)
];

% Add delay to u (process with delay)
process_delay = d;
u = [u; zeros(process_delay,1)];

% r = [r(t+1) r(t+2) ... r(t+n)]
r = ones(N,1);
w = zeros(N,1);

disturbance = 0;
noise = 0;

plot_y = [];
plot_u = [];
plot_sp = [];

options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');


K = -1*(inv((Gd'*Gd + lambda*I))*Gd');
fGl = Gl*delta_u(2:end);    
fFd = Fd*[y(1);y(2)];  % Fd*[y(t); y(t-1)]
f = fGl + fFd;



for i=1:100
    
    %%%%%%%%%%%%%%%%%%%%%%
    % setpoint filtering %
    %%%%%%%%%%%%%%%%%%%%%%
    if(setpoint_filter_enabled == true)
        %w(1) = fGl(1) + y(1);
        w(1) = y(1);
        for j=2:size(w,1)
            w(j) = alpha*r(j-1) + (1-alpha)*r(j);
        end
    else
        w = r;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%
    % Predictive Control law %
    %%%%%%%%%%%%%%%%%%%%%%%%%%    
    
    %f1 = Gl*[delta_u(2); delta_u(3); delta_u(4)];    
    fGl = Gl*delta_u(2:end);    
    fFd = Fd*[y(1);y(2)];     
  
    % free response
    f = fGl+fFd;
    
    if control_via_expression == true
        % Here Iam using solution without contraints but I could use de
        % quadprog to solve the problem and find the optmal control action
        delta_control_actions = K*(f - w);

    end
    
    if control_via_quadprog == true
        % https://www.mathworks.com/help/optim/ug/quadprog.html
        H = 2*(Gd'*Gd + lambda*I);
        b_T = 2*(f-w)'*Gd;
        
        lb = delta_u_min*ones(N,1); % delta_u(:) > -0.5        
        ub = delta_u_max*ones(N,1); % delta_u(:) < 0.5
        
        % delta_u(1) + 0 + 0 ... + 0 + u(1) <= u_max
        % delta_u(1) + delta_u(2) + 0 + 0 + ... + u(1) <= u_max
        % ...
        % delta_u(1) + delta_u(2) + ... + delta_u(N) + u(1) <= u_max
        
        % delta_u(1) + 0 + 0 ... + 0 + u(1) >= u_min
        % delta_u(1) + delta_u(2) + 0 + 0 + ... + u(1) >= u_min
        % ...
        % delta_u(1) + delta_u(2) + ... + delta_u(N) + u(1) >= u_min
        A = [tril(ones(N,N),0); tril(ones(N,N),0)*-1];
        b = [(u_max - u(1))*ones(N,1); (u(1) - u_min)*ones(N,1)];
        
        [delta_control_actions,fval,exitflag,output,lambda_q] = ...
        quadprog(H,b_T,A,b,[],[],lb,ub,[],options);

    end
    
    % get just the first control action calculated
    delta_u(1) = delta_control_actions(1);

    u(1) = u(2) + delta_u(1);         
    

    
    % force u(1) to 1 to test process simulation
    %u(1) = 1;
    %%%%%%%%%%%%%%%%%%%%%
    % update old values %
    %%%%%%%%%%%%%%%%%%%%%

     
    for j=size(delta_u):-1:2
        delta_u(j) = delta_u(j-1);
    end

    for j=size(u):-1:2
        u(j) = u(j-1);
    end

    %%%%%%%%%%%%%%%%%%%%%%
    % Process simulation %
    %%%%%%%%%%%%%%%%%%%%%%
    if(noise_enabled == true)
       noise = rand*noise_percent;
    end
    
    if(i == 25)
        r = ones(N,1)*5;
    end

    if(i == 50)
        disturbance = 1;
    end
    
    y(2) = y(1);
    y(1) = b0p*u(end-1) + b1p*u(end) - ap*y(2) + disturbance + noise; %whithout delay (just inerent delay u(t-1)
    
    
    %%%%%%%%%%%%%%%%%
    % Plotting data %
    %%%%%%%%%%%%%%%%%
    plot_y(end + 1) = y(1);
    plot_u(end + 1) = u(1);
    plot_sp(end + 1) = r(1);
    
%     y_predicted = G*control_action + f;
%     figure; stairs([cumsum(control_action) y_predicted]);
%     figure; stairs([plot_y' plot_u' plot_sp']); 
%     close all;

end

figure;
%plot([plot_y' plot_u' plot_sp']);
stairs([plot_y' plot_u' plot_sp']);
legend('y', 'u', 'sp');