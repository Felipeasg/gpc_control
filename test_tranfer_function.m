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

A = [1 a];
B = [b0 b1];

lambda = 1;

alpha = 0.8;
setpoint_filter_enabled = false;

noise_enabled = false;
noise_percent = 0.2;
        
delta_A = conv([1 -1],A);

predictionHorizon = 9;

control_via_quadprog = false;
control_via_expression = true;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FIND MATRIX OF GPC - G, Gl and F %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% E and F
[E,F]=poly_long_div_v2([1 0 0],delta_A,predictionHorizon);

%E = [zeros(size(E,1),1) E];
%F = [zeros(size(F,1),1) F];
% G Matrix
G = [];
for i=1:size(E,1)
    %EB = conv(E(i,:),conv(B,[0 1]));
    EB = conv(E(i,:),B);
    
    G(end + 1, :) = zeros(1,predictionHorizon);
    
    for j = i:-1:1        
        G(i,(i-j)+1) = EB(1,j);        
    end
end

% Gl matrix
Gl = [];
for i=1:size(E,1)
    %EB = conv(E(i,:),conv(B,[0 1]));
    EB = conv(E(i,:),B);
    
    Gl(end+1, :) = EB(1, i+1);
end

% change some plant parameters to diverge the plant from model
b0p = 0.4; b1p = 0.6; b2p = 0;
ap = -0.8;

I = eye(size(G));


y = [
     0; % y(t)
     0; % y(t-1)
     0; % y(t-2)
];

delta_u = [
    0; % delta_u(t)
    0;  % delta_u(t-1)
    0
];

u = [
    0; % u(t)
    0; % u(t-1)
    0; % u(t-2)
    0;
    0
];

% Add one delay to u (process with delay)
process_delay = 0;
u = [u; zeros(process_delay,1)];

% r = [r(t+1) r(t+2) ... r(t+n)]
r = ones(predictionHorizon,1);
w = zeros(predictionHorizon,1);

disturbance = 0;
noise = 0;

plot_y = [];
plot_u = [];
plot_sp = [];

options = optimoptions('quadprog',...
    'Algorithm','interior-point-convex','Display','off');
lb = 0; % delta_u >  0;
ub = 100;


y_predicted = zeros(predictionHorizon,1);
for i=1:50  
    


    %%%%%%%%%%%%%%%%%%%%%%
    % setpoint filtering %
    %%%%%%%%%%%%%%%%%%%%%%
    if(setpoint_filter_enabled == true)
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
    
    f1 = Gl*delta_u(2);    
    %f2 = F*[y(1);y(2)]; 
    f2 = F*[y_predicted(1); y(1)];
    %f2 = F*[y_predicted(2); y_predicted(1)];
    
    % free response
    f = f1+f2;
    
    if control_via_expression == true
        % Here Iam using solution without contraints but I could use de
        % quadprog to solve the problem and find the optmal control action
        control_action = -1*((inv((G'*G + lambda*I))*G')*(f - w));

    end
    
    if control_via_quadprog == true
        % https://www.mathworks.com/help/optim/ug/quadprog.html
        H = 2*(G'*G + lambda*I);
        b_T = 2*(f-w)'*G;
        [control_action,fval,exitflag,output,lambda_q] = ...
        quadprog(H,b_T,[],[],[],[],[],[],[],options);

    end
    
    % get just the first control action calculated
    delta_u(1) = control_action(1);

    u(1) = u(2) + delta_u(1);
    y_predicted = G*control_action + f;  
    
    %y_predicted = G*control_action + f;
    %figure; stairs([cumsum(control_action) y_predicted]);
    %figure; stairs([plot_y' plot_u' plot_sp']); 
    %close all;
    
    % force u(1) to 1 to test process simulation
    %u(1) = 1;
    %%%%%%%%%%%%%%%%%%%%%
    % update old values %
    %%%%%%%%%%%%%%%%%%%%%

    delta_u(3) = delta_u(2);
    delta_u(2) = delta_u(1);    

    u(5) = u(4);
    u(4) = u(3);    
    u(3) = u(2);
    u(2) = u(1);    
    
    %%%%%%%%%%%%%%%%%%%%%%
    % Process simulation %
    %%%%%%%%%%%%%%%%%%%%%%
    
    y(2) = y(1);
    %y(1) = b0p*u(2) + b1p*u(3) - ap*y(2); %whithout delay (just inerent delay u(t-1)
    y(1) = b0p*u(3) + b1p*u(4) - ap*y(2);  % with 1 sample delay
    %y(1) = b0p*u(4) + b1p*u(5) - ap*y(2);  % with 2 sample delay
    
    
    
    %%%%%%%%%%%%%%%%%
    % Plotting data %
    %%%%%%%%%%%%%%%%%
    plot_y(end + 1) = y(1);
    plot_u(end + 1) = u(1);
    plot_sp(end + 1) = r(1);
end

figure;
stairs([plot_y' plot_u' plot_sp']);