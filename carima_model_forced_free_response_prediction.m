close all;
clear all;


z = tf('z');

noise_enabled = true;
noise_percent = 0.2;
noise = 0;
disturbance = 0;

% Model parameters
b0 = 0.4; b1 = 0.6;
a = -0.8;

ztf = (b0 + b1*z^-1) / (1 - 0.8*z^-1);

d = 0;  % Model delay (used to calc GPC Matrices)
N = 25;  % Control Horizon


A = [1 a+1.2];
B = [b0 b1-0.5]; % sem o delay

% delta = (1 - z^-1)
delta_A = conv([1 -1],A);

predictionHorizon = N + d;


% E and F
[E,F]=poly_long_div_v2([1 0 0],delta_A,predictionHorizon);

Ed = E(d+1:end,:);
Fd = F(d+1:end,:);

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

Gl = [];
for i=1:size(Ed,1)
    %EB = conv(E(i,:),conv(B,[0 1]));
    EdB = conv(Ed(i,:),B);
    shiftedEdB = circshift(EdB, [0 -i]);
    
    Gl(end+1, :) = shiftedEdB(1:d+1);
end

y = zeros(2,1);
y(1) = 0; y(2) = 0;

delta_u = zeros(d+2,1);
delta_u(1) = 0; delta_u(2) = 0;

u = [
    0; % u(t)
    0; % u(t-1)
    0; % u(t-2)
];

% Add delay to u (process with delay)
process_delay = d;
u = [u; zeros(process_delay,1)];


fGl = Gl*delta_u(2:end);    
fFd = Fd*[y(1);y(2)];     


% free response
f = fGl+fFd;

delta_control_action = zeros(N,1);
%delta_control_action = [1; zeros(N-1,1)];


y_predicted = Gd*delta_control_action + f;

% % figure;
% % stairs(fFd);
% % figure;
% % stairs(fGl);
% % figure;
% % stairs(y_predicted);
% % 
% % figure;
% % step(ztf);

plot_y = [];
plot_yp = [];
plot_u = [];
plot_sp = [];
for i=1:200
    
    if i == 10
        delta_u(1) = 1;
        delta_control_action(1) = delta_u(1);
        
        u(1) = u(2) + delta_u(1);  
    else
        delta_u(1) = 0;
        delta_control_action(1) = delta_u(1);
    end
    
    


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

    if(i == 70)
        disturbance = -2;
    end
    
    y(2) = y(1);
    y(1) = b0*u(end-1) + b1*u(end) - a*y(2) + disturbance + noise; %whithout delay (just inerent delay u(t-1)
    
    
    fGl = Gl*delta_u(2:end);    
    fFd = Fd*[y(1);y(2)];     


    % free response
    f = fGl+fFd;   

    y_predicted = Gd*delta_control_action + f;
    %%%%%%%%%%%%%%%%%
    % Plotting data %
    %%%%%%%%%%%%%%%%%
    plot_y(end + 1) = y(1);
    plot_yp(end + 1) = y_predicted(1);
    plot_u(end + 1) = u(1);
%     plot_sp(end + 1) = r(1);
    
%     y_predicted = G*control_action + f;
%     figure; stairs([cumsum(control_action) y_predicted]);
%     figure; stairs([plot_y' plot_u' plot_sp']); 
%     close all;

end

% % plot_yp(end) = 0;
% % plot_yp= circshift(plot_yp, [0 1]);


figure; plot([plot_y' plot_yp']); legend('y', 'ŷ');
%figure; plot(plot_yp);