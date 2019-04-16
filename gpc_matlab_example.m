

% Specify the plant described in Example 1.8  of 
% [1].
G = tf(9.8*[1 -0.5 6.3],conv([1 0.6565],[1 -0.2366 0.1493]));

% Discretize the plant with sample time of 0.6 seconds.
Ts = 0.6;
Gd = c2d(G, Ts);

% Create a GPC settings structure.
GPCoptions = gpc2mpc;

% Specify the GPC settings described in example 4.11 of 
% [1].
% Hu
GPCoptions.NU = 2; 
% Hp
GPCoptions.N2 = 5; 
% R
GPCoptions.Lam = 0; 
GPCoptions.T = [1 -0.8];

% Convert GPC to an MPC controller.
mpc = gpc2mpc(Gd, GPCoptions);

% Simulate for 50 steps with unmeasured disturbance between 
% steps 26 and 28, and reference signal of 0. 
SimOptions = mpcsimopt(mpc);
SimOptions.UnmeasuredDisturbance = [zeros(25,1); ...
-0.1*ones(3,1); 0];
sim(mpc, 50, 0, SimOptions);
