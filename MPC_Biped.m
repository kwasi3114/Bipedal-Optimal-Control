%the main structure of this program comes from the github repository
%that was used for much of the simulation and control for this project

%following the tutorial in the repo, I wrote this program and modified
%some of the parameters of the acutal MPC controller

%furthermore, usage of this algorithm depends on the dependencies
%detailed in the readme and setup program of the repository

g = 9.8;         % Gravity
zc = 0.18;       % Center of Mass Height (constant)
Ts = 1e-2;       % Sample Time 

% Define state space matrices
A = [0,1,0;0,0,1;0,0,0];
B = [0;0;1];
C = [1,0,-zc/g; 1,0,0; 0,1,0];      % zmp, pos, vel
D = [0;0;0];

% Create state space representation
lip_x = ss(A,B,C,D);
lip_y = ss(A,B,C,D);

% Specify names
lip_x.InputName = 'xddd';
lip_x.StateName = {'x', 'xd', 'xdd'};
lip_x.OutputName = {'px', 'x', 'xd'};
lip_y.InputName = 'yddd';
lip_y.StateName = {'y', 'yd', 'ydd'};
lip_y.OutputName = {'py', 'y', 'yd'};

% Discretize plant
lip_x_d = c2d(lip_x,Ts);
lip_y_d = c2d(lip_y,Ts);
disp(lip_x_d);

% Assign measured outputs and unmeasured outputs
lip_x_d = setmpcsignals(lip_x_d, 'MeasuredOutputs', 1, 'UnmeasuredOutputs',[2 3]);
lip_y_d = setmpcsignals(lip_y_d, 'MeasuredOutputs', 1, 'UnmeasuredOutputs',[2 3]);

% Step parameters
stepLength = 0.04;
stepWidth = 0.05;
stepTime = 0.5;

% MPC parameters
predictionHorizon = stepTime/Ts;
numOutputs = 3;
initialState = [0;0;0];

% Create MPC controller object with sample time
mpc1 = mpc(lip_y_d, Ts);
% Specify prediction horizon
mpc1.PredictionHorizon = predictionHorizon;
% Specify control horizon
mpc1.ControlHorizon = predictionHorizon;
% Specify nominal values for inputs and outputs
mpc1.Model.Nominal.U = 0;
mpc1.Model.Nominal.Y = [0;0;0];
% Specify weights
mpc1.Weights.MV = 0.05;
mpc1.Weights.MVRate = 0.1;
mpc1.Weights.OV = [2 0.5 0.5];
mpc1.Weights.ECR = 50000;
