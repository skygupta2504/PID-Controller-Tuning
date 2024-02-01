# PID-Controller-Tuning
#This is a Matlab Code for tuning the hyper parameter of a PID controller using reinforcement learning . 
#A learingn algorithm called Q-learning Algorithm is used ofr tuning purpose.
#Code:-
% Define the PID controller parameters
Kp = 1;
Ki = 1;
Kd = 1;
% Define the reward func<on
rewardFunc = @(y, r) -abs(y – r);
% Define the state and ac<on spaces
stateSpace = 1:10; % Example state space
ac<onSpace = 1:1:50; % Example ac<on space
% Ini<alize Q-table
Q = zeros(length(stateSpace), length(ac<onSpace));
% Set hyperparameters
Alpha = 0.1; % Learning rate
Gamma = 0.9; % Discount factor
Epsilon = 0.1; % Explora<on rate
% Define the number of episodes and steps per episode
numEpisodes = 1000;
stepsPerEpisode = 100;
% Define the setpoint (desired reference value or target value)
Setpoint = 1;
% Perform Q-learning
For episode = 1:numEpisodes
State = 1; % Ini<al state
For step = 1:stepsPerEpisode
% Choose an ac<on using epsilon-greedy explora<on
If rand < epsilon
ac<onIdx = randperm(length(ac<onSpace), 1);
ac<on = ac<onSpace(ac<onIdx);
else
if state >= 1 && state <= numel(stateSpace) % Check validity of state index
[~, actionIdx] = max(Q(state, :));
Ac<on = ac<onSpace(ac<onIdx);
Else
Disp(“Invalid state!”);
Break;
End
End
% Simulate the system with the selected ac<on and calculate the reward
Y = simulateSystem(Kp, Ki, Kd, ac<on);
Reward = rewardFunc(y, setpoint);
% Update Q-table using Q-learning algorithm
If state >= 1 && state <= numel(stateSpace) % Check validity of state index
[~, ac<onIdx] = max(Q(state, :));
nextState = getNextState(state, ac<on, stateSpace);
if nextState >= 1 && nextState <= numel(stateSpace) % Check validity of next
state index
nextStateMaxQ = max(Q(nextState, :));
Q(state, ac<onIdx) = Q(state, ac<onIdx) + alpha * (reward + gamma *
nextStateMaxQ – Q(state, ac<onIdx));
Else
Disp(“Invalid next state!”);
Break;
End
% Update the state
State = nextState;
Else
Disp(“Invalid state!”);
Break;
End
End
End
% Extract the learned PID controller parameters
[~, maxQIdx] = max(Q(1, :));
Kp_op<mal = ac<onSpace(maxQIdx);
[~, maxQIdx] = max(Q(2, :));
Ki_op<mal = ac<onSpace(maxQIdx);
[~, maxQIdx] = max(Q(3, :));
Kd_op<mal = ac<onSpace(maxQIdx);
% Display the learned PID controller parameters
Disp(“Op<mal PID Controller Parameters:”);
Disp(“Kp: “ + Kp_op<mal);
Disp(“Ki: “ + Ki_op<mal);
Disp(“Kd: “ + Kd_op<mal);
% Func<on to simulate the system with given PID parameters and ac<on
Func<on y = simulateSystem(Kp, Ki, Kd, ac<on)
% Simulate the system and return the output (y)
% based on the provided PID parameters (Kp, Ki, Kd) and ac<on
% Implement your own system simula<on logic here
% Example: Assuming a first-order system with transfer func<on G(s) = 1/(s+1)
% and a unit step input, the output can be computed using the PID control
% signal (ac<on) and the PID parameters (Kp, Ki, Kd)
% Compute the control signal
controlSignal = (Kp + Ki + Kd) * ac<on;
% Simulate the system response
Sys = m(1, [1 1]); % Transfer func<on of the syste
T = 0:0.1:10; % Time vector
U = ones(size(t)); % Unit step input
[~, y] = lsim(sys, u, t, [0 controlSignal]);
% Return the output (y)
Y = y(end); % Assuming steady-state output is of interest
End
% Func<on to determine the next state
Func<on nextState = getNextState(currentState, ac<on, stateSpace)
% Determine the next state based on the current state, ac<on, and state space
% Implement your own state transi<on logic here
% Example: Assuming a discrete state space, the next state can be determined
% based on the current state and ac<on using a simple mapping or equa<on
% Example mapping: nextState = currentState + ac<on;
% Example equa<on: nextState = 2 * currentState + ac<on;
% Ensure the next state remains within the valid range
nextState = currentState + ac<on;
nextState = max(nextState, 1); % Ensure the next state is not less than the minimum
state
nextState = min(nextState, numel(stateSpace)); % Ensure the next state is not
greater than the maximum state
End
