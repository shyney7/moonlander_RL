run("rl_init_parameters.m")

obsInfo = rlNumericSpec([7 1],...
    'LowerLimit',[-inf 0 0 -inf -inf -inf -inf]',...
    'UpperLimit',[inf inf inf inf inf inf inf]');
obsInfo.Name = 'observations';
obsInfo.Description = 'velocity, altitude fuel consumed, alt_err, alt_err_dx, vel_err, vel_err_dx';
numObservations = obsInfo.Dimension(1);

actInfo = rlNumericSpec([1 1], 'LowerLimit', 0, 'UpperLimit', 1);
actInfo.Name = 'thrust';
numActions = numel(actInfo);

env = rlSimulinkEnv("moonlander_man", "moonlander_man/RL Agent", obsInfo, actInfo);

Ts = 0.1;
Tf = 30000;

rng(0)

%% State inputh path
actorNet = [
    featureInputLayer(obsInfo.Dimension(1),'Normalization','none','Name','state')
    fullyConnectedLayer(256,'Name','fc1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(128,'Name','fc2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(actInfo.Dimension(1),'Name','fc3')
    tanhLayer('Name','tanh')          % in [‑1,1]
    scalingLayer('Name','ActorScaling','Scale',0.5, 'Bias', 0.5)];  

actorNet = dlnetwork(actorNet);
%plot(actorNet);
actorNet = initialize(actorNet);
actor = rlContinuousDeterministicActor(actorNet,obsInfo,actInfo);

%% Critic: takes (state, action) → Q-value
statePathCritic = [
    featureInputLayer(obsInfo.Dimension(1),'Normalization','none','Name','state')
    fullyConnectedLayer(256,'Name','fc1')
    ];
actionPathCritic = [
    featureInputLayer(actInfo.Dimension(1),'Normalization','none','Name','action')
    fullyConnectedLayer(256,'Name','fc2')];

commonPath = [
    additionLayer(2,'Name','add')
    reluLayer('Name','c_relu2')
    fullyConnectedLayer(128,'Name','c_fc3')
    reluLayer('Name','c_relu3')
    fullyConnectedLayer(1,'Name','qValue')];

criticNet = dlnetwork();
criticNet = addLayers(criticNet, statePathCritic);
criticNet = addLayers(criticNet, actionPathCritic);
criticNet = addLayers(criticNet, commonPath);

% connect layers
criticNet = connectLayers(criticNet, 'fc1', 'add/in1');
criticNet = connectLayers(criticNet, 'fc2', 'add/in2');
%plot(criticNet);

summary(initialize(criticNet));

critic1 = rlQValueFunction(initialize(criticNet), obsInfo, actInfo);
critic2 = rlQValueFunction(initialize(criticNet), obsInfo, actInfo);
%% GPU Setup
actor.UseDevice = "gpu";
critic1.UseDevice = "gpu";
critic2.UseDevice = "gpu";
%% ExplorationModel
%expModel = rl.option.GaussianActionNoise;
%expModel.LowerLimit = 0.0;
%expModel.UpperLimit = 1.0;
%% 3.3. Agent Options
agentOpts = rlTD3AgentOptions(...
    SampleTime=Ts, ...
    DiscountFactor=0.98, ...
    ExperienceBufferLength=200000, ...
    MiniBatchSize=1024);
%agentOpts.NoiseOptions.Variance = 0.1;      % exploration noise
%agentOpts.TargetSmoothFactor = 5e-3;       % soft update

% Critic optimizer options
for idx = 1:2
    agentOpts.CriticOptimizerOptions(idx).LearnRate = 1e-3;
    agentOpts.CriticOptimizerOptions(idx).GradientThreshold = 1;
    agentOpts.CriticOptimizerOptions(idx).L2RegularizationFactor = 1e-3;
end

% Actor optimizer options
agentOpts.ActorOptimizerOptions.LearnRate = 1e-3;
agentOpts.ActorOptimizerOptions.GradientThreshold = 1;
agentOpts.ActorOptimizerOptions.L2RegularizationFactor = 1e-3;

% agentOpts.ExplorationModel.StandardDeviationMin =  0.05;
agentOpts.ExplorationModel.StandardDeviation = 0.1;
% 
% agentOpts.TargetPolicySmoothModel.StandardDeviation = 0.1;

agent = rlTD3Agent(actor,[critic1,critic2],agentOpts);

%% 3.4. Training Options
trainOpts = rlTrainingOptions(...
    MaxEpisodes=10000, ...
    MaxStepsPerEpisode=ceil(Tf/Ts), ...
    Verbose=false, ...
    Plots='training-progress', ...
    StopTrainingCriteria='AverageReward', ...
    StopTrainingValue=2000, ...
    SaveAgentCriteria='AverageReward', ...
    SaveAgentValue=1000, ...
    UseParallel=true);  

evaluator = rlEvaluator(...
    NumEpisodes=3,...
    EvaluationFrequency=10);

%% 3.5. Train
trainingStats = train(agent,env,trainOpts, Evaluator=evaluator);
