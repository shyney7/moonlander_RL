previousRngState = rng(0,"twister");
run("rl_init_parameters.m")

nObs = 3;  % Number of dimension of the observation space
nAct = 1;   % Number of dimension of the action space


obsInfo = rlNumericSpec([nObs 1],...
    'LowerLimit',[-inf 0 0]',...
    'UpperLimit',[inf inf 1]');
obsInfo.Name = 'observations';
obsInfo.Description = 'velocity, altitude, success';
numObservations = obsInfo.Dimension(1);

actInfo = rlNumericSpec([nAct 1]);
actInfo.Name = 'thrust';
actInfo.LowerLimit = -1;
actInfo.UpperLimit = 1;
numActions = numel(actInfo);

env = rlSimulinkEnv("moonlander_man", "moonlander_man/RL Agent", obsInfo, actInfo);

Ts = 0.1;
Tf = 300000

%% State inputh path
rng(0,"twister");
% Define the network paths.
observationPath = [
    featureInputLayer(nObs,Name="observation")
    concatenationLayer(1,2,Name="concat")
    fullyConnectedLayer(128)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(32)
    reluLayer
    fullyConnectedLayer(1,Name="QValueOutLyr")
    ];
actionPath = featureInputLayer(nAct,Name="action");
% Assemble dlnetwork object for critic
criticNet = dlnetwork;
criticNet = addLayers(criticNet, observationPath);
criticNet = addLayers(criticNet, actionPath);
criticNet = connectLayers(criticNet,"action","concat/in2");
%plot(criticNet)
summary(initialize(criticNet))
critic1 = rlQValueFunction(initialize(criticNet), ...
    obsInfo,actInfo, ...
    ObservationInputNames="observation");
critic2 = rlQValueFunction(initialize(criticNet), ...
    obsInfo,actInfo, ...
    ObservationInputNames="observation");
%% Create Actor
rng(0,"twister");
% Create the actor network layers.
commonPath = [
    featureInputLayer(nObs,Name="observation")
    fullyConnectedLayer(128)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer(Name="commonPath")
    ];
meanPath = [
    fullyConnectedLayer(32,Name="meanFC")
    reluLayer
    fullyConnectedLayer(nAct,Name="actionMean")
    ];
stdPath = [
    fullyConnectedLayer(nAct,Name="stdFC")
    reluLayer
    softplusLayer(Name="actionStd")
    ];
actorNet = dlnetwork;
actorNet = addLayers(actorNet,commonPath);
actorNet = addLayers(actorNet,meanPath);
actorNet = addLayers(actorNet,stdPath);
actorNet = connectLayers(actorNet,"commonPath","meanFC/in");
actorNet = connectLayers(actorNet,"commonPath","stdFC/in");

%View the actor neural network.
%plot(actorNet)
actorNet = initialize(actorNet);
summary(actorNet)
actor = rlContinuousGaussianActor(actorNet, obsInfo, actInfo, ...
    ObservationInputNames="observation", ...
    ActionMeanOutputNames="actionMean", ...
    ActionStandardDeviationOutputNames="actionStd");

%% GPU Setup
actor.UseDevice = "gpu";
critic1.UseDevice = "gpu";
critic2.UseDevice = "gpu";

%% Create Agent Opj
agentOpts = rlSACAgentOptions( ...
    SampleTime             = Ts, ...
    ExperienceBufferLength = 1e6, ...
    NumWarmStartSteps      = 1e3, ...
    MiniBatchSize          = 300, ...
    DiscountFactor=0.99);
agentOpts.EntropyWeightOptions.TargetEntropy = -2;

agentOpts.ActorOptimizerOptions.Algorithm = "adam";
agentOpts.ActorOptimizerOptions.LearnRate = 3e-4;
agentOpts.ActorOptimizerOptions.GradientThreshold = 1;

for ct = 1:2
    agentOpts.CriticOptimizerOptions(ct).Algorithm = "adam";
    agentOpts.CriticOptimizerOptions(ct).LearnRate = 3e-4;
    agentOpts.CriticOptimizerOptions(ct).GradientThreshold = 1;
end

rng(0,"twister");
agent = rlSACAgent(actor,[critic1,critic2],agentOpts);

%% 3.4. Training Options
trainOpts = rlTrainingOptions(...
    MaxEpisodes=10000, ...
    MaxStepsPerEpisode=floor(Tf/Ts), ...
    Verbose=false, ...
    Plots='training-progress', ...
    ScoreAveragingWindowLength=100, ...
    StopTrainingCriteria='AverageReward', ...
    StopTrainingValue=200, ...
    SaveAgentCriteria='EpisodeReward', ...
    SaveAgentValue=10, ...
    UseParallel=true, ...
    SimulationStorageType="none");  
trainOpts.ParallelizationOptions.Mode = "async";

% agent evaluation
evl = rlEvaluator(EvaluationFrequency=100, NumEpisodes=5);

%% logging parallel
Simulink.sdi.enablePCTSupport('local');

rng(0,"twister");

%% 3.5. Train
trainingStats = train(agent,env,trainOpts, Evaluator=evl);
