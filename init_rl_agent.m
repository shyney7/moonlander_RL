previousRngState = rng(0,"twister");
run("rl_init_parameters.m")

nObs = 3;  % Number of dimension of the observation space
nAct = 1;   % Number of dimension of the action space


obsInfo = rlNumericSpec([nObs 1],...
    'LowerLimit',[-1 -1 0]',...
    'UpperLimit',[1 1 1]');
obsInfo.Name = 'observations';
obsInfo.Description = 'velocity, altitude, success';
numObservations = obsInfo.Dimension(1);

actInfo = rlFiniteSetSpec({0,1});
actInfo.Name = "actions";
numActions = numel(actInfo.Elements)

env = rlSimulinkEnv("moonlander_man", "moonlander_man/RL Agent", obsInfo, actInfo);

Ts = 0.1;
Tf = 300000

%% State inputh path
initOpts = rlAgentInitializationOptions(NumHiddenUnit=128);
% actor and critic optimizer options
actorOpts = rlOptimizerOptions(LearnRate=4e-4, ...
    GradientThreshold=1);
criticOpts = rlOptimizerOptions(LearnRate=3e-5, ...
    GradientThreshold=1);

% agent options
agentOpts = rlPPOAgentOptions(...
    ExperienceHorizon       = 1e6,...
    ActorOptimizerOptions   = actorOpts,...
    CriticOptimizerOptions  = criticOpts,...
    MiniBatchSize           = 256,...
    NumEpoch                = 40,...
    SampleTime              = Ts,...
    DiscountFactor          = 0.99, ...
    ClipFactor              = 0.2, ... 
    EntropyLossWeight       = 0.01);

% Enable GPU for actor and critic
%agentOpts.Actor.UseDevice = "gpu";
%agentOpts.Critic.UseDevice = "gpu";

rng(0,"twister");
agent = rlPPOAgent(obsInfo, actInfo, initOpts, agentOpts);


%% GPU Setup
%actor.UseDevice = "gpu";
%critic1.UseDevice = "gpu";
%critic2.UseDevice = "gpu";

%% 3.4. Training Options
trainOpts = rlTrainingOptions(...
    MaxEpisodes=10000, ...
    MaxStepsPerEpisode=floor(Tf/Ts), ...
    Verbose=false, ...
    Plots='training-progress', ...
    ScoreAveragingWindowLength=100, ...
    StopTrainingCriteria='AverageReward', ...
    StopTrainingValue=480, ...
    SaveAgentCriteria='EpisodeReward', ...
    SaveAgentValue=400, ...%UseParallel=true, ...
    SimulationStorageType="none");  
%trainOpts.ParallelizationOptions.Mode = "async";

% agent evaluation
evl = rlEvaluator(EvaluationFrequency=100, NumEpisodes=5);

%% logging parallel
Simulink.sdi.enablePCTSupport('local');

rng(0,"twister");

%% 3.5. Train
trainingStats = train(agent,env,trainOpts, Evaluator=evl);
