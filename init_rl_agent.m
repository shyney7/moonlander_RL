run("rl_init_parameters.m")

obsInfo = rlNumericSpec([8 1],...
    'LowerLimit',[0 -inf 0 0 -inf -inf -inf -inf]',...
    'UpperLimit',[inf inf inf 1 inf inf inf inf]');
obsInfo.Name = 'observations';
obsInfo.Description = 'altitude, velocity, fuel consumed, fail boolean, alt_err, alt_err_dx, vel_err, vel_err_dx';
numObservations = obsInfo.Dimension(1);

actInfo = rlNumericSpec([1 1], 'LowerLimit', 0, 'UpperLimit', 1);
actInfo.Name = 'thrust';
numActions = numel(actInfo);

env = rlSimulinkEnv("moonlander_man", "moonlander_man/RL Agent", obsInfo, actInfo);

Ts = 0.1;
Tf = 200;

% Observation path
obsPath = featureInputLayer(obsInfo.Dimension(1), ...
    Name="obsInLyr");

% Action path
actPath = featureInputLayer(actInfo.Dimension(1), ...
    Name="actInLyr");

% Common path
commonPath = [
    concatenationLayer(1,2,Name="concat")
    fullyConnectedLayer(250)
    reluLayer()
    fullyConnectedLayer(150)
    reluLayer()
    fullyConnectedLayer(1,Name="QValue")
    ];

% Create the network object and add the layers
criticNet = dlnetwork();
criticNet = addLayers(criticNet,obsPath);
criticNet = addLayers(criticNet,actPath);
criticNet = addLayers(criticNet,commonPath);

% Connect the layers
criticNet = connectLayers(criticNet, ...
    "obsInLyr","concat/in1");
criticNet = connectLayers(criticNet, ...
    "actInLyr","concat/in2");

% plot(criticNet)

rng(0,"twister");
criticNet = initialize(criticNet);
summary(criticNet)

critic = rlQValueFunction(criticNet, ...
    obsInfo,actInfo, ...
    ObservationInputNames="obsInLyr", ...
    ActionInputNames="actInLyr");

getValue(critic, ...
    {rand(obsInfo.Dimension)}, ...
    {rand(actInfo.Dimension)})
%critic.UseDevice = "gpu";

% Actor
actorNet = [
    featureInputLayer(obsInfo.Dimension(1))
    fullyConnectedLayer(250)
    reluLayer()
    fullyConnectedLayer(150)
    reluLayer()
    fullyConnectedLayer(actInfo.Dimension(1))
    ];

rng(0,"twister");
actorNet = dlnetwork(actorNet);
summary(actorNet)

% plot(actorNet)

actor = rlContinuousDeterministicActor(actorNet,obsInfo,actInfo);
%actor.UseDevice = "gpu";

agent = rlDDPGAgent(actor,critic);

agent.AgentOptions.SampleTime = Ts;
agent.AgentOptions.DiscountFactor = 0.99;
agent.AgentOptions.MiniBatchSize = 128;
agent.AgentOptions.ExperienceBufferLength = 1e6;

actorOpts = rlOptimizerOptions( ...
    LearnRate=1e-4, ...
    GradientThreshold=1);
criticOpts = rlOptimizerOptions( ...
    LearnRate=1e-4, ...
    GradientThreshold=1);
agent.AgentOptions.ActorOptimizerOptions = actorOpts;
agent.AgentOptions.CriticOptimizerOptions = criticOpts;

agent.AgentOptions.NoiseOptions.StandardDeviation = 0.3;
agent.AgentOptions.NoiseOptions.StandardDeviationDecayRate = 1e-5;

% training options maxsteps ceil(Tf/Ts)
trainOpts = rlTrainingOptions(...
    MaxEpisodes=10000, ...
    MaxStepsPerEpisode=2500, ...
    Plots="training-progress", ...
    Verbose=false, ...
    StopTrainingCriteria="AverageReward", ...
    StopTrainingValue=2000, UseParallel=true, ...
    SaveAgentCriteria="EpisodeReward", ...
    SaveAgentValue=1500); %UseParallel=true

% agent evaluator
evl = rlEvaluator(EvaluationFrequency=10,NumEpisodes=5);
%% 

rng(0,"twister");

trainingStats = train(agent,env,trainOpts,Evaluator=evl);
