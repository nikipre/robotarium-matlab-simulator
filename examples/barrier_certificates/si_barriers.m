%% Single-integrator Barrier Certificate Algorithm
% by Paul Glotfelter 
% 3/24/2016

%% Set up Robotarium object 
% Before starting the algorithm, we need to initialize the Robotarium
% object so that we can communicate with the agents

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents();

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
% Since we've chosen point-controlled dynamics, the inputs will be (x, y)
% points
r = rb.build('NumberOfAgents', N, 'Dynamics', 'PointControlled', ...
'CollisionAvoidance', true, 'ShowFigure', true, 'SaveData', true);

% This is a totally arbitrary number
iterations = 20000;

%% Experiment constants 
% Next, we set up some experiment constants

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

% This code ensures that the agents are initially distributed around an
% ellipse.  
xybound = 2*[-.4, .4, -0.7, 0.7];
p_theta = (1:2:2*N)/(2*N)*2*pi;
p_circ = [xybound(2)*cos(p_theta) xybound(2)*cos(p_theta+pi); xybound(4)*sin(p_theta)  xybound(4)*sin(p_theta+pi)];

x_goal = p_circ(:,1:N);

flag = 0; %flag of task completion

%% Begin the experiment
% This section contains the actual implementation of the barrier
% certificate experiment.

%Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    % Since we've chosen point-controlled dynamics, the states will be (x,
    % y) points
    x = r.get_states();   
    
    %% Algorithm
  
    % Let's make sure we're close enough the the goals
    if norm(x_goal-x,1)<0.08
         flag = 1-flag;
    end
    
    % This code makes the robots switch positions on the ellipse
    if flag == 0
        x_goal = p_circ(:,1:N);
    else
        x_goal = p_circ(:,N+1:2*N);
    end
        
    % Since we've chosen point-controlled dynamics, the inputs will be (x,
    % y) points
    r.set_inputs(1:N, x_goal);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();    
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();

