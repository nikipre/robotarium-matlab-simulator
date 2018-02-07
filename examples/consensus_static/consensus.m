%% Vanilla consensus with a static, undirected topology
%Paul Glotfelter
%3/24/2016

%% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents();

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
% Since we've chosen single-integrator dynamics, the inputs will be linear
% velocities
r = rb.build('NumberOfAgents', N, 'Dynamics', 'SingleIntegrator', ... 
    'CollisionAvoidance', true, 'SaveData', true, 'ShowFigure', true);

%% Experiment constants

% Generate a cyclic graph Laplacian from our handy utilities.  For this
% algorithm, any connected graph will yield consensus
L = cycleGL(N);

% Select the number of iterations for the experiment.  This value is
% arbitrary
iterations = 1000;

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dxi = zeros(2, N);

%Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    % Since we've chosen single-integrator dynamics, the states will be (x,
    % y) points
    xi = r.get_states();
    
    % Initialize velocity to zero for each agent.  This allows us to sum
    %over agent i's neighbors
    dxi = zeros(2, N);
    
    %% Algorithm
    
    for i = 1:N       
        
        % Get the topological neighbors of agent i based on the graph
        %Laplacian L
        neighbors = topological_neighbors(L, i);
        
        % Iterate through agent i's neighbors
        for j = neighbors
            
            % For each neighbor, calculate appropriate consensus term and
            %add it to the total velocity
            dxi(:, i) = dxi(:, i) + (xi(:, j) - xi(:, i));
        end
    end
    
    
    %% Send single-integrator velocities to agents
    
    % Since we've chosen single-integrator dynamics, the inputs will be
    % linear velocities
    r.set_inputs(1:N, dxi);
    
    % Send the previously set velocities to the agents.  This function must be called!
    
    r.step();
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();