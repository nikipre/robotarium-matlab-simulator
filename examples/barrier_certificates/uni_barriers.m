%% Barrier certificates for unicycle-modeled systems
%Paul Glotfelter 
%3/24/2016

%% Setup Robotarium object

% Get Robotarium object used to communicate with the robots/simulator
rb = RobotariumBuilder();

% Get the number of available agents from the Robotarium.  We don't need a
% specific value for this algorithm
N = rb.get_available_agents(); 

% Set the number of agents and whether we would like to save data.  Then,
% build the Robotarium simulator object!
% Since we've chosen unicycle dynamics, the inputs will be linear and
% angular velocities
r = rb.build('NumberOfAgents', N, 'Dynamics', 'Unicycle', ...
'CollisionAvoidance', true, 'ShowFigure', true, 'SaveData', true);

%Run the simulation for a specific number of iterations
iterations = 2000;

%% Set up constants for experiments

% Initialize velocity vector for agents.  Each agent expects a 2 x 1
% velocity vector containing the linear and angular velocity, respectively.
dx = zeros(2, N);

% Distribute the agents into a circle that fits into the Robotarium
% boundaries
xybound = [-0.5, 0.5, -0.3, 0.3];
p_theta = (1:2:2*N)/(2*N)*2*pi;
p_circ = [xybound(2)*cos(p_theta) xybound(2)*cos(p_theta+pi); xybound(4)*sin(p_theta)  xybound(4)*sin(p_theta+pi)];
x_goal = p_circ(:,1:N);
flag = 0; %flag of task completion

lambda = 0.05;
safety = 0.05;

%% Tools to map single-integrator -> unicycle

% Get the tools we need to map from single-integrator
[si_to_uni_dyn, uni_to_si_states] = create_si_to_uni_mapping('ProjectionDistance', lambda);

% Grab a position controller for single-integrator systems
si_pos_controller = create_si_position_controller();

%Iterate for the previously specified number of iterations
for t = 1:iterations
    
    % Retrieve the most recent poses from the Robotarium.  The time delay is
    % approximately 0.033 seconds
    % Since we've chosen unicycle dynamics, the states will be (x, y,
    % theta) poses
    x = r.get_states();
    
    %% Algorithm
  
    % Check if we've reached our destination on the ellipse
    if norm(x_goal-x(1:2, :),1)<0.1
         flag = 1-flag;
    end
    
    % Switch positions on the ellipse
    if flag == 0
        x_goal = p_circ(:,1:N);
    else
        x_goal = p_circ(:,N+1:2*N);
    end
    
    % Convert to single-integrator domain 
    x_int = uni_to_si_states(x);
    
    %Currently in integrator dynamics
    dx = si_pos_controller(x_int, x_goal);
    
    % Threshold velocities for safety
    dxmax = 0.1;
    for i = 1:N
        if norm(dx(:,i)) > dxmax
            dx(:,i) = dx(:,i)/norm(dx(:,i))*dxmax;
        end
    end
    
    % Map to unicycle dynamics
    dx = si_to_uni_dyn(dx, x);      
    
    % Since we've chosen unicycle dynamics, the inputs will be linear and
    % angular velocities
    r.set_inputs(1:N, dx);
    
    % Send the previously set velocities to the agents.  This function must be called!
    r.step();
    
end

% Though we didn't save any data, we still should call r.call_at_scripts_end() after our
% experiment is over!
r.call_at_scripts_end();
