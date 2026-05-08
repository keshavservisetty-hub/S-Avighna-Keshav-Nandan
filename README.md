# S-Avighna-Keshav-Nandan
# Quarter-Car Suspension Dynamics: Passive vs. Semi-Active

## Overview
This repository serves as the digital code appendix for the mechanical engineering report detailing the design and simulation of a Modified (Semi-Active) Suspension System. The MATLAB scripts provided here are used to define the mathematical plant parameters, generate the synthesized road profiles, and execute the closed-loop control logic utilized within the Simulink environment.

## Repository Contents

This repository contains three primary MATLAB scripts corresponding to the phases of the simulation:

1. passive_quarter_car_script
% Quarter Car Suspension Parameters
m = 350;    % Mass (kg)
c = 18000;  % Spring Stiffness (N/m) 
b = 1000;   % Damping Coefficient (N.s/m) 

%% road profile
t = (0:0.001:10)';           % Time vector 
car_speed_kmh = 10;         % Speed
car_speed_ms = car_speed_kmh * (5/18);       
x = car_speed_ms * t;       % Distance traveled by car (meters)

% Bump profile
bump_height = 0.10;         
bump_width = 0.30;          
bump_gap = 0.60;            
num_bumps = 3;              
start_dist = 2.0;           

%% ROAD GENERATION
z_dist = zeros(size(t));    % Road height
z_vel = zeros(size(t));     % Road velocity

for i = 1:num_bumps
    % Calculate start and end distance for each bump
    b_start = start_dist + (i-1)*(bump_width + bump_gap);
    b_end = b_start + bump_width;
    % Find exact moments the tire is on the current bump
    on_bump = (x >= b_start) & (x <= b_end);
    % Calculate height (Sine wave)
    z_dist(on_bump) = bump_height * sin(pi * (x(on_bump) - b_start) / bump_width);
    % Calculate velocity mathematically (Cos wave)
    z_vel(on_bump) = bump_height * (pi/bump_width) * car_speed_ms * cos(pi * (x(on_bump) - b_start) / bump_width);
end

%% WORKSPACE FOR SIMULINK
road_position = timeseries(z_dist, t);
road_velocity = timeseries(z_vel, t);

%% CONFIRMATION MESSAGE
fprintf('SUCCESS: Sedan loaded with %d consecutive bumps.\n', num_bumps);

The main initialization script for the baseline passive suspension system. It defines the Quarter-Car parameters (sprung mass, fixed stiffness, fixed damping) and generates the time-series data for a three-bump road profile.

2. semiactive_script
% Quarter Car Parameters (Realistic Sedan)
m = 350;          % Sprung mass (kg)
c = 18000;        % Spring stiffness (N/m) (Swapped 'c' to 'k')
b_min = 800;      % Minimum damping (N.s/m) (Swapped 'b' to 'c')
b_max = 3000;     % Maximum damping (N.s/m)

%% ROAD PROFILE (5 Consecutive Bumps)
t = (0:0.001:10)';           
car_speed_kmh = 10;         
car_speed_ms = car_speed_kmh * (5/18);       
x = car_speed_ms * t;       

% Bump Properties
bump_height = 0.10;         
bump_width = 0.30;          
bump_gap = 0.60;            
num_bumps = 3;              
start_dist = 2.0;           

%% DATA GENERATION
z_dist = zeros(size(t));    
z_vel = zeros(size(t));     

for i = 1:num_bumps
    b_start = start_dist + (i-1)*(bump_width + bump_gap);
    b_end = b_start + bump_width;
    on_bump = (x >= b_start) & (x <= b_end);
    z_dist(on_bump) = bump_height * sin(pi * (x(on_bump) - b_start) / bump_width);
    z_vel(on_bump) = bump_height * (pi/bump_width) * car_speed_ms * cos(pi * (x(on_bump) - b_start) / bump_width);
end

%% WORKSPACE FOR SIMULINK
road_position = timeseries(z_dist, t);
road_velocity = timeseries(z_vel, t);

fprintf('Success! Semi-active sedan loaded with 3 consecutive bumps.\n');

The initialization script for the modified semi-active system. It defines the identical baseline parameters and road profile to ensure parity in testing, while establishing the minimum and maximum damping boundaries for the controller.

3. gain_scheduler_script
function [Kp, Ki, Kd] = gain_scheduler(rel_vel)
    
    
    % Thresholds
    velocity_impact = 0.6;  
    velocity_settle = 0.12; 
    
    if abs(rel_vel) > velocity_impact
        % STAGE 1: THE HIT (Impact Mode)
        Kp = 0; 
        Ki = 0;    
        Kd = 1000;
        
    elseif abs(rel_vel) > velocity_settle
        % STAGE 2: THE CATCH (Rebound Mode)
        Kp = 0;
        Ki = 0;
        Kd = 2800; 
        
    else
        % STAGE 3: CRUISING (Comfort Mode)
        Kp = 0;
        Ki = 0;
        Kd = 1200;
    end
 within the Simulink closed-loop architecture. This function continuously evaluates the relative velocity of the suspension and outputs dynamically scheduled PID gains across three distinct operational modes: Impact, Rebound, and Cruising.

## Usage Notes
These scripts are designed to initialize the MATLAB Workspace variables (`road_position`, `road_velocity`, etc.) prior to executing the corresponding `.slx` Simulink block diagrams discussed in the main report.
The core logic function embedded
---
*Project developed as part of an undergraduate mechanical engineering study.*
