function output = controller(input)
%%
% INPUTS/DEFINITION/CONVERSIONS
w_MGB = input(1);            % get flywheel angular velocity
dw_MGB = input(2);           % get flywheel angular acceleration
T_MGB = input(3);            % get flywheel torque
Q_BT = input(4);             % get charge of battery (C)

global w_EM_max;             % define maximum motor angular velocity (global) 
global T_EM_max;             % define maximum motor torque (global)
persistent next_mode;        % define next mode (local)
persistent prev_mode;        % define previous mode (local)

theta_EM = 0.1;              % define motor inertia
T_MGB_th = 55;               % torque threshold - lower bound for entering into LPS in generator mode
T_MGB_th2 = 90;              % torque threshold - lower bound for entering into LPS in motor mode
T_ED_max = 38;               % define max Torque for Electric Driving
epsilon = 0.01;              % define epsilon
u_LPS_max = 0.33;            % define maximum torque-split factor for LPS
SOC = Q_BT*(1/360);          % define State of Charge (SoC) in (%)
SOC_in = 14;                 % define SOC (%) for entering into Charging mode
SOC_out = 38;                % define SOC (%) for quiting Charging mode 

%%
% Mods realisation

if isempty(next_mode)        % first mode by default is Start-Stop mode
    next_mode = 1;
    prev_mode = 1;
end

switch (next_mode)           % execution of the modes under given conditions
%%
       
    case 1                   % Start - Stop mode
        %%
        % State
        state_CE = 0;
        u = 1;
        prev_mode = 1;
        
        % Switch Conditions
        if SOC <= SOC_in
          next_mode = 4;
         elseif T_MGB < T_ED_max
          next_mode = 3;
         elseif T_MGB >= T_ED_max
          next_mode = 2;
        end
        
        
    case 2                   % Load Point Shifting mode
        %%
        % State
        state_CE = 1;
        prev_mode = 2;
        
        if T_MGB >= T_MGB_th2                    % Motor Mode
          u = min((interp1(w_EM_max,T_EM_max,w_MGB)-abs(theta_EM*dw_MGB)-epsilon)/T_MGB,u_LPS_max);
         elseif T_MGB > 0 && T_MGB < T_MGB_th    % Generator Mode
          u = max((interp1(w_EM_max,-T_EM_max,w_MGB)-abs(theta_EM*dw_MGB)-epsilon)/T_MGB,-u_LPS_max);
         else
          u = 0;
        end
        
        % Switch Conditions
        if T_MGB < 0
          next_mode = 5;
         elseif SOC <= SOC_in
          next_mode = 4;
         elseif T_MGB < T_ED_max
          next_mode = 3;
         elseif T_MGB == 0
          next_mode = 1;            
        end
       
    case 3                   % Electric driving mode
        %%
        % State
        state_CE = 0;
        u = 1;
        prev_mode = 3;
        
        % Switch Conditions
        if SOC <= SOC_in
          next_mode = 4;
         elseif T_MGB < 0
          next_mode = 5;
         elseif T_MGB >= T_ED_max
          next_mode = 2;
          state_CE = 1;
          u = 0;
        end
        
    case 4                  % Charging mode
        %%
        % State
        state_CE = 1;
        prev_mode = 4;
        if T_MGB>0 && T_MGB <= T_MGB_th2    % Generator Mode
          u = max((interp1(w_EM_max,-T_EM_max,w_MGB)-abs(theta_EM*dw_MGB)-epsilon)/T_MGB,-u_LPS_max);
         else
          u = 0;
        end
        
        % Switch Conditions
        if SOC >= SOC_out
          if T_MGB >= T_ED_max
            next_mode = 2;
           else
            next_mode = 3;
          end
         elseif T_MGB < 0
            next_mode = 5;
        end
  
    case 5                  % Breaking mode
        %%
        % State
        state_CE = 0;
        
        if T_MGB < 0
          u = min((interp1(w_EM_max,-T_EM_max,w_MGB)+abs(theta_EM*dw_MGB)+epsilon)/T_MGB,1);
         else
          u = 1;
        end
        
        % Switch Conditions
        if T_MGB >= 0 && prev_mode == 4
          next_mode = 4;
          prev_mode = 5;
         elseif T_MGB >= 0 && T_MGB < T_ED_max
          next_mode = 3;
          prev_mode = 5;
         elseif T_MGB >= 0 && T_MGB >= T_ED_max
          next_mode = 2;
          prev_mode = 5;
        end
end        

%% 
%OUTPUT
output(1) = state_CE;
output(2) = u;