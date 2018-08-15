% WINDY_GR_QLearn_Script - Performs on-policy q-learn iterative action value funtion estimation for the windy grid world example.
% 
% Written by:
% -- 
% John L. Weatherwax                2007-12-03
% 
% email: wax@alum.mit.edu
% 
% Please send comments and especially bug reports to the
% above email address.
% 
%-----

close all; 
clearvars
clc

alpha = 1e-1; 
epsilon = 0.2;
decayingEpsilon = 1;    % 1 = TRUE, 0 = FALSE

sideII  = 21;
sideJJ = 30; 

% the wind in each column: 
wind = [ 0 0 0 1 1 1 2 2 1 0 ]; 
wind = zeros(1,sideJJ);
wind(1:fix(0.3*sideJJ)) = 1;
wind(fix(0.3*sideJJ):fix(0.5*sideJJ)) = -1;
wind(fix(0.5*sideJJ):fix(0.7*sideJJ)) = 2;
wind(fix(0.7*sideJJ):fix(0.9*sideJJ)) = 1;
wind(fix(0.9*sideJJ):fix(1.0*sideJJ)) = -1;

% the beginning and terminal states (in matrix notation): 
s_start = [ round(sideII/2), 1 ]; 
s_end   = [ round(sideII/2), round(0.8*sideJJ) ]; 

MAX_N_EPISODES=30; 
MAX_N_EPISODES=1e3; % 28 episodes to get around 8000 timesteps 
MAX_N_EPISODES=1e4;
%MAX_N_EPISODES=1e5;
%MAX_N_EPISODES=1e6;
%MAX_N_EPISODES=10e6;

[Q,ets, statesInLastEpisode] = windy_gw(alpha,epsilon,decayingEpsilon,sideII,sideJJ,s_start,s_end,wind,MAX_N_EPISODES);

[Q_QL,ets_QL, statesInLastEpisode_QL] = windy_gw_QLearn(alpha,epsilon,decayingEpsilon,sideII,sideJJ,s_start,s_end,wind,MAX_N_EPISODES);

pol_pi = zeros(sideII,sideJJ);
pol_pi_QL = zeros(sideII,sideJJ);
V = zeros(sideII,sideJJ); 
V_QL = zeros(sideII,sideJJ); 
for ii=1:sideII
  for jj=1:sideJJ
    sti = sub2ind( [sideII,sideJJ], ii, jj ); 
    [V(ii,jj),pol_pi(ii,jj)] = max( Q(sti,:) ); 
    [V_QL(ii,jj),pol_pi_QL(ii,jj)] = max( Q_QL(sti,:) ); 
  end
end

plot_gw_policy(pol_pi,s_start,s_end,wind);
title( 'SARSA-policy (1=>up,2=>down,3=>right,4=>left)' ); 
plot_gw_policy(pol_pi_QL,s_start,s_end,wind);
title( 'Q-Learning-policy (1=>up,2=>down,3=>right,4=>left)' ); 

figure;
imagesc( V );
colormap(flipud(jet));
colorbar; 
title( 'state value function' ); 
fn = sprintf('windy_gw_Comparison_state_value_fn_nE_%d',MAX_N_EPISODES);
hold on
plot(statesInLastEpisode(:,2), statesInLastEpisode(:,1), 'k', 'LineWidth',3)
plot(statesInLastEpisode_QL(:,2), statesInLastEpisode_QL(:,1), 'b', 'LineWidth',3)
legend('SARSA','Q-Learning');
saveas( gcf, fn, 'png' ); 

figure;
plot( ets, 1:length(ets) );
hold on
plot( ets_QL, 1:length(ets_QL) );
grid on;
title('episodes completed per time step')
ylabel('episodes')
xlabel('time steps')
legend('SARSA','Q-Learning');
fn = sprintf('windy_gw_Comparison_learning_rate_nE_%d',MAX_N_EPISODES);
saveas( gcf, fn, 'png' ); 