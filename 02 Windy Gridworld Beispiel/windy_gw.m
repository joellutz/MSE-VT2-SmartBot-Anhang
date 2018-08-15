function [Q,episodesPerTimesteps, statesInLastEpisode] = windy_gw(alpha,epsilonStart,decayingEpsilon,sideII,sideJJ,s_start,s_end,wind,MAX_N_EPISODES)
% WINDY_GR - Performs on-policy sarsa iterative action value funtion estimation 
% for the windy grid world example.
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

PLOT_STEPS = 0; 

gamma = 1;    % <- take this is an undiscounted task 

epsilon = epsilonStart; % for our epsilon greedy policy 

% the number of states: 
nStates = sideII*sideJJ; 

% on each grid we can choose from among this many actions (except on edges where this action is reduced): 
nActions = 4; 

% An array to hold the values of the action-value function 
Q = zeros(nStates,nActions);

if( PLOT_STEPS )
  figure; imagesc( zeros(sideII,sideJJ) ); colorbar; hold on; 
  plot( s_start(2), s_start(1), 'x', 'MarkerSize', 10, 'MarkerFaceColor', 'k' ); 
  plot( s_end(2), s_end(1), 'o', 'MarkerSize', 10, 'MarkerFaceColor', 'k' ); 
end

% keep track of how many timestep we take per episode:
episodesPerTimesteps = zeros(MAX_N_EPISODES,1);
timestep=0; 
for episode=1:MAX_N_EPISODES
  tic; 
  if( episode==1 ) 
    fprintf('working on episode %d...\n',episode);
  else
    fprintf('working on episode %d (ptt=%10.6f secs) with eps=%0.3f...\n',episode, toc, epsilon);
    tic; 
  end
  episodesPerTimesteps(episode,1) = timestep+1; 
  % initialize the starting state
  state = s_start;
  stateIndex = sub2ind( [sideII,sideJJ], state(1), state(2) ); 

  % decay the epsilon after the first 20% of all episodes linearly towards
  % 0 until 80% of all episodes are reached, leave it then to 0 for the
  % rest
  if(decayingEpsilon)
      if(episode>0.2*MAX_N_EPISODES)
          if(episode<0.8*MAX_N_EPISODES)
            epsilon = max(epsilon - epsilonStart/(0.6*MAX_N_EPISODES),0);
          else
              epsilon = 0;
          end
      end
  end
  % pick action using an epsilon greedy policy derived from Q: 
  [~,action] = max(Q(stateIndex,:));  % action \in [1,2,3,4]=[up,down,right,left]
  if( rand<epsilon )         % explore ... with a random action 
    tmp=randperm(nActions);
    action=tmp(1); 
  end
  
  if(episode == MAX_N_EPISODES)
      statesInLastEpisode = zeros(100,2);
      statesInLastEpisode(1,:) = state;
      i = 2;
  end
  
  % begin the episode
  while( 1 ) 
    timestep=timestep+1; 
    %fprintf('st=(%d,%d); act=%3d...\n',st(1),st(2),at);
    % propagate to state stp1 and collect a reward rew
    [reward,nextState] = takeStep(state,action,wind,sideII,sideJJ,s_end); 
    %fprintf('stp1=(%d,%d); rew=%3d...\n',stp1(1),stp1(2),rew);
    % pick the greedy action from state stp1: 
    nextStateIndex = sub2ind( [sideII,sideJJ], nextState(1), nextState(2) ); 
    % make the greedy action selection: 
    [~,nextAction] = max(Q(nextStateIndex,:)); 
    if( rand<epsilon )         % explore ... with a random action 
      tmp=randperm(nActions);
      nextAction=tmp(1); 
    end
    
    if(episode == MAX_N_EPISODES)
      statesInLastEpisode(i,:) = nextState;
      i = i + 1;
    end
    
    if( ~( (nextState(1)==s_end(1)) && (nextState(2)==s_end(2)) ) ) % stp1 is not the terminal state
      Q(stateIndex,action) = Q(stateIndex,action) + alpha*( reward + gamma*Q(nextStateIndex,nextAction) - Q(stateIndex,action) ); 
    else                                                  % stp1 IS the terminal state ... no Q(s';a') term in the sarsa update
      Q(stateIndex,action) = Q(stateIndex,action) + alpha*( reward - Q(stateIndex,action) ); 
      break; 
    end
    %update (st,at) pair: 
    if( PLOT_STEPS && timestep>8000 ) 
      num2act = { 'UP', 'DOWN', 'RIGHT', 'LEFT' }; 
      plot( state(2), state(1), 'o', 'MarkerFaceColor', 'g' ); title( ['action = ',num2act(nextAction)] ); 
      plot( nextState(2), nextState(1), 'o', 'MarkerFaceColor', 'k' ); drawnow; 
    end 
    state = nextState;
    stateIndex = nextStateIndex;
    action = nextAction; 
    
    %pause; 
  end % end policy while 
end % end episode loop 

% from: https://ch.mathworks.com/matlabcentral/answers/40018-delete-zeros-rows-and-columns
statesInLastEpisode( ~any(statesInLastEpisode,2), : ) = [];  % removes all rows with all zero

function [rew,stp1] = takeStep(st,act,wind,sideII,sideJJ,s_end)
% STNAC2STP1 - state and action to (state plus one) and reward 
%   

% convert to row/column notation: 
ii = st(1); jj = st(2); 

% get the wind amount: 
theWind = wind(jj); 

% incorporate any actions and fix our position if we end up outside the grid:
% 
switch act
 case 1
  %
  % action = UP 
  %
  stp1 = [ii-1-theWind,jj];
 case 2
  %
  % action = DOWN
  %
  stp1 = [ii+1-theWind,jj];
 case 3
  %
  % action = RIGHT
  %
  stp1 = [ii-theWind,jj+1];
 case 4
  %
  % action = LEFT 
  %
  stp1 = [ii-theWind,jj-1];
 otherwise
  error(sprintf('unknown value for of action = %d',act)); 
end

% adjust our position of we have fallen outside of the grid:
% 
if( stp1(1)<1      ) stp1(1)=1;      end
if( stp1(1)>sideII ) stp1(1)=sideII; end
if( stp1(2)<1      ) stp1(2)=1;      end
if( stp1(2)>sideJJ ) stp1(2)=sideJJ; end

% get the reward for this step: 
% 
if( (ii==s_end(1)) && (jj==s_end(2)) )
  %rew=+1;
  rew=0;
else
  rew=-1;
end



