function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER
local A b

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)
I=eye(K);
J_opt=zeros(K,1);
f=-ones(K,1);


for l=1:5 % iterate for every input
    for i=i:K
        for j=i:K
        %J_opt < G(i,l) + P(i,j,l)*J_opt
        %I(i)-P(i,j,l)* J_opt < G(i,l)
        
        % here we get the A and b parts for a single output, in for of
        % cells that are then stacked. like this we have a giant stack of
        % all the possibilities, stacked for all the inputs
        end
    end
    % here is where we actually stack them 
    %stacked A 
    
    %stacked b
    
    
end 
%after we have our massive matrix we run linprog and it magically spits out a solution
[%x,fval] = linprog(___), for any input arguments, returns the value of the objective function fun at the solution x: fval = f'*x.
 [J_opt, u_opt_ind]=linprog(f,A,b);
end

