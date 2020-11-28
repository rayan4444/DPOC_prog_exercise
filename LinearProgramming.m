function [ J_opt, u_opt_ind, A, b, cost_every_action] = LinearProgramming(P, G)
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

global K HOVER;

%% Handle terminal state
global TERMINAL_STATE_INDEX;

%Terminal state index should be removed from the P and G matrices. For P,
%considering both the initial and transition state: 

P(TERMINAL_STATE_INDEX,:,:)=[];
P(:,TERMINAL_STATE_INDEX,:)=[];
G(TERMINAL_STATE_INDEX,:)=[];


%We cannot have infinity in the G matrix for the computations. We can
%either choose to remove it or set it to a very large number as below. 
G(isinf(G))=10^12;

%Identity matrix
I=eye(K-1); 
f=-ones(K-1,1);

%Matrices to be implemented in linprog function 
A=[];
b=[];

for l=1:5 % iterate for every input/action

        %Defining left entry:
        cost_vector = [];
        
        %Defining right entry:
        prob_matrix = P(:,:,l);
        
        for i = 1:K-1
            cost_entry = G(i,l);
            cost_vector = [cost_vector;G(i,l)];
        end  
        
    %stacked A
   A=[A; I-P(:,:,l)];
    
    %stacked b       
   b=[b;cost_vector];
   
    
end

%Check for sizes (debugging)
size(A);
size(b);

%Lower bound, upper bound and equalities are all empty
l_bd = [];
u_bd=[];
A_eq = [];
b_eq = [];

%Initialising J_opt and u_opt:
J_opt = ones(K-1,1)
u_opt = ones(K-1,1)
u_opt_ind=ones(K-1,1)

%Using linprog function to compute J_opt:
J_opt = linprog(f,A,b,A_eq,b_eq,l_bd,u_bd);


%Now looping first through every state as the optimal action for every
%state should be determined we have: 

cost_every_action=ones(5,1) %Size based on all possible inputs

for i = 1:K-1
    for l=1:5
        cost_to_go = G(i,l); %Cost for every input at this state
        trans_prob = P(i,:,l); %Transition prob for all other states given this input
        cost_every_action(l) = cost_to_go + trans_prob*J_opt;
    end

    [u_opt(i),u_opt_ind(i)] = min(cost_every_action);
end

%At the terminal state, in principle the optimal cost-to-go is 0 and the
%ideal action is assumed as hover

u_opt_ind = [u_opt_ind(1:TERMINAL_STATE_INDEX-1);5; u_opt_ind(TERMINAL_STATE_INDEX:end)];
J_opt = [J_opt(1:TERMINAL_STATE_INDEX-1);0;J_opt(TERMINAL_STATE_INDEX:end)]
end


