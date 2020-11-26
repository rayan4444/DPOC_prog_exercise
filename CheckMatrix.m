
%Open the example matrices
P_ex= load('example_P');
P_array = struct2array(P_ex);


G_ex = load('example_G');
G_array =  struct2array(G_ex);

%Self-Computed P and G:
P=ComputeTransitionProbabilities(stateSpace, map);
G = ComputeStageCosts(stateSpace, map);

%Checks if both matrices are the same (output 0 if no, 1 if yes)
P_check= isequal(P, P_array)
G_check = isequal(G, G_ex)