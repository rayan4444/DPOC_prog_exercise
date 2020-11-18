%Open the example matrix
P_ex= load('example_P');

G_ex = load('example_G');

P=ComputeTransitionProbabilities(stateSpace, map);
G = ComputeStageCosts(stateSpace, map);

%Checks if both matrices are the same
P_check = isequal(P,P_ex)

G_check = isequal(G, G_ex)