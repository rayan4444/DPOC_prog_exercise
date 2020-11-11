function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
%ComputeTerminalStateIndex Compute the index of the terminal state in the
%stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the terrain of the estate map. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix

global DROP_OFF K

%MATLAB starts indexing at 1 so an index of 0 doesn't correspond to any
%state in the matrix
stateIndex= 0; 

for i= 1:K
    m = stateSpace(i,1);
    n = stateSpace(i,2);
    if (map(m,n)==DROP_OFF) && ( stateSpace(i,3) == 1)
        stateIndex=i;
        return;
    end 
end

%If the function doesn't return above then display an error message
disp('Error looking for the terinal state index');

end
