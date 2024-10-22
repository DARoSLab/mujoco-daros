% This script generates a C++ file from a symbolically defined matrix

% Clear workspace and command window
clear; clc;

% Add symbolic package if using Octave
% Uncomment the next line if you are using GNU Octave instead of MATLAB
% pkg load symbolic

% Define symbolic variables
syms x y;

z = [x; y];
% Define a symbolic matrix
A = [z(1)^2 + z(2), 0; 0, z(1)^2 - z(2)^2];

% Use matlabFunction to generate a C++ file
% 'File' specifies the name of the output file
% 'Vars' specifies the input variables to the function
matlabFunction(A, 'File', 'matrixFunction', 'Vars', {z});


disp('C++ file "matrixFunction.cpp" has been generated.');
