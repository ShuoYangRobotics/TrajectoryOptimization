function setpath(varargin)

% addpath([pwd,'/util'           ], '-end');
% addpath([pwd,'/examples'       ], '-end');
% addpath([pwd,'/examples/t1diet'], '-end');
% addpath([pwd,'/examples/sntoy' ], '-end');
% addpath([pwd,'/examples/snmain'], '-end');
% addpath([pwd,'/examples/hsmain'], '-end');
% addpath([pwd,'/examples/hs76'  ], '-end');
% addpath([pwd,'/examples/fmincon'], '-end');
% addpath([pwd,'/examples/hs116'  ], '-end');
% addpath([pwd,'/examples/spring' ], '-end');

p = genpath('/home/biorobotics/sy_ws/TrajectoryOptimization/snopt/snopt-matlab-3.0.0/')
addpath(p);
p = genpath('/home/biorobotics/sy_ws/OptimTraj')
addpath(p);

setenv('SNOPT_LICENSE', '/home/biorobotics/sy_ws/TrajectoryOptimization/snopt/snopt7.lic')
