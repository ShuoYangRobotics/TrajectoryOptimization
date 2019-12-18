function setpath(varargin)

addpath([pwd,'/util'           ], '-end');
addpath([pwd,'/examples'       ], '-end');
addpath([pwd,'/examples/t1diet'], '-end');
addpath([pwd,'/examples/sntoy' ], '-end');
addpath([pwd,'/examples/snmain'], '-end');
addpath([pwd,'/examples/hsmain'], '-end');
addpath([pwd,'/examples/hs76'  ], '-end');
addpath([pwd,'/examples/fmincon'], '-end');
addpath([pwd,'/examples/hs116'  ], '-end');
addpath([pwd,'/examples/spring' ], '-end');

setenv('SNOPT_LICENSE', 'E:\CMU_research\TrajectoryOptimization\snopt\snopt7.lic')
