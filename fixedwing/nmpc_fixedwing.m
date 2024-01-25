% define nx,ny,nu for the numbers of model state, output, inputs
nx = 9;
ny = 9; %nx=ny, state equal to output
nu = 4; 
nlobj = nlmpc(nx,ny,nu);

Ts = 0.1;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 5;

nlobj.Model.StateFcn = "fixedwing_DT";
nlobj.Model.IsContinuousTime = false;

nlobj.Model.NumberOfParameters = 1;

nlobj.Model.OutputFcn = @(x,u,Ts) [x(1); x(3)];

x0 = [250;0;0;5;5;0;0;0;0];
u0 = [0;0;0;0];
validateFcns(nlobj, x0, u0, [], {Ts});