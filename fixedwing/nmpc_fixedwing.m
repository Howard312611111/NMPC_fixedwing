% define nx,ny,nu for the numbers of model state, output, inputs
nx = 9;
ny = 9; %nx=ny, state equal to output
nu = 4; 
nlobj = nlmpc(nx,ny,nu);

Ts = 0.1;
nlobj.Ts = Ts; %sample time=0.1s
nlobj.PredictionHorizon = 10; %set predictionhorizon
nlobj.ControlHorizon = 5;     %set controlhorizon

nlobj.Model.StateFcn = "fixedwing_DT";  %set model in the file
nlobj.Model.IsContinuousTime = false;

nlobj.Model.NumberOfParameters = 1;

nlobj.Model.OutputFcn = @(x,u,Ts)[x(1);x(2);x(3);x(4);x(5);x(6);x(7);x(8);x(9)] ;

x0 = [250;0;0;5;5;0;0;0;0];   %initial state
u0 = [0;0;0;0];               %initial input
validateFcns(nlobj, x0, u0, [], {Ts});