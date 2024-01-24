% define nx,ny,nu for the numbers of model state, output, inputs
nx = 5;
ny = 5; %nx=ny, state equal to output
nu = 2; 
nlobj = nlmpc(nx,ny,nu);

