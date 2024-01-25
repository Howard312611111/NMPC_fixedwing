% define nx,ny,nu for the numbers of model state, output, inputs
nx = 9;
ny = 9; %nx=ny, state equal to output
nu = 4; 
nlobj = nlmpc(nx,ny,nu);

