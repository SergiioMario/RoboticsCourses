x_r = 3;        %Initial robot position X
y_r = 3;        %Initial robot position Y
t_r = pi()/4;   %Initial robot angle

%Control constants:
%(They are defined in an array to run the simulation several times
% and get different figures)
alpha = [0.1, 1.0, 4.0];
beta  = [0.1, 0.4, 1.0];

%Physical params and simulation params
L = 0.48;
delta_t = 0.01;

for t