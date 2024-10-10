clc; clear; 

% Define our tubes (ID, OD, r, l, d, E)
tube1 = Tube(3.046*10^-3, 3.3*10^-3, 1/17, 90*10^-3, 50*10^-3, 1935*10^6);
tube2 = Tube(2.386*10^-3, 2.64*10^-3, 1/22, 170*10^-3, 50*10^-3, 1935*10^6);
tube3 = Tube(1.726*10^-3, 1.98*10^-3, 1/29, 250*10^-3, 50*10^-3, 1935*10^6);

% TODO You will need to uncomment one of the below lines, depending if you
% are testing a 2-tube or 3-tube case
%tubes = [tube1, tube2];
tubes = [tube1, tube2, tube3];

robot = Robot(tubes);

% q values for testing with two tubes
% q_var = [0, 0, 60, 60; 
%          0, 0, 60, 0; 
%          0, 0, 60, -30];


% q values for testing with three tubes
 q_var = [0, 0, 0, 60, 60, 0;
          0, 0, 0, 60, 0, 0; 
          0, 0, 0, 60, -30, 0];

T = {};
for i = 1:size(q_var,1)
    disp(q_var(i,:));
    set_T = robot.fkin(q_var(i,:));
    T{i} = set_T;
    disp(T{i});
end