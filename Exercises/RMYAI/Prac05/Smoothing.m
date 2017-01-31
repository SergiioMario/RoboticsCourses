%%%THIS SCRIPT IS INTENDED TO BE RUN USING MATLAB (Yes, I'm sorry)%%%%%%        
clear

x_old = [ 3.5, 4, 4.5, 5, 5.5, 6,   6, 6,  6, 6,  6, 6,  6, 6, 6.5, 7, 7.5, 8, 8.5];
y_old = [  10,10,  10,10,  10,10, 9.5, 9,8.5, 8,7.5, 7,6.5, 6,   6, 6,   6, 6,   6];
plot(x_old,y_old, '.r-', 'MarkerSize', 20, 'color', 'red'); 
hold on;

%%%%Parameters
tolerance = 0.0001;
tolerance = tolerance * numel(x_old);

%%%%First run calculates a good path 
x_new = x_old;
y_new = y_old;
a = 0.35;   %%Weights how similar is the new path to the old one
b = 0.65;   %%Weights how smooth is the new path
change = tolerance + 1;
attempts = 10000;
while change > tolerance && attempts > 0
    change = 0;
    for i=2:(numel(x_old)-1)
        last_x = x_new(i);
        last_y = y_new(i);
        x_new(i) = x_new(i) + a*(x_old(i) - x_new(i)) + b*(x_new(i+1) + x_new(i-1) - 2*x_new(i));
        y_new(i) = y_new(i) + a*(y_old(i) - y_new(i)) + b*(y_new(i+1) + y_new(i-1) - 2*y_new(i));
        change = change + abs(x_new(i) - last_x) + abs(y_new(i) - last_y);
    end
    attempts = attempts - 1;
end
plot(x_new,y_new, '.r-', 'MarkerSize', 20, 'color', [0 0.6 0]); 


%%%%Second run calculates a straight line
x_new = x_old;
y_new = y_old;
a = 0.0;   %%Weights how similar is the new path to the old one
b = 0.8;   %%Weights how smooth is the new path
change = tolerance + 1;
attempts = 10000
while change > tolerance && attempts > 0
    change = 0;
    for i=2:(numel(x_old)-1)
        last_x = x_new(i);
        last_y = y_new(i);
        x_new(i) = x_new(i) + a*(x_old(i) - x_new(i)) + b*(x_new(i+1) + x_new(i-1) - 2*x_new(i));
        y_new(i) = y_new(i) + a*(y_old(i) - y_new(i)) + b*(y_new(i+1) + y_new(i-1) - 2*y_new(i));
        change = change + abs(x_new(i) - last_x) + abs(y_new(i) - last_y);
    end
    attempts = attempts - 1;
end
plot(x_new,y_new, '.r-', 'MarkerSize', 20, 'color', 'blue'); 

axis([3, 9, 5.5, 10.5])
%legend('\alpha=1.0, beta=0.0','\alpha=1.0, beta=0.0', '\alpha=1.0, beta=0.0')
set(gca, 'xtick', [])
set(gca, 'ytick', [])
box off;
