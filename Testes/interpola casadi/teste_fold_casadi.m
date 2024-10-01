clear
import casadi.*

N = 4;
X = MX.sym('X',1,N);
disp(f)
ys = {};
for i=1:N
    ys{end+1} = f(X(:,i));
end
Y = [ys{:}];
F = Function('F',{X},{Y});
disp(F)