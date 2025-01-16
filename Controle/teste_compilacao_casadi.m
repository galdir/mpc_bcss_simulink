import casadi.*

x = MX.sym('x');

f = Function('f',{x},{sin(x)});
disp(f(3.14))


opts = struct('main', true,...
              'mex', true,...
              'verbose', true);
f.generate('gen.c',opts);
%f.generate('gen.c');

%C = Importer('gen.c','shell');

%!gcc gen.c -o gen.exe
%!gcc -fPIC -shared gen.c -o gen.so
!C:\MinGW64\mingw64\bin\gcc.exe -fPIC -shared gen.c -o gen.dll
f = external('f', './gen.dll');
disp(f(3.14))

% try
%     mex gen.c
% catch ME
%     disp(['Erro na compilação com mex: ', ME.message]);
%     return; % Sai do script em caso de erro
% end

%f = external('f',C);

