function g = dibujar_robot(g, r, view_vector, escala_ejes, rango_x, rango_y, rango_z)
% DIBUJAR_ROBOT - Dibuja el robot y actualiza la estructura gráfica.

N_DOFS = r.NGDL;
base = r.A0(1:3,4);

% Verificación de valores complejos en r.T
if ~isreal(r.T)
    error('¡La matriz r.T contiene valores complejos!');
end

max_reach = 1.1 * max(sum(abs(r.d)), sum(abs(r.a)));

% Configurar figura solo una vez
if (g.h == -1)
    figure;
    hold on;
    grid on;
    axis equal;

    if nargin < 3 || isempty(view_vector)
        view_vector = [1 1 1];
    end
    if nargin < 4 || isempty(escala_ejes)
        escala_ejes = 0.5;
    end
    if nargin < 5 || isempty(rango_x)
        rango_x = [base(1)-max_reach, base(1)+max_reach];
    end
    if nargin < 6 || isempty(rango_y)
        rango_y = [base(2)-max_reach, base(2)+max_reach];
    end
    if nargin < 7 || isempty(rango_z)
        rango_z = [base(3)-max_reach, base(3)+max_reach];
    end

    xlim(rango_x);
    ylim(rango_y);
    zlim(rango_z);

    view(view_vector);

    g.h = plot3(0, 0, 0, '-m*', 'LineWidth', 4);
    g.quiver_x = quiver3(0, 0, 0, 0, 0, 0, escala_ejes, 'r');
    g.quiver_y = quiver3(0, 0, 0, 0, 0, 0, escala_ejes, 'g');
    g.quiver_z = quiver3(0, 0, 0, 0, 0, 0, escala_ejes, 'b');

    xlabel('x');
    ylabel('y');
    zlabel('z');

    for i = 0 : N_DOFS
        g.htxt(i+1) = text(0, 0, 0, num2str(i), 'FontWeight', 'bold');
    end

    set(g.htxt(1), 'Position', base);
end

% Inicialización
vx = zeros(3, N_DOFS);
vy = zeros(3, N_DOFS);
vz = zeros(3, N_DOFS);
x  = zeros(3, N_DOFS);

for i = 1 : N_DOFS
    vx(:, i) = r.T(1:3, 1:3, i) * [1; 0; 0];
    vy(:, i) = r.T(1:3, 1:3, i) * [0; 1; 0];
    vz(:, i) = r.T(1:3, 1:3, i) * [0; 0; 1];
    x(:, i)  = r.T(1:3, 4, i);

    set(g.htxt(i+1), 'Position', x(:, i) + [0; 0; 0.2]);
end

% Limpieza de posibles valores complejos residuales
vx = real(vx); vy = real(vy); vz = real(vz); x = real(x);

% Actualización de línea de eslabones
set(g.h, 'XData', [base(1) x(1, :)], ...
         'YData', [base(2) x(2, :)], ...
         'ZData', [base(3) x(3, :)]);

% Actualización de flechas (ejes locales)
set(g.quiver_x, 'XData', x(1, :), 'YData', x(2, :), 'ZData', x(3, :), ...
                'UData', vx(1, :), 'VData', vx(2, :), 'WData', vx(3, :));
set(g.quiver_y, 'XData', x(1, :), 'YData', x(2, :), 'ZData', x(3, :), ...
                'UData', vy(1, :), 'VData', vy(2, :), 'WData', vy(3, :));
set(g.quiver_z, 'XData', x(1, :), 'YData', x(2, :), 'ZData', x(3, :), ...
                'UData', vz(1, :), 'VData', vz(2, :), 'WData', vz(3, :));

drawnow;



end
