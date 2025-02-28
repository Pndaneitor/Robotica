function q_log = quaternLog(Q)
    % Q es la matriz de cuaterniones de tamaño 4 x m
    [n, m] = size(Q);
    
    if n ~= 4
        error('La entrada debe ser una matriz de 4 x m cuaterniones.');
    end
    
    % Inicializar la matriz de salida
    q_log = zeros(4, m);
    
    % Iterar sobre cada cuaternión (cada columna)
    for i = 1:m
        q = Q(:, i);      % Extraer el cuaternión i-ésimo
        v = q(2:4);       % Parte vectorial del cuaternión
        q_0 = q(1);       % Parte escalar del cuaternión
        v_norm = norm(v); % Norma de la parte vectorial
        
        % Calcular el logaritmo del cuaternión
        if v_norm == 0
            q_log(:, i) = [log(norm(q)); 0; 0; 0];
        else
            q_log(:, i) = [log(norm(q)); v / v_norm * acos(q_0 / norm(q))];
        end
    end
end