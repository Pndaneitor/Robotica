function v_r = quaternRotate(q, v)
    % Rota un conjunto de vectores usando cuaterniones
    % Entrada:
    %   q = [q_1; q_2; q_3; q_4] (cuaterniones de rotación, uno o más)
    %   v = [v_1; v_2; v_3] (vectores de entrada, uno o más)
    % Salida:
    %   v_r = [vr_1; vr_2; vr_3] (vectores rotados)

    % Normalizar cuaterniones
    q = q ./ vecnorm(q);

    % Número de vectores y cuaterniones
    num_quaternions = size(q, 2);
    num_vectors = size(v, 2);

    % Caso 1: Si solo hay un cuaternión y múltiples vectores, usar el cuaternión para todos los vectores
    if num_quaternions == 1 && num_vectors > 1
        q = repmat(q, 1, num_vectors);  % Replicar cuaternión para todos los vectores

    % Caso 2: Si solo hay un vector y múltiples cuaterniones, rotar el mismo vector con cada cuaternión
    elseif num_quaternions > 1 && num_vectors == 1
        v = repmat(v, 1, num_quaternions);  % Replicar vector para todos los cuaterniones

    % Caso 3: Si el número de cuaterniones y vectores no coinciden, lanzar un error
    elseif num_quaternions ~= num_vectors
        error('El numero de cuaterniones debe coincidir con el numero de vectores o ser uno.');
    end

    % Inicializar la matriz de salida
    v_r = zeros(size(v));
    
    % Aplicar la rotación a cada vector con su cuaternión correspondiente
    for i = 1:max(num_quaternions, num_vectors)
        % Convertir el vector en cuaternión con parte escalar 0
        qv = quaternProd(q(:, i), [0; v(:, i)]);
        
        % Calcular la rotación usando q * v * q_conj
        qvconjq = quaternProd(qv, quaternConj(q(:, i)));
        
        % La parte vectorial resultante es la rotación final
        v_r(:, i) = qvconjq(2:4);  % Ignorar la parte escalar
    end
end
