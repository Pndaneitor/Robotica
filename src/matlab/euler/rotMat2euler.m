function euler = rotMat2euler(R, secuencia)
%rotMat2euler Convierte una matriz de rotación a la orientación en ángulos de Euler.
%
% Ejemplo de uso:
% R = [-1 0 0
%       0 0 1
%       0 1 0];
% secuencia = "XYZ"
% euler = rotMat2euler(R, secuencia)

% Obtén las ecuaciones de la imagen
if secuencia == "XYZ"
    phi =       % phi:   rotación alrededor del eje X
    theta =     % theta: rotación alrededor del eje Y
    psi =       % psi:   rotación alrededor del eje Z
    euler =
end

