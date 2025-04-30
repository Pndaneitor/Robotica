function R = euler2rotMat(euler, secuencia)
%euler2rotMat Convierte la orientación en ángulos de Euler en una matriz de rotación.
%
% Ejemplo de uso:
% euler = [pi/2; -pi/4; pi/6]
% secuencia = "XYZ"
% R = euler2rotMat(euler, secuencia)

phi = euler(1,:);      % phi:   rotación alrededor del eje X
theta = euler(2,:);    % theta: rotación alrededor del eje Y
psi = euler(3,:);      % psi:   rotación alrededor del eje Z
if secuencia == "XYZ"
<<<<<<< HEAD
    R =Rx(phi)*Ry(theta)*Rz(psi);
=======
    R = Rx(phi)*Ry(theta)*Rz(psi);
>>>>>>> 819eefc5a62cfcd4c057ce6ba022a87a051ac44e
end

