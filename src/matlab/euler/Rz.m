<<<<<<< HEAD
function R = Rz(spi)
%Rz realiza una rotación en un ángulo theta en radianes con respecto al eje z

R =[cos(spi), -sin(spi), 0;
    sin(spi),  cos(spi), 0;
     0,          0,          1];
=======
function R = Rz(theta)
% Rz realiza una rotación en un ángulo theta en radianes con respecto al eje Z.
R = [cos(theta), -sin(theta), 0;
     sin(theta), cos(theta), 0;
     0, 0, 1];
end
>>>>>>> 819eefc5a62cfcd4c057ce6ba022a87a051ac44e
