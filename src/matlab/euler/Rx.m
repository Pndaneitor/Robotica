<<<<<<< HEAD
function R=Rx(phi)
%Rx realiza una rotación en un ángulo theta en radianes con respecto al eje X
 R = [1,      0,       0;
          0, cos(phi), -sin(phi);
          0, sin(phi),  cos(phi)];
=======
function R = Rx(theta)
% Rx realiza una rotación en un ángulo theta en radianes con respecto al eje X.
R = [1, 0, 0;
     0, cos(theta), -sin(theta);
     0, sin(theta), cos(theta)];
>>>>>>> 819eefc5a62cfcd4c057ce6ba022a87a051ac44e
end