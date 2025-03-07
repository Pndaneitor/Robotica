function R = Rz(spi)
%Rz realiza una rotación en un ángulo theta en radianes con respecto al eje z

R =[cos(spi), -sin(spi), 0;
    sin(spi),  cos(spi), 0;
     0,          0,          1];