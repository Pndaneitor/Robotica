function RHz=HRz(theta)
dato=whos('theta');
 if strcmp(dato.class, 'sym') %variables simbólicas
   RHz=simplify([cos(theta), -sin(theta),  0,  0;
                 sin(theta),  cos(theta),  0,  0;
                          0,           0,  1,  0;
                          0,           0,  0,  1],3);                
 else
      digits(3); %cálculos numéricos
     RHz=[ cos(theta),  -sin(theta),  0,  0;
                   sin(theta),  cos(theta),  0,  0;
                     0,      0,      1,  0;
                     0,      0,      0,  1];          
 end
end
