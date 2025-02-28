function ab = quaternProd(a, b)
% Calcula el producto entre los cuaterniones a y b.
%
%   ab = prodCuatern(a, b)

%     dato1=whos('a');
%     dato2=whos('b');
    % variables simbólicas   
%     if strcmp(dato1.class, 'sym') || strcmp(dato2.class, 'sym')
%         ab = sym(zeros(size(a)));
%     else % variables numéricas
%         ab = zeros(size(a));
%     end
    ab = zeros(size(a));
    ab(1,:) = a(1,:).*b(1,:)-a(2,:).*b(2,:)-a(3,:).*b(3,:)-a(4,:).*b(4,:);
    ab(2,:) = a(1,:).*b(2,:)+a(2,:).*b(1,:)+a(3,:).*b(4,:)-a(4,:).*b(3,:);
    ab(3,:) = a(1,:).*b(3,:)-a(2,:).*b(4,:)+a(3,:).*b(1,:)+a(4,:).*b(2,:);
    ab(4,:) = a(1,:).*b(4,:)+a(2,:).*b(3,:)-a(3,:).*b(2,:)+a(4,:).*b(1,:);
end