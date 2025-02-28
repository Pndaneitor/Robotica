function qConj = quaternConj(q)
% Convierte un cuaternión en su conjugado.
%
%   qConj = quaternConj(q)

    qConj = [q(1,:); -q(2,:); -q(3,:); -q(4,:)];
end