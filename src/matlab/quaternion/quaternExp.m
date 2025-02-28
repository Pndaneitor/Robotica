function expq = quaternExp(q)
num_quaternions = size(q, 2);
expq = zeros(size(q));
% Aplicar la rotación a cada vector con su cuaternión correspondiente
for i = 1:max(num_quaternions)
    q_0 = q(1,i);
    v = q(2:4,i);
    v_norm = norm(v);
    if v_norm == 0
        expq(:,i) = [exp(q_0); 0; 0; 0];
    else
        expq(:,i) = exp(q_0)*[cos(v_norm); sin(v_norm) * v / v_norm];
    end
end
