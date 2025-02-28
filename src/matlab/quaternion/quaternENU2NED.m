function Q_ned = quaternENU2NED(Q_enu)
    % Convert a matrix of quaternions from ENU to NED
    % Q_enu is a 4×n matrix where each column is a quaternion [q0; q1; q2; q3]
    % Q_ned is the resulting 4×n matrix in the NED frame
    
    Q_ned = [Q_enu(1, :);   % q0 remains unchanged
             Q_enu(3, :);   % q2 becomes the second component
             Q_enu(2, :);   % q1 becomes the third component
            -Q_enu(4, :)];  % q3 is negated
end
