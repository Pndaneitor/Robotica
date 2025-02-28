function R = quatern2rotMat(q)
%QUATERN2ROTMAT Converts a quaternion orientation to a rotation matrix
%
%   R = quatern2rotMat(q)
%
%   Converts a quaternion orientation to a rotation matrix.
%
%   For more information see:
%   http://www.x-io.co.uk/node/8#quaternions
%
%	Date          Author          Notes
%	27/09/2011    SOH Madgwick    Initial release

% Rx = [q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2
%         2*q(1)*q(4) + 2*q(2)*q(3)
%         2*q(2)*q(4) - 2*q(1)*q(3)];
% % Rx = Rx/norm(Rx);
% 
% Ry = [        2*q(2)*q(3) - 2*q(1)*q(4)
%       q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2
%               2*q(1)*q(2) + 2*q(3)*q(4)];
% % Ry = Ry/norm(Ry);
% 
% Rz = [        2*q(1)*q(3) + 2*q(2)*q(4)
%               2*q(3)*q(4) - 2*q(1)*q(2)
%       q(1)^2 - q(2)^2 - q(3)^2 + q(4)^2];
% % Rz = Rz/norm(Rz);
% R = [Rx Ry Rz];

% Rx = [2*q(1)^2-1+2*q(2)^2;
%       2*(q(2)*q(3)+q(1)*q(4));
%       2*(q(2)*q(4)-q(1)*q(3))];
% Rx = Rx/norm(Rx);
% 
% Ry = [2*(q(2)*q(3)-q(1)*q(4));
%       2*q(1)^2-1+2.*q(3)^2;
%       2*(q(3)*q(4)+q(1)*q(2))];
% Ry = Ry/norm(Ry);
% Rz = [2*(q(2)*q(4)+q(1)*q(3));
%       2*(q(3)*q(4)-q(1)*q(2));
%       2*q(1)^2-1+2.*q(4)^2];
% Rz = Rz/norm(Rz);
% R = [Rx Ry Rz];

R(1,1,:) = 2.*q(1,:).^2-1+2.*q(2,:).^2;
R(1,2,:) = 2.*(q(2,:).*q(3,:)-q(1,:).*q(4,:));
R(1,3,:) = 2.*(q(2,:).*q(4,:)+q(1,:).*q(3,:));
R(2,1,:) = 2.*(q(2,:).*q(3,:)+q(1,:).*q(4,:));
R(2,2,:) = 2.*q(1,:).^2-1+2.*q(3,:).^2;
R(2,3,:) = 2.*(q(3,:).*q(4,:)-q(1,:).*q(2,:));
R(3,1,:) = 2.*(q(2,:).*q(4,:)-q(1,:).*q(3,:));
R(3,2,:) = 2.*(q(3,:).*q(4,:)+q(1,:).*q(2,:));
R(3,3,:) = 2.*q(1,:).^2-1+2.*q(4,:).^2;
end

