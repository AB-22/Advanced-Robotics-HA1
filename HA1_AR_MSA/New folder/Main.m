% This Assignment was done by:
% Suliman Badour , Amer Al Badr,  Mohamad Al Madfaa


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% elastic joint +rigid link+ passive joint  + flexible link + passive joint +flexible
% link + passive joint+ rigid link 


% woking area
x = 200:10:250;
y = 200:10:250;
z = 200:10:250;

% applied forces
F = [100,100,100,0,0,0];

% tensor with deflections
graph = zeros(length(x),length(y),length(z));

for i=1:length(x)
    for j = 1:length(y)
        for k = 1:length(z)
            
            end_effector = [x(i)/1000,y(j)/1000,z(k)/1000,0,0,0];
  
%             calculating stiffness of each leg for given end effector
%             position
            k1 = calculate_MSA('x',end_effector);
            k2 = calculate_MSA('y',end_effector);
            k3 = calculate_MSA('z',end_effector);
            
%             stiffness matrix for the whole robot equals sum of stiffness
%             matrices for each leg
            K = k1 + k2 + k3;
            
%             F = Kc * delta_t => delta_t = inv(Kc)*F 
% deflections
            dt=inv(K)*F';
            dr=sqrt(dt(1)^2+dt(2)^2 + dt(3)^2);
            graph(i,j,k) = dr;
        end
    end
end

%% here is only plotting part 
[x,y,z] = meshgrid(1:6,1:6,1:6);

figure
scatter3(x(:),y(:),z(:),500,graph(:),'filled');
c=colorbar;
c.Label.String='\bf \Delta t'
title('Deflection Map')
