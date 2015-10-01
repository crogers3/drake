classdef RigidBodyDiscretizedCylinder < RigidBodyCylinder
  
  methods 
    function obj = RigidBodyDiscretizedCylinder(cylinder,n)
      % obj = RigidBodyDiscretizedCylinder(cylinder,n) constructs a
      % RigidBodyCylinder object with the geometry-to-body transform set
      % to identity, and with n collosion points equally spaced along the
      % center of the cylinder.
      %
      % @param cylinder - a RigidBodyCylinder
      % @param n - the number of collision points to place along the center
      %            of the side of the cylinder
      obj = obj@RigidBodyCylinder(cylinder.radius, cylinder.len, cylinder.T);
      sizecheck(n,1);
      assert(n >= 0);
      obj.n = n;
    end
    
    function pts = getPoints(obj)
      [x,y,z] = pol2cart(linspace(0,2*pi,obj.n), obj.radius * ones(1,obj.n), zeros(1,obj.n));
      pts = obj.T(1:3,:) * [x;y;z;ones(1,obj.n)];
    end
    
    function pts = getTerrainContactPoints(obj)
      % pts = getTerrainContactPoints(obj) returns the terrain contact points
      % of this object.
      %
      % @param  obj - RigidBodyDiscretizedCylinder object
      % @retval pts - 3xm array of points on this geometry (in link frame) that
      %               can collide with the world.
      pts = getPoints(obj);
    end
  end
  
  properties
    n;
    pts;
  end
end
