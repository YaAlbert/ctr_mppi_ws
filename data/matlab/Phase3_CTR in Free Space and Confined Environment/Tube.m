% This function defines the physical parameters of tubes
classdef Tube < handle

    properties
        id = 0 % inner diameter
        od = 0 % outer diameter
        r = 0  % radius of curvature
        k = 0  % curvature
        l = 0  % total length of the tube (straight + curved)
        d = 0  % length of the curved section of tube
        E = 0  % Young's modulus
        I = 0  % Second moment of area 
        J = 0  % Polar moment
        G = 0  % Shear modulus

    end

    methods
        function self = Tube(id, od, r, l, d, E)
            self.id = id;
            self.od = od;
            self.r = r;
            self.k = 1/r;
            self.l = l;
            self.d = d;
            self.E = E;
            self.I = (pi/64)*(od^4 - id^4);
            self.J = 2*self.I;
            self.G = self.E/(2 * (1 + 0.217));

        end

        function params = get_tube_params(self)
            params = [self.id, self.od, self.r, self.k, self.d, self.E, self.I];
        end
    end
end