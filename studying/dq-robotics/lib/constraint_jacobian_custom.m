function J = constraint_jacobian_custom(q, wheel_radius, distance_between_wheels)
            % J = constraint_jacobian(phi) returns the Jacobian matrix that
            % satisfies [x_dot, y_dot, phi_dot]' = J*[wr,wl]', where wr and
            % wl are the angular velocities of the right and left wheels,
            % respectively.
            % phi is the robot orientation angle
            r = wheel_radius;
            l = distance_between_wheels;
            c = cos(q(3));
            s = sin(q(3));
            
            J = [(r/2)*c, (r/2)*c;
                 (r/2)*s, (r/2)*s;
                  r/l, -r/l
                ];
        end