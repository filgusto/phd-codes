classdef DualQuaternion 
    % Class for manipulating DualQuaternion objects
    
    properties
        q_p % quaternion primary part
        q_d % quaternion dual part
    end
    
    methods(Static)
    % static methods
        
        function r = adj(q, h)
        % adjoint operator
            r = q * h * q.conj;
        end
        
         function ret = pureRotation(varargin)
        % Sets a pure rotation dual quaternion
        % rot <quaternion, rotm(3x3)>: rotation object
            
            switch nargin
                
                case 1
                % in case of receiving directly a rotation object
                    
                    % mounts using a quaternion as input
                    if isa(varargin{1}, 'quaternion')
                        % mounts the dual quaternion
                        q_p = varargin{1}.normalize;
                        q_d = quaternion([0, 0, 0, 0]);
                        
                    % mounts using a rotation matrix as input
                    elseif isnumeric(varargin{1})
                        % checks size
                        [rot_m, rot_n] = size(varargin{1})
                        if rot_m == 3 & rot_n == 3 & det(varargin{1}) == 1
                            q_p = rotm2quat(varargin{1});
                            q_d = quaternion([0, 0, 0, 0]);;
                        else
                            error('dualquaternion:purerotation:invalidrotationmatrix','Invalid rotation matrix');
                        end
                    else
                        error('dualquaternion:purerotation:invalidinput','Invalid input. Insert a quaternion or rotation matrix');
                    end
            
                case 2
                % in case of receiving an angle and a rotation axis
                    if isnumeric(varargin{1}) & isnumeric(varargin{2}) & length(varargin{1} == 1) & length(varargin{2} == 3)
                        
                        % separating input for clarity
                        r_angle = varargin{1};
                        r_axis = varargin{2};
                        
                        % aux variable
                        sin_angle_div_2 = sin(r_angle/2);
                        
                        % mounting dual-quaternion
                        q_p = quaternion(cos(r_angle/2),...
                            r_axis(1) * sin_angle_div_2,...
                            r_axis(2) * sin_angle_div_2,...
                            r_axis(3) * sin_angle_div_2);
                        q_d = quaternion(0,0,0,0);
                    else
                       error('dualquaternion:purerotation:invalidinput','Invalid input considering an angle and an axis.'); 
                    end
                    
            end
            
            % returning object
            ret = DualQuaternion(q_p, q_d);           
            
        end
        
        function ret = pureTranslation(tr)
        % Returns a pure translation dual quaternion
        % tr <array(3)> = translation vector
            
            % mounts the dual quaternion objects
            q_p = quaternion(1, 0, 0, 0);
            q_d = 0.5 * quaternion(0, tr(1), tr(2), tr(3));
            
            % returns the object
            ret = DualQuaternion(q_p, q_d);
        end
        
        
        function ret = transform(rot, tr, order)
        % creates a relative pose dual quaternion
        % order<string>: transform order
        %   - rotfirst: rotates and then translates the frame
        %   - trfirst: translates then rotates the frame
            
            % creating the pure dual quaternion objects
            dq_rot = DualQuaternion.pureRotation(rot);
            dq_tr = DualQuaternion.pureTranslation(tr);
            
            if strcmp(order, 'rotfirst')
                ret = dq_rot * dq_tr;
            elseif strcmp(order,'trfirst');
                ret = dq_tr * dq_rot;
            else
                error('dualquaternion:wronginput','Wrong order assignment. define as rotfirst or trfirst');
            end
                
        end
        
    end % end of static methods
    
    methods
        
        %% === CONSTRUCTOR METHOD
        function obj = DualQuaternion(varargin)
            switch nargin
                
                % creating with no input
                case 0
                    obj.q_p = quaternion(1,0,0,0);
                    obj.q_d = quaternion(0,0,0,0);
                    
                % receiving a 8 position vector as input
                case 1
                    if isnumeric(varargin{1}) & length(varargin{1}) == 8
                        obj.q_p = quaternion(varargin{1}(1:4));
                        obj.q_d = quaternion(varargin{1}(5:8));
                    else
                        error('dualquaternion:wrongconstructorassignment''Bad DualQuaternion creation parameters.');
                    end
                    
                % receiving quaternions as input
                case 2
                    if isa(varargin{1}, 'quaternion') & isa(varargin{1}, 'quaternion')
                        obj.q_p = varargin{1};
                        obj.q_d = varargin{2};
                    else
                        error('dualquaternion:wrongconstructorassignment''Bad DualQuaternion creation parameters.');
                    end
            end   
        end
               
        %% === OPERATORS
        
        
        function r = mtimes(lhs, rhs)
        % Multiply operator for DualQuaternion objects    
          
            % in case of dual quaternion objects multiplications
            if isa(lhs, 'DualQuaternion') && isa(rhs, 'DualQuaternion')
                % multiplicating primary and dual quaternion parts
                r_p = lhs.q_p * rhs.q_p;
                r_d = (lhs.q_d * rhs.q_p) + (lhs.q_p * rhs.q_d);
                
                % mounting resulting object
                r = DualQuaternion(r_p, r_d);
            end
            
            % in case of dualquaternion and doublem multiplication
            if isa(lhs, 'DualQuaternion') && isnumeric(rhs)                
                r = DualQuaternion(lhs.q_p * rhs, lhs.q_d * rhs);
            end
            
            % in case of double and dual quaternion multiplication
            if isnumeric(lhs) && isa(rhs, 'DualQuaternion')
                r = DualQuaternion(rhs.q_p * lhs, rhs.q_d * lhs);
            end
        end % end of multiply operator
        
        
        function r = plus(lhs, rhs)
        % Sum operator for DualQuaternion objects
            r = DualQuaternion(lhs.q_p + rhs.q_p, lhs.q_d + rhs.q_d);
        end 
        
        
        function r = minus(lhs, rhs)
        % Minus operator for DualQuaternion objects
           
            % mounting resulting object
            r = DualQuaternion(lhs.q_p - rhs.q_p, lhs.q_d - rhs.q_d);            
        end
        
        
        function r = normalize(dq_in)
        % Normalizes the dual quaternion
            
           % computing the primary part quaternion magnitude
           mag = dot(dq_in.q_p.compact, dq_in.q_p.compact);
            
           % normalizing both primary and dual parts
           r_p = dq_in.q_p * (1/mag);
           r_d = dq_in.q_d * (1/mag);
           
           % mounting resulting object
           r = DualQuaternion(r_p, r_d);
        end
        
        function r = conj(obj)
        % Conjugate operator
            
            r = DualQuaternion(obj.q_p.conj, obj.q_d.conj);            
        end
        
        
        
        
        
        % returns the dual quaternion complex modulus (magnitude)
        function mag = mag(obj)
           
            % computes the magnitude of the dual quaternion
            mag = obj * obj.conj;
            
        end
                
        
        % Method for plotting multiple transforms
        function plot(dq_arr, size)
            
            % extracting translations and rotations
            o = [];
            t = [];
            for i=1:length(dq_arr)
                
               % orientation
               aux_o = dq_arr(i).q_p;
               o(i,:) = aux_o.compact;
               t(i,:) = dq_arr(i).extractTranslation;
            end
            
            % plotting transforms
            if nargin == 1  % if only frames are provided
                 plotTransforms(t, o);
            elseif nargin == 2    % if the frame plot size is provided
                plotTransforms(t, o, 'FrameSize', size);
            end
           
            % arrangements
            grid on;
            xlabel('x'); ylabel('y'); zlabel('z');
            axis equal
            
        end
        

       %% === CONVERSIONS AND EXTRACTIONS
       
       % Extracts the translation vector from the dual quaternion
       function tr = extractTranslation(obj)
           q_tr = (obj.q_d * 2) * (obj.q_p.conj);
           aux = q_tr.compact;
           tr = aux(2:end);
       end
       
       
       % Converts the pose dual-quaternion to homogeneous transform matrix
       function th = dq2th(obj)
          
           % normalizes the dq
           dq_n = obj.normalize;
           
           % extracts the rotation matrix
           rotm = quat2rotm(obj.q_p);
           
           % extracts the translation vector
           tr = obj.extractTranslation;
           
           % mounting the th matrix
           th = [[rotm,tr.'];[0, 0, 0, 1]];
       end
              
       
      %% === INTERFACES
     
      % overrides disp function
      function disp(obj)
          
          % compacting quaternion formats
          q_p = obj.q_p.compact;
          q_d = obj.q_d.compact;
          
          % rounding responses
          q_p = round(q_p, 6);
          q_d = round(q_d, 6);
          
          sig = {};
          
          for i=1:4
              if i==1 && q_p(i) == 0
                  sig{i} = '';
              elseif q_p(i) > 0
                  sig{i} = '+';
              else
                  sig{i} = '';
              end
          end
          
          for i=5:8
              if i==5 && q_d(i-4) == 0
                  sig{i} = '';
              elseif q_d(i-4) > 0
                  sig{i} = '+';
              else
                  sig{i} = '';
              end
          end
          
          % printing
          fprintf('(%c%.6f %c%.6f<strong>i</strong> %c%.6f<strong>j</strong> %c%.6f<strong>k</strong>) + (%c%.6f %c%.6f<strong>i</strong> %c%.6f<strong>j</strong> %c%.6f<strong>k</strong>)<strong>ε</strong> \n\n', ...
              sig{1}, q_p(1), sig{2}, q_p(2), sig{3}, q_p(3), sig{4}, q_p(4), ...
              sig{5}, q_d(1), sig{6}, q_d(2), sig{7}, q_d(3), sig{8}, q_d(4));
      end
      
    end
end





























