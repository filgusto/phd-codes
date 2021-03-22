classdef plot_control
    %PLOT_CONTROL
    % For plotting control variables
    
    properties
        h_fig;
        flag_plot_a;
        plot_a_index;
        plot_hor_size;
    end
    
    methods
        
        %% class constructor
        function this = plot_control()
            disp('plot_control class instantiated');
            
            % setting variables
            this.h_fig = {};
            this.flag_plot_a = 0;
            this.plot_hor_size = 20;
            
        end
        
        %% class destructor
        function delete(this)
           close(h_fig); 
        end
        
        %% plot a control data
        function this = plot_a(this, hist_q, hist_x, hist_xd, hist_e, hist_u)
            
            if this.flag_plot_a == 0
                
                % sets the flag
                this.flag_plot_a = 1;
                
                % creates a handler for the plot figure
                this.h_fig{end+1} = figure;
                this.plot_a_index = length(this.h_fig);
                
                % prepares the figure window
                figure(this.h_fig{this.plot_a_index});
                set(gcf, 'Position', [0 0 1000 1000]);
                
                subplot(2,1,1);
                title('Error Module');
                grid on;
                
                subplot(2,1,2);
                title('Control signal');
                grid on;
                
            end
            
            % selecting desired figure handler
            figure(this.h_fig{this.plot_a_index});
            
            % obtaining variables for plotting
            e_norm = this.mountErrorNormArray(hist_e, this.plot_hor_size);
            
            
            
            subplot(2,1,1);
            plot(e_norm);
%             
%             subplot(2,1,2);
%             plot(u);

            % forces drawing
            drawnow;
            
        end
        
        
    end % end of methods definition
    
    methods(Static)
        
        %% function for mouting e array
        function e_norm = mountErrorNormArray(hist_e,array_size)
                        
            % extracting a part of the total array size
            aux_e = {};
            if length(hist_e) < array_size
                aux_e = hist_e(:);
            else
                aux_e = hist_e(end-array_size+1:end);
            end
                
            % converting the cell into a matrix
            aux_e = cell2mat(aux_e);
            
            % computing the norm of each colum
            e_norm = vecnorm(aux_e);
            
        end
        
        %         function outputArg = method1(this,inputArg)
        %METHOD1 Summary of this method goes here
        %   Detailed explanation goes here
        %             outputArg = this.Property1 + inputArg;
        %         end
        
    end % end of static methods definition
end % end of class definition

