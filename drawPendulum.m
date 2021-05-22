function drawPendulum(u,L,gap,width,height)

    % process inputs to function
    theta1        = u(1);
    theta2    = u(2);
    t        = u(3);
    y = 0;
    % define persistent variables 
    persistent base_handle
    persistent rod_handle
    persistent rod2_handle
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        track_width=3;
        plot([-track_width,track_width],[0,0],'k'); % plot track
        hold on
        base_handle = drawBase(y, width, height, gap, [], 'normal');
        rod_handle  = drawRod(y, theta1, L, gap, height, [], 'normal');
        rod2_handle  = drawRod2(y, theta1, theta2, L, gap, height, [], 'normal');
        axis([-track_width, track_width, -L, 2*track_width-L]);
    
        
    % at every other time step, redraw base and rod
    else 
        drawBase(y, width, height, gap, base_handle);
        drawRod(y, theta1, L, gap, height, rod_handle);
        drawRod2(y, theta1, theta2, L, gap, height, rod2_handle);
    end
end

   
%
%=======================================================================
% drawBase
% draw the base of the pendulum
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawBase(y, width, height, gap, handle, mode)
  
  X = [y-width/2, y+width/2, y+width/2, y-width/2];
  Y = [gap, gap, gap+height, gap+height];

  if isempty(handle),
    handle = fill(X,Y,'m', 'EraseMode', mode);
    %handle = plot(X,Y,'m', 'EraseMode', mode);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
 
%
%=======================================================================
% drawRod
% draw the pendulum rod
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawRod(y, theta, L, gap, height, handle, mode)

  
  X = [y, y+L*sin(theta)];
  Y = [gap+height, gap + height + L*cos(theta)];

  if isempty(handle),
    handle = plot(X, Y, 'g', 'EraseMode', mode);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end

%
%=======================================================================
% drawRod2
% draw the pendulum rod
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawRod2(y, theta1, theta2, L, gap, height, handle, mode)

  
  X = [y+L*sin(theta1),y+L*sin(theta1)+L*sin(theta2)];
  Y = [gap + height + L*cos(theta1),gap + height + L*cos(theta1) + L*cos(theta2)];

  if isempty(handle),
    handle = plot(X, Y, 'g', 'EraseMode', mode);
  else
    set(handle,'XData',X,'YData',Y);
    drawnow
  end
end
  