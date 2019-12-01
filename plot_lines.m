function [saved_data, time] = plot_lines(baudrate, num_lines, ylowerlim, yupperlim)

if ~isempty(instrfind)
    fclose(instrfind);
    delete(instrfind);
end

port = 'COM7';
s=serial(port);
set(s,'BaudRate',baudrate);
fopen(s);


f = figure(1);
H = uicontrol();
H.Position = [0,0,0,0];

for i = 1:num_lines
    anim_line(i) = animatedline;
end

anim_line(1) = animatedline('Color','b');
if(num_lines >= 3)
    anim_line(2) = animatedline('Color',[.3 0 0]);
    anim_line(3) = animatedline('Color', 'r');
end


grid on;
tic;
buf_width = 500;

linebuf = zeros(num_lines,buf_width);
timebuf = zeros(num_lines,buf_width);

ylim([ylowerlim,yupperlim]);
% xlim([-1,1]);
xadv = 0;
tic;
while(ishandle(H))
    %get new value
%     v = sin(t);
%     fprintf('%f\r\n',t);

    data = fread(s,num_lines,'single');
    if(isempty(data) ~= 1 && size(data,1) == num_lines)
        set(gcf,'Renderer','OpenGL');
        for i = 1:num_lines

            %buffer new value. may be built into line structure
            linebuf(i,2:end) = linebuf(i, 1:end-1);
            linebuf(i,1) = data(i);
                        
            clearpoints(anim_line(i));
            addpoints(anim_line(i), 1:buf_width, linebuf(i,:));

        end
        
    end
    timebuf(2:end) = timebuf(1:end-1);
    timebuf(1) = toc;
    
    drawnow limitrate     
    
        
    pause(0.000000001);
end

fclose(s);

saved_data = linebuf;
time = timebuf;
end
