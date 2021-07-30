
clc
close all
clear all
video = VideoReader('project_video.mp4');
load('roi_variables', 'c', 'r');


while hasFrame(video)    
    frame = readFrame(video);    
    frame = imgaussfilt3(frame);
    %figure('Name','Filtered Image'), imshow(frame);
    redYmin = 130;
    redYmax = 255;
    greenYmin = 130;
    greenYmax = 255;
    blueYmin = 0;
    blueYmax = 130;
    Yellow=((frame(:,:,1)>=redYmin)|(frame(:,:,1)<=redYmax))& ...
        (frame(:,:,2)>=greenYmin)&(frame(:,:,2)<=greenYmax)&...
        (frame(:,:,3)>=blueYmin)&(frame(:,:,3)<=blueYmax);    
    %figure('Name','Yellow Mask'), imshow(Yellow);
    
    redWmin = 200;
    redWmax = 255;
    greenWmin = 200;
    greenWmax = 255;
    blueWmin = 200;
    blueWmax = 255;
    White=((frame(:,:,1)>=redWmin)|(frame(:,:,1)<=redWmax))&...
        (frame(:,:,2)>=greenWmin)&(frame(:,:,2)<=greenWmax)& ...
        (frame(:,:,3)>=blueWmin)&(frame(:,:,3)<=blueWmax);    
   %figure('Name','White Mask'), imshow(White);

    frameW = edge(White, 'canny', 0.2);
    frameY = edge(Yellow, 'canny', 0.2);
    
    frameY = bwareaopen(frameY,15);
    frameW = bwareaopen(frameW,15);
    
    %figure('Name','Detecting Edges of Yellow mask'), imshow(frameY);
    %figure('Name','Detecting Edges of White mask'), imshow(frameW);

    roiY = roipoly(frameY, r, c);
    [R , C] = size(roiY);
    for i = 1:R
        for j = 1:C
            if roiY(i,j) == 1
                frame_roiY(i,j) = frameY(i,j);
            else
                frame_roiY(i,j) = 0;  
            end
        end
    end  
    %figure('Name','Filtering ROI from Yellow mask'), imshow(frame_roiY);
  
    roiW = roipoly(frameW, r, c);
    [R , C] = size(roiW);
    for i = 1:R
        for j = 1:C
            if roiW(i,j) == 1
                frame_roiW(i,j) = frameW(i,j);
            else
                frame_roiW(i,j) = 0;  
            end
        end
    end  
    
    %figure('Name','Filtering ROI from White mask'), imshow(frame_roiW);
        
    [H_Y,theta_Y,rho_Y] = hough(frame_roiY);
    [H_W,theta_W,rho_W] = hough(frame_roiW);
    
    P_Y = houghpeaks(H_Y,2,'threshold',2);
    P_W = houghpeaks(H_W,2,'threshold',2);
    
    lines_Y = houghlines(frame_roiY,theta_Y,rho_Y,P_Y,'FillGap',3000,'MinLength',20);  
    lines_W = houghlines(frame_roiW,theta_W,rho_W,P_W,'FillGap',3000,'MinLength',20);
       
    %Extract start and end points of lines
    
    leftp1 = [lines_Y(1).point1; lines_Y(1).point2];
    leftp2 = [lines_Y(2).point1; lines_Y(2).point2];  
    
    rightp1 = [lines_W(1).point1; lines_W(1).point2];
    rightp2 = [lines_W(2).point1; lines_W(2).point2];
    
    if leftp1(1,1) < leftp2(1,1)
        left_plot(1,:) = leftp1(1,:);
    else
        left_plot(1,:) = leftp2(1,:);
    end
    
    if leftp1(2,2) < leftp2(2,2)
        left_plot(2,:) = leftp1(2,:);
    else
        left_plot(2,:) = leftp2(2,:);
    end
    
    if rightp1(1,2) < rightp2(1,2)
        right_plot(1,:) = rightp1(1,:);
    else
        right_plot(1,:) = rightp2(1,:);
    end
    
    if rightp1(2,1) > rightp2(2,2)
        right_plot(2,:) = rightp1(2,:);
    else
        right_plot(2,:) = rightp2(2,:);
    end
    
    %Calculate slope of left and right lines
    
    slopeL = (left_plot(2,2)-left_plot(1,2))/(left_plot(2,1)-left_plot(1,1));
    slopeR = (right_plot(2,2)-right_plot(1,2))/(right_plot(2,1)-right_plot(1,1));

    %Make equations of left and right lines to extrapolate them
    
    xLeftY = 1; 
    yLeftY = slopeL * (xLeftY - left_plot(1,1)) + left_plot(1,2);
    xRightY = 550; 
    yRightY = slopeL * (xRightY - left_plot(2,1)) + left_plot(2,2);
    
    xLeftW = 750; 
    yLeftW = slopeR * (xLeftW - right_plot(1,1)) + right_plot(1,2);
    xRightW = 1300; 
    yRightW = slopeR * (xRightW - right_plot(2,1)) + right_plot(2,2);
    
    %Making a transparent Trapezoid between 4 poits of 2 lines
    
    points = [xLeftY yLeftY; xRightY yRightY ;xLeftW yLeftW; xRightW yRightW ];
    number = [1 2 3 4];
    %Turn Prediction
    
    Yellow_dir = cross([left_plot(1,1), left_plot(1,2), 1], [left_plot(2,1), left_plot(2,2), 1]);
    Yellow_dir = Yellow_dir ./ sqrt(Yellow_dir(1)^2 + Yellow_dir(2)^2);
    theta_y = atan2(Yellow_dir(2), Yellow_dir(1));
    rho_y = Yellow_dir(3);
    yellow_line = [cos(theta_y), sin(theta_y), rho_y];
    
    %Finding vanishing point using cross poduct
    white_dir = cross([right_plot(1,1),right_plot(1,2),1], [right_plot(2,1),right_plot(2,2),1]);
    white_dir = white_dir ./ (sqrt(white_dir(1)^2 + white_dir(2)^2));
    theta_w = atan2(white_dir(2),white_dir(1));
    rho_w = white_dir(3);
    white_line = [cos(theta_w), sin(theta_w), rho_w];
    

    vanishing_point = cross(yellow_line, white_line);
    vanishing_point = vanishing_point ./ vanishing_point(3);
    vanishing_ratio = vanishing_point(1) / size(frame, 2);
    
    if vanishing_ratio < 0.49 && vanishing_ratio >=0.47
        direction = 'Turn Left';
    elseif vanishing_ratio >= 0.49 && vanishing_ratio <= 0.51
        direction = 'Go Straight';
    else
        direction = 'Turn Right';
    end
      
    %Plot the extrapolated lines, Trapezoid and direction on each frame
    
    imshow(frame);
    hold on
    plot([xLeftY, xRightY], [yLeftY, yRightY], 'LineWidth',8,'Color','red');
    plot([xLeftW, xRightW], [yLeftW, yRightW], 'LineWidth',8,'Color','red');
    text(650, 65, direction,'horizontalAlignment', 'center', 'Color','red','FontSize',20)
    patch('Faces', number, 'Vertices', points, 'FaceColor','green','Edgecolor','green','FaceAlpha',0.4)
    hold off
    pause(0.05)
    
end