1;


function [K,T,z_near,z_far,width,height] = extractCamParams(filename)
    file = fopen("data/camera.dat", 'r');

    % Check if the file opened successfully
    if file == -1
        error('Unable to open file');
    end

    % Initialize variables to store the data
    K = zeros(3, 3);
    T = zeros(4, 4);
    z_near = 0;
    z_far = 0;
    width = 0;
    height = 0;

    while ~feof(file)
        line = fgetl(file);
        switch line
            case "camera matrix:"
                for i = 1:3
                    line = fgetl(file);

                    K(i,:) = str2num(line);
                end
            case "cam_transform:"
                for i = 1:4
                    line = fgetl(file);
                    T(i,:) = str2num(line);
                end
            otherwise
                array = strsplit(line, ":");
                switch array{1}
                    case "z_near"
                        z_near = str2num(array{2});
                    case "z_far"
                        z_far = str2num(array{2});
                    case "width"
                        width = str2num(array{2});
                    case "height"
                        height = str2num(array{2});
                end
        end        
    end

    fclose(file)
end


function odometry = readTrajectory(filename)
    file = fopen(filename, "r");

    if file == -1
        error('Unable to open file');
    end

    odometry = zeros(7,1)
    i = 1
    while ~feof(file)
        line = strsplit(fgetl(file));
        #loses 2 decimals worth of precision
        odometry(:,i) = str2double(line);
        i +=1;
    end
end


        
