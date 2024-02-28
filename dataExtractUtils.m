1;


function [K,T,z_near,z_far,width,height] = extractCamParams(filename)
    file = fopen(filename, "r");

    if file == -1
        error("Unable to open file");
    end

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

    fclose(file);
endfunction


function odometry = readTrajectory(filename)
    file = fopen(filename, "r");

    if file == -1
        error("Unable to open file");
    end

    # let's not count the ids as odometry measurements should be ordered
    #odometry = zeros(7,1);
    odometry = zeros(6,1);
    i = 1;
    while ~feof(file)
        line = strsplit(fgetl(file));
        #loses 2 decimals worth of precision
        odometry(:,i) = str2double(line(2:length(line)));
        i +=1;
    end

    fclose(file);
endfunction


# by cycling through all the measurements files it puts together 
# a big matrix with every landmark measurement in the image frame
function [pose, da, points] = extractMeasurements(filename)
            
    seq = 0;
    pose = zeros(6,1);
    
    file = fopen(filename, "r");
    das = zeros(3,1);
    points = zeros(2,1);
    if file == -1
        error("Unable to open file")
    end
    i = size(points, 2) + 1;
    while ~feof(file)
        line = strsplit(fgetl(file));
        switch line{1}
        # Assuming data files formatted as given, seq always gets updated first
            case "seq:"
                seq = str2num(line{2});
            case "gt_pose:"
                #inserting ground truth pose in the second half of the array so to maintain consistency with the odometry readings
                pose(4:6) = str2double(line(2:length(line)));
            case "odom_pose:"
                #of course noisy odometry goes in the first 3 positions
                pose(1:3) = str2double(line(2:4));
            case "point"
                #updates data association
                da(1,i) = seq;
                da(2,i) = str2num(line{2});
                da(3,i) = str2num(line{3});
                points(:,i) = str2double(line(4:5));
                i +=1;
        end
    end

    fclose(file);
endfunction



#returns matrix where every column represents a 3D landmark position
#ignores indices as it expects the .dat file to have them ordered already
function P = extractWorld(filename)

    Z = load(filename);
    P = Z'(2:4,:);

endfunction