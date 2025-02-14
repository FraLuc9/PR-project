1;


function [K,T,z_near,z_far,width,height] = extractCamParams()
    printf("extracting camera data...\n")
    filename = "data/camera.dat";
    file = fopen(filename, "r");

    if file == -1
        error("Unable to open file");
    endif

    global K T z_near z_far width height;

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
                endfor
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
    endwhile

    fclose(file);
    printf("done!\n");

endfunction


function [odometry, gt_odom, XR, XR_true] = readTrajectory()
    printf("extracting odometry data...\n")
    filename = "data/trajectory.dat";
    file = fopen(filename, "r");

    if file == -1
        error("Unable to open file");
    endif

    odometry = zeros(3,1);
    gt_odom = zeros(3,1);
    i = 1;
    while ~feof(file)
        line = strsplit(fgetl(file));
        #loses 2 decimals worth of precision
        odometry(:,i) = str2double(line(2:4));
        gt_odom(:,i) = str2double(line(5:length(line)));
        i +=1;
    endwhile

    fclose(file);

    assert(length(odometry) == length(gt_odom))
    XR = zeros(4,4,length(odometry));
    XR_gt = zeros(4,4,length(odometry));
    for i=1:length(odometry)
        XR(:,:,i) = v2t(odometry(:,i));
        XR_true(:,:,i) = v2t(gt_odom(:,i));
    endfor

    printf("done!\n");

endfunction


% measurements(i)(j)(k) where i is the measurement, j is the measured point and k=1 gives the point id while k=2 gives the point coords
function measurements = extractMeasurements()
    printf("Extracting measurement data...\n")
    dpath = "data/";

    directory = dir(dpath);

    string = "meas";
    n = 0;
    for i = 1 : length(directory)
        if !isempty(regexp(directory(i).name, string, 'once'));
            n++;
        endif
    endfor

    measurements = containers.Map('KeyType', 'int32', 'ValueType', 'any');

    for i = 1 : n
        meas = containers.Map('KeyType', 'int32', 'ValueType', 'any');
        meas_str = "data/meas-00";
        if i-1 < 100
            meas_str = strcat(meas_str, "0");
            if i-1 < 10
                meas_str = strcat(meas_str, "0");
            endif
        endif

        meas_str = strcat(meas_str, int2str(i-1));
        meas_str = strcat(meas_str, ".dat");
        file = fopen(meas_str, "r");
        if file == -1
            error("Unable to open file")
        endif

        while ~feof(file)
            line = strsplit(fgetl(file));
            switch line{1}
            # Assuming data files formatted as given, seq always gets updated first
                case "point"
                    #updates data association
                    cur_id = str2num(line{2});
                    landmark_id = str2num(line{3});
                    points = str2double(line(4:5))';
                    meas(landmark_id) = [points];
            end
            
        endwhile

        fclose(file);
        measurements(i) = meas;
        
    endfor
    printf("done!\n");

endfunction


#returns matrix where every column represents a 3D landmark position
#ignores indices as it expects the .dat file to have them ordered already
function XL_gt = extractWorld()
    filename = "data/world.dat";
    Z = load(filename);
    XL_gt = Z'(2:4,:);

endfunction