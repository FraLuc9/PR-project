1;

source "dataExtractUtils.m"

function n = countMeasurements()
    dpath = "data/";

    directory = dir(dpath);

    string = "meas";
    n = 0;
    for i = 1 : length(directory)
        if !isempty(regexp(directory(i).name, string, 'once'));
            n++;
        end
    end
endfunction

#measurements and data association as cell arrays
#each index represents data from a single step
function [da, meas, n_measurements] = extractAllMeasurements()
    n_measurements = countMeasurements();

    da = cell(1, n_measurements);
    meas = cell(1, n_measurements);

    for i = 1 : n_measurements
        stringa = "data/meas-00";
        if i-1 < 100
            stringa = strcat(stringa, "0");
            if i-1 < 10
                stringa = strcat(stringa, "0");
            end
        end
        stringa = strcat(stringa, int2str(i-1));
        stringa = strcat(stringa, ".dat");
        [_, da{i}, meas{i}] = extractMeasurements(stringa);
    end
endfunction