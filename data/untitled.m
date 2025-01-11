% Define the number of observations
numObservations = 17;

% Define the target point coordinates
targetX = 90;
targetY = 145;

% Define the variance
variance = 1;

% Generate random observations with the specified variance
observationsX = targetX + variance * randn(numObservations, 1);
observationsY = targetY + variance * randn(numObservations, 1);
observationsZ = 0

% Open a file to write the observations
fileID = fopen('C:/Users/User/OneDrive - University of Calgary/SUAV/Model/data/observations.txt', 'w');

% Write the observations to the file
for i = 1:numObservations
    fprintf(fileID, '%.2f %.2f\n', observationsY(i), observationsZ, observationsX(i));
end

% Close the file
fclose(fileID);

disp('Observations written to observations.txt');
