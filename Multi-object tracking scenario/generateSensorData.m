scenario = drivingScenario;
scenario.SampleTime = 0.01;
% Add all road segments
roadCenters = [46.3 10.6 0;
    33.8 -13.4 0;
    8.2 -13.2 0;
    19.8 11 0;
    46.3 10.6 0;
    46.3 10.6 0];
laneSpecification = lanespec(2, 'Width', 2.925);
road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego vehicle
egoCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [16.6 11 0]);
waypoints = [16.6 11 0;
    20.6 13.3 0;
    27.2 16.6 0;
    33.2 18 0;
    40.1 17.2 0;
    47.1 12.5 0;
    49.5 5.5 0;
    48.5 -0.7 0;
    43.7 -8.2 0;
    35.1 -14.8 0;
    27.2 -18.6 0;
    19.1 -20.1 0;
    10.9 -17.9 0;
    6.3 -13 0;
    4.8 -5.5 0;
    7.4 1.9 0;
    11.4 6.6 0];
speed = 32;
trajectory(egoCar, waypoints, speed);

% Add the non-ego actors
leadCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [8.7 3.9 0]);
waypoints = [8.7 3.9 0;
    14.6 9.7 0;
    20.8 13.6 0;
    25.6 16.1 0;
    33.7 18.3 0;
    40.4 17.6 0;
    47.3 12.7 0;
    49.7 5.6 0;
    48.7 -0.8 0;
    43.9 -8 0;
    35.2 -15 0;
    27.4 -18.5 0;
    19.2 -19.8 0;
    11.1 -17.9 0;
    6.3 -12.8 0;
    4.9 -6.1 0;
    6.4 0.1 0];
speed = 31;
trajectory(leadCar, waypoints, speed);

chaseCar = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [27 16.4 0]);
waypoints = [27 16.4 0;
    33.4 18.1 0;
    40.1 17.5 0;
    47.4 12.5 0;
    49.7 5.4 0;
    48.6 -0.9 0;
    43.7 -8.1 0;
    34.7 -14.9 0;
    27.3 -18.5 0;
    19 -20 0;
    11.2 -17.9 0;
    6.2 -12.8 0;
    4.6 -5.6 0;
    7.4 1.9 0;
    11.4 6.5 0;
    13.4 8.4 0;
    20.6 13.4 0;
    24.8 15.6 0];
speed = 30;
trajectory(chaseCar, waypoints, speed);
sensors = cell(4,1);
sensors{1} = radarDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [3.7 0], ...
    'MaxRange', 100, ...
    'ActorProfiles', profiles);
sensors{3} = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [0.99 0], ...
    'Yaw', 360, ...
    'DetectorOutput', 'Objects only', ...
    'ActorProfiles', profiles);
sensors{2} = radarDetectionGenerator('SensorIndex', 3, ...
    'SensorLocation', [-1 0], ...
    'Yaw', -180, ...
    'MaxRange', 100, ...
    'ActorProfiles', profiles);
sensors{4} = visionDetectionGenerator('SensorIndex', 4, ...
    'SensorLocation', [0 0], ...
    'Yaw', -180, ...
    'DetectorOutput', 'Objects only', ...
    'ActorProfiles', profiles);
numSensors = 4;
profiles = actorProfiles(scenario);
for m = 1:numel(sensors)
    sensors{m}.ActorProfiles = profiles;
end
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

% Create the display and return a handle to the bird's-eye plot
BEP = createDemoDisplay(egoCar, sensors);

toSnap = true;
while advance(scenario) && ishghandle(BEP.Parent)
    % Get the scenario time
    time = scenario.SimulationTime;

    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoCar);

    % Simulate the sensors
    detections = {};
    isValidTime = false(1,4);
    for i = 1:4
        [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            for j = 1:numValidDets
               
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end
            end
            detections = [detections; sensorDets];   
        end   
        
    end
    % Update the tracker if there are new detections
    if any(isValidTime)
        vehicleLength = sensors{1}.ActorProfiles.Length;
        detectionClusters = clusterDetections(detections, vehicleLength);
        confirmedTracks = updateTracks(tracker, detectionClusters, time);
updateBEP(BEP, egoCar, detections, confirmedTracks, positionSelector, velocitySelector);
    end
if ta(1).Position(1) > 0 && toSnap
        toSnap = false;
        snapnow
end
end
function filter = initSimDemoFilter(detection)
% Use a 2-D constant velocity model to initialize a trackingKF filter.
% The state vector is [x;vx;y;vy]
% The detection measurement vector is [x;y;vx;vy]
% As a result, the measurement model is H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
filter = trackingKF('MotionModel', '2D Constant Velocity', ...
    'State', H' * detection.Measurement, ...
    'MeasurementModel', H, ...
    'StateCovariance', H' * detection.MeasurementNoise * H, ...
    'MeasurementNoise', detection.MeasurementNoise);
end

function detectionClusters = clusterDetections(detections, vehicleSize)
N = numel(detections);
distances = zeros(N);
for i = 1:N
    for j = i+1:N
        if detections{i}.SensorIndex == detections{j}.SensorIndex
            distances(i,j) = norm(detections{i}.Measurement(1:2) - detections{j}.Measurement(1:2));
        else
            distances(i,j) = inf;
        end
    end
end
leftToCheck = 1:N;
i = 0;
detectionClusters = cell(N,1);
while ~isempty(leftToCheck)
    % Remove the detections that are in the same cluster as the one under
    % consideration
    underConsideration = leftToCheck(1);
    clusterInds = (distances(underConsideration, leftToCheck) < vehicleSize);
    detInds = leftToCheck(clusterInds);
    clusterDets = [detections{detInds}];
    clusterMeas = [clusterDets.Measurement];
    meas = mean(clusterMeas, 2);
    meas2D = [meas(1:2);meas(4:5)];
    i = i + 1;
    detectionClusters{i} = detections{detInds(1)};
    detectionClusters{i}.Measurement = meas2D;
    leftToCheck(clusterInds) = [];
end
detectionClusters(i+1:end) = [];

% Since the detections are now for clusters, modify the noise to represent
% that they are of the whole car
for i = 1:numel(detectionClusters)
    measNoise(1:2,1:2) = vehicleSize^2 * eye(2);
    measNoise(3:4,3:4) = eye(2) * 100 * vehicleSize^2;
    detectionClusters{i}.MeasurementNoise = measNoise;
end
end

function BEP = createDemoDisplay(egoCar, sensors)
    % Make a figure
    hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top

    % Add a car plot that follows the ego vehicle from behind
    hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot);

    % Add a car plot that follows the ego vehicle from a top view
    hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
    hCarPlot = axes(hTopViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);

    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');

    % Create bird's-eye plot for the ego vehicle and sensor coverage
    hBEVPlot = axes(hBEVPanel);
    frontBackLim = 60;
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);

    % Plot the coverage areas for radars
   
        cap = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
        plotCoverageArea(cap, sensors{1}.SensorLocation,...
            sensors{1}.MaxRange, sensors{1}.Yaw, sensors{1}.FieldOfView(1));
    

    % Plot the coverage areas for vision sensors
        cap = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
        plotCoverageArea(cap, sensors{2}.SensorLocation,...
            sensors{2}.MaxRange, sensors{2}.Yaw, 45);
   

    % Create a vision detection plotter put it in a struct for future use
    detectionPlotter(BEP, 'DisplayName','vision', 'MarkerEdgeColor','blue', 'Marker','^');

    % Combine all radar detections into one entry and store it for later update
    detectionPlotter(BEP, 'DisplayName','radar', 'MarkerEdgeColor','red');

    % Add road borders to plot
    laneMarkingPlotter(BEP, 'DisplayName','lane markings');

    % Add the tracks to the bird's-eye plot. Show last 10 track updates.
    trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);

    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-40 40]);

    % Add an outline plotter for ground truth
    outlinePlotter(BEP, 'Tag', 'Ground truth');
    
    
    

end
function updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel)
    % Update road boundaries and their display
    [lmv, lmf] = laneMarkingVertices(egoCar);
    plotLaneMarking(findPlotter(BEP,'DisplayName','lane markings'),lmv,lmf);

    % update ground truth data
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);

    % Prepare and update detections display
    N = numel(detections);
    detPos = zeros(N,2);
    isRadar = true(N,1);
    for i = 1:N
        detPos(i,:) = detections{i}.Measurement(1:2)';
        if detections{i}.SensorIndex > 2 % Vision detections
            isRadar(i) = false;
        end
    end
    plotDetection(findPlotter(BEP,'DisplayName','vision'), detPos(~isRadar,:));
    plotDetection(findPlotter(BEP,'DisplayName','radar'), detPos(isRadar,:));

    % Prepare and update tracks display
    trackIDs = {confirmedTracks.TrackID};
    labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
    [tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
    tracksVel = getTrackVelocities(confirmedTracks, vsel);
    plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, tracksVel, tracksCov, labels);
end