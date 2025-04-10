#--------------------------------------------------Header-----------------------------------------------------
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt
from pyproj import CRS, Transformer
import json

def parametricModel(file_path):
    #--------------------------------------------------Functions-----------------------------------------------------
    # Load coordinates from file
    def parse_json_to_vectors(file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)

        # If the JSON is a dict, try to account for different top-level structures:
        # - If wrapped in a "points" key, then use that list.
        # - Otherwise, assume it is a single record and wrap it in a list.
        if isinstance(data, dict):
            if "points" in data:
                data = data["points"]
            else:
                data = [data]

        # Define the keys extracted
        keys = [
            "last_time", "lat", "lon", "rel_alt", "alt",
            "roll", "pitch", "yaw", "dlat", "dlon", "dalt", "heading", "num_satellites", 
            "position_uncertainty", "alt_uncertainty", "speed_uncertainty", "heading_uncertainty", 
            "x", "y"
        ]

        # Initialize an empty vector (list) for each key.
        vectors = {key: [] for key in keys}

        # Iterate through each data point and append values to appropriate vectors.
        for epoch in data:
            for key in keys:
                # Depending on your use case you might wish to validate that key exists.
                vectors[key].append(epoch[key])

        return vectors


    def lat_long_to_utm(lat, lon):
        """
        Convert geographic coordinates (latitude, longitude, and altitude) to UTM coordinates.
        
        Parameters:
        lat (float): Latitude in decimal degrees.
        lon (float): Longitude in decimal degrees.
        MSL (float): Altitude (e.g., Mean Sea Level height, usually in meters).
        
        Returns:
        (easting, northing, z): A three-element tuple where easting is easting (meters), northing is northing (meters),
                    and z is the altitude (meters).
        """
        # Define the geographic coordinate system (WGS84)
        wgs84 = CRS.from_epsg(4326)
        
        # Determine the UTM zone from the longitude
        # UTM zones are 6° wide; zone calculation: zone = int((lon + 180)/6) + 1
        zone = int((lon + 180) / 6) + 1
        
        # Determine the correct EPSG code for UTM based on hemisphere:
        # - Northern Hemisphere: EPSG:326XX (where XX is the zone)
        # - Southern Hemisphere: EPSG:327XX
        if lat >= 0:
            utm_epsg = 32600 + zone
        else:
            utm_epsg = 32700 + zone

        # Define UTM coordinate system for the calculated zone
        utm_crs = CRS.from_epsg(utm_epsg)
        
        # Create a Transformer from geographic coordinates (WGS84) to UTM
        transformer = Transformer.from_crs(wgs84, utm_crs, always_xy=True)
        
        # Transform (lon, lat), while passing MSL as the vertical coordinate.
        # Note: UTM is typically 2D but if you pass a third coordinate, it will usually apply an identity transformation on it.
        easting, northing = transformer.transform(lon, lat)
        
        return easting, northing, zone


    def utm_to_lat_long(easting, northing, zone, northern=True):
        """
        Convert UTM coordinates (easting, northing, and vert value) back to geographic coordinates (latitude and longitude).
        
        Parameters:
        easting (float): Easting in meters.
        northing (float): Northing in meters.
        vert (float): Vertical component (altitude), in meters.
        zone (int): UTM zone number (1 through 60).
        northern (bool): True if the coordinates are in the northern hemisphere; False for southern.
        
        Returns:
        (lat, lon, altitude): A tuple containing latitude, longitude (in decimal degrees), and altitude (in meters).
        """
        # Select the appropriate EPSG code for the UTM CRS:
        # For the northern hemisphere, the EPSG code is 32600 + zone,
        # For the southern hemisphere, it’s 32700 + zone.
        if northern:
            utm_epsg = 32600 + zone
        else:
            utm_epsg = 32700 + zone

        # Define the UTM coordinate system for the given zone.
        utm_crs = CRS.from_epsg(utm_epsg)
        
        # Define WGS84 geographic coordinate system.
        wgs84 = CRS.from_epsg(4326)
        
        # Create a Transformer to convert UTM coordinates back to WGS84.
        # Note: Transformer with always_xy=True expects the input order to be (easting, northing),
        # and returns coordinates in the order (lon, lat, altitude).
        transformer = Transformer.from_crs(utm_crs, wgs84, always_xy=True)
        
        # Perform the transformation.
        lon, lat = transformer.transform(easting, northing)
        
        return lat, lon


    # fiducial centre pix (728, 544)
    # x rotation(pitch)
    # y rotation(roll)
    # z rotation(yaw)
    # assuming y axis within image space is dirction of travel (provides greatest swath perpendicular to direction of travel)
    # assuming x axis within image space is perpendicular to direction of travel
    # assuming z axis within image space is down
    # if these changes aren't met, rotation squences will need to be changed
    # positive pitch raises nose of UAV
    # positive roll is right bank, i.e. right wing down
    # positive yaw is right turn, i.e. nose of UAV turns right relative to positive north
    def image_to_object_space(eastingDrone, northingDrone, AGL, xPix, yPix, Yaw, Pitch, Roll): 
        Yaw = np.deg2rad(Yaw)
        Pitch = np.deg2rad(Pitch)
        Roll = np.deg2rad(Roll)

        focalLength = 0.006 # meters (m)
        pixelSpacing = 0.00345 # mm per pix
        # fiducial centre (mm)
        xFiducial = 2.5116
        yFiducial = -1.8768

        Scale = AGL / focalLength

        # image x and y (mm)
        Image = np.array([(xPix * pixelSpacing) - xFiducial, ((-yPix * pixelSpacing) - yFiducial)])

        # object x and y in m relative to drone
        Object = np.array([((Scale * Image[0]) / 1000), ((Scale * Image[1]) / 1000), AGL])
        
        # y is direction of travel therefore conventional roll and pitch rotations are swapped
        # roll is rotation about y axis, pitch is rotation about x axis
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(Pitch), np.sin(Pitch)],
            [0, -np.sin(Pitch), np.cos(Pitch)]
        ])

        Ry = np.array([
            [np.cos(Roll), 0, -np.sin(Roll)],
            [0, 1, 0],
            [np.sin(Roll), 0, np.cos(Roll)]
        ])

        Rz = np.array([
            [np.cos(Yaw), np.sin(Yaw), 0],
            [-np.sin(Yaw), np.cos(Yaw), 0],
            [0, 0, 1]
        ])

        R = Rz @ Rx @ Ry

        # Target coordinates relative to drone position, not intersecting ground plane though
        # Therefore must compute intersection coordinates with ground plane by scaling by t
        target = R @ Object.transpose()
    

        verticalDepth = target[2]
        
        t = AGL / verticalDepth

        target[0] = target[0] * t
        target[1] = target[1] * t

        eastingTarget = eastingDrone + target[0]
        northingTarget = northingDrone + target[1]

        return eastingTarget, northingTarget


    # Functional parametric model
    def model(eastingDrone, northingDrone, eastingTarget, northingTarget):
        xDelta = eastingDrone - eastingTarget
        yDelta = northingDrone - northingTarget
        Obs = np.sqrt(xDelta**2 + yDelta**2)
        
        return Obs

    # df/dxTarget
    def fDxTarget(eastingDrone, northingDrone, eastingTarget, northingTarget):
        xDelta = eastingDrone - eastingTarget
        yDelta = northingDrone - northingTarget
        fDxTarget = -xDelta / np.sqrt(xDelta**2 + yDelta**2)
        
        return fDxTarget

    # df/dyTarget
    def fDyTarget(eastingDrone, northingDrone, eastingTarget, northingTarget):
        xDelta = eastingDrone - eastingTarget
        yDelta = northingDrone - northingTarget
        fDyTarget = -yDelta / np.sqrt(xDelta**2 + yDelta**2)
        
        return fDyTarget

    # Create covariance matrix of observations
    def covarianceMatrixObs(size):
        Cl = np.zeros((size, size))
        np.fill_diagonal(Cl, 1)

        return Cl

    def parametricAdjustment(eastingDrone, northingDrone, AGLDrone, eastingTarget, northingTarget):
        # Defining u(# of parameters) and n (# of observations)
        # Assuming number of observed horizontal distances is equal to n where a distance is calculating using the 2D vector equation (refer to function name 'model')
        u = 2
        n = eastingTarget.size

        # Degrees of freedom
        dof = n - u

        # Compute standard deviation of observations
        
        # Create covariance matrix
        Cl = covarianceMatrixObs(n)

        # Compute weight matrix P
        P = np.linalg.inv(Cl)

        # Declaring design matrix A and misclosure vector w dimensions
        A = np.zeros((n, u))
        w = np.zeros((n))

        # Declaring threshold vector (how much each parameter changes given an iteration)
        dHat = np.array([])

        # Estimating target coordinates by mean of target coordinate observations
        # Assuming flat plane of operation z-component will near 0m AGL
        eastingTargetEst = np.mean(eastingTarget)
        northingTargetEst = np.mean(northingTarget)

        # Outputs mean values
        print("\nEstimated mean prior to adjustment: ", "Easting: ", eastingTargetEst, "Northing: ", northingTargetEst)

        # Holds coordinate plot of mean target observations
        plt.scatter(eastingTargetEst, northingTargetEst, label = 'Mean Target Location')
        plt.scatter(eastingDrone, northingDrone, label = 'Drone Observation Points')
        plt.scatter(eastingTarget, northingTarget, label = 'Target Observations')
        plt.scatter(eastingTargetEst, northingTargetEst, label = 'Adjust Target Point')

        plt.title('Plot') 
        plt.xlabel('Easting') 
        plt.ylabel('Northing')
        plt.legend()
        plt.show()

        # Iteration count
        iteration = 0

        # Minimum threshold value before adjustment completes (m)
        threshold = 0.0001

        while iteration == 0 or abs(np.max(dHat)) > threshold:
            # Populating design matrix A
            for i in range(A.shape[0]):
                A[i, 0] = fDxTarget(eastingDrone[i], northingDrone[i], eastingTargetEst, northingTargetEst)
                A[i, 1] = fDyTarget(eastingDrone[i], northingDrone[i], eastingTargetEst, northingTargetEst)

            # Populating misclosure vector w (w = l - f(xo)) convention
            for i in range(w.size):
                w[i] = (model(eastingDrone[i], northingDrone[i], eastingTarget[i], northingTarget[i]) - model(eastingDrone[i], northingDrone[i], eastingTargetEst, northingTargetEst))


            N = (A.transpose()) @ P @ A
            U = (A.transpose()) @ P @ w
            dHat = (np.linalg.inv(N)) @ U

            eastingTargetEst += dHat[0]
            northingTargetEst += dHat[1]
            iteration += 1
            

        # Residuals
        vHat = A @ dHat + w

        # Computed post adjustment (ensures Cl is scaled properly)
        aPosterioriVarianceFactor = ((vHat.transpose()) @ P @ vHat ) / dof

        # Recomputing Cl to properly scale weight matrix to perform data snooping
        Cl = aPosterioriVarianceFactor * covarianceMatrixObs(n)
        
        # Compute weight matrix P
        P = np.linalg.inv(Cl)

        # Should be equal to or near 1 after rescaling (Ensures none bias data analysis)
        aPosterioriVarianceFactor = ((vHat.transpose()) @ P @ vHat ) / dof

        # Recomputed N
        N = (A.transpose()) @ P @ A

        CxHat = aPosterioriVarianceFactor * np.linalg.inv(N)


        ClHat = A @ CxHat @ A.transpose()


        CvHat = Cl - ClHat

        # Plot data and adjusted target point
        print("\nEstimated post adjustment: ", "Easting: ", eastingTargetEst, "Northing: ", northingTargetEst)

        plt.scatter(eastingDrone, northingDrone, label = 'Drone Observation Points')
        plt.scatter(eastingTarget, northingTarget, label = 'Target Observations')
        plt.scatter(eastingTargetEst, northingTargetEst, label = 'Adjust Target Point')
    
        for i, (x_val, y_val) in enumerate(zip(eastingDrone, northingDrone)): 
            plt.annotate(f'{i}', (x_val, y_val), textcoords="offset points", xytext=(0,10), ha='center')

        for i, (x_val, y_val) in enumerate(zip(eastingTarget, northingTarget)): 
            plt.annotate(f'{i}', (x_val, y_val), textcoords="offset points", xytext=(0,10), ha='center')

        plt.title('Plot') 
        plt.xlabel('Easting') 
        plt.ylabel('Northing')
        plt.legend()
        plt.show()


        # Data snooping
        standardizedVHat = vHat / np.sqrt(np.diag(CvHat))

        # Computes critical values of student-t distribution
        # Significance level (alpha)
        alpha = 0.05

        # Critical value for two-tailed test
        criticalValue = stats.t.ppf(1 - alpha/2, dof)


        if (np.max(standardizedVHat) > criticalValue) or (np.min(standardizedVHat) < -criticalValue):
            for i in range(standardizedVHat.size):
                if (standardizedVHat[i] > criticalValue) or (standardizedVHat[i] < -criticalValue):
                    eastingDrone = np.delete(eastingDrone, i)
                    northingDrone = np.delete(northingDrone, i)
                    AGLDrone = np.delete(AGLDrone, i)

                    eastingTarget = np.delete(eastingTarget, i)
                    northingTarget = np.delete(northingTarget, i)

            return parametricAdjustment(eastingDrone, northingDrone, AGLDrone, eastingTarget, northingTarget)
        
        else:
            return eastingTargetEst, northingTargetEst
        
    #--------------------------------------------------Main-----------------------------------------------------
    #--------------------------------------------------Loading Data-----------------------------------------------------
    vectors = parse_json_to_vectors(file_path)
    # "C:/Users/mfles/OneDrive - University of Calgary/SUAV/Model/test_data_points_with_pixels.json"
    eastingDrone = []
    northingDrone = []
    zoneDrone = []
    for lat, lon in zip(vectors['lat'], vectors['lon']):
        easting, northing, zone = lat_long_to_utm(lat, lon)

        eastingDrone.append(easting)
        northingDrone.append(northing)
        zoneDrone.append(zone)

    eastingTarget = []
    northingTarget = []
    AGLDrone = []

    for eastingDroneTemp, northingDroneTemp, AGL, xPix, yPix, Yaw, Pitch, Roll in zip(eastingDrone, northingDrone, vectors['rel_alt'], vectors['x'], vectors['y'], vectors['yaw'], vectors['pitch'], vectors['roll']):
        easting, northing = image_to_object_space(eastingDroneTemp, northingDroneTemp, AGL, xPix, yPix, Yaw, Pitch, Roll)
        eastingTarget.append(easting)
        northingTarget.append(northing)
        AGLDrone.append(AGL)

    #--------------------------------------------------Adjustment-----------------------------------------------------
    eastingTargetEst, northingTargetEst = parametricAdjustment(np.array(eastingDrone), np.array(northingDrone), np.array(AGLDrone), np.array(eastingTarget), np.array(northingTarget))

    zone = zoneDrone[0]

    # Assuming all drone observations are in the same zone, reasonable assumption for a small area.
    # Convert adjusted UTM coordinates back to latitude and longitude
    lat, lon = utm_to_lat_long(eastingTargetEst, northingTargetEst, zone, northern=True)

    #--------------------------------------------------Output-----------------------------------------------------
    return lat, lon