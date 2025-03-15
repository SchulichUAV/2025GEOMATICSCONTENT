#--------------------------------------------------Header-----------------------------------------------------
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt
from pyproj import CRS, Transformer
import json

#--------------------------------------------------Functions-----------------------------------------------------
def lat_long_to_utm(lat, lon, MSL):
    """
    Convert geographic coordinates (latitude, longitude, and altitude) to UTM coordinates.
    
    Parameters:
      lat (float): Latitude in decimal degrees.
      lon (float): Longitude in decimal degrees.
      MSL (float): Altitude (e.g., Mean Sea Level height, usually in meters).
    
    Returns:
      (x, y, z): A three-element tuple where x is easting (meters), y is northing (meters),
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
    x, y, z = transformer.transform(lon, lat, MSL)
    
    return x, y, z, zone


from pyproj import CRS, Transformer

def utm_to_lat_long(easting, northing, up, zone, northern=True):
    """
    Convert UTM coordinates (easting, northing, and up value) back to geographic coordinates (latitude and longitude).
    
    Parameters:
      easting (float): Easting in meters.
      northing (float): Northing in meters.
      up (float): Vertical component (altitude), in meters.
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
    lon, lat, MSL = transformer.transform(easting, northing, up)
    
    return lat, lon, MSL


def image_to_object_space(xDrone, yDrone, AGL, xPix, yPix, Yaw):
    focalLength = 0.012

    Scale = AGL / focalLength

    pixelSpacing = 0.3528 # mm per pix

    #mm
    xFiducial = 256.838
    yFiducial = -191.923

    # image x and y in mm
    Image = np.array([(xPix * pixelSpacing) - xFiducial, ((-yPix * pixelSpacing) - yFiducial)])

    # object x and y in m relatvie to drone
    Object = np.array([((Scale * Image[0]) / 1000), ((Scale * Image[1]) / 1000)])
    
    rotationYaw = np.array([
        [np.cos(Yaw), -np.sin(Yaw)],
        [np.sin(Yaw), np.cos(Yaw)]
    ])

    Target = rotationYaw @ Object.transpose()

    xTarget = xDrone + Target[0]
    yTarget = yDrone + Target[1]

    return xTarget, yTarget



# Functional parametric model
def model(xCoordsDrone, yCoordsDrone, xCoordsTarget, yCoordsTarget):
    xDelta = xCoordsDrone - xCoordsTarget
    yDelta = yCoordsDrone - yCoordsTarget
    Obs = np.sqrt(xDelta**2 + yDelta**2)
    
    return Obs


# df/dxTarget
def fDxTarget(xCoordsDrone, yCoordsDrone, xCoordsTarget, yCoordsTarget):
    xDelta = xCoordsDrone - xCoordsTarget
    yDelta = yCoordsDrone - yCoordsTarget
    fDxTarget = -xDelta / np.sqrt(xDelta**2 + yDelta**2)
    
    return fDxTarget


# df/dyTarget
def fDyTarget(xCoordsDrone, yCoordsDrone, xCoordsTarget, yCoordsTarget):
    xDelta = xCoordsDrone - xCoordsTarget
    yDelta = yCoordsDrone - yCoordsTarget
    fDyTarget = -yDelta / np.sqrt(xDelta**2 + yDelta**2)
    
    return fDyTarget


# Load coordinates from file
def parse_json_to_vectors(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    # Initialize vectors for each parameter type
    x_pixel_vector = []
    y_pixel_vector = []
    last_time_vector = []
    lat_vector = []
    lon_vector = []
    rel_alt_vector = []
    alt_vector = []
    roll_vector = []
    pitch_vector = []
    yaw_vector = []
    dlat_vector = []
    dlon_vector = []
    dalt_vector = []
    heading_vector = []

    # Iterate through each data point and append values to respective vectors
    for point in data:
        x_pixel_vector.append(point["x_pixel"])
        y_pixel_vector.append(point["y_pixel"])
        last_time_vector.append(point["last_time"])
        lat_vector.append(point["lat"])
        lon_vector.append(point["lon"])
        rel_alt_vector.append(point["rel_alt"])
        alt_vector.append(point["alt"])
        roll_vector.append(point["roll"])
        pitch_vector.append(point["pitch"])
        yaw_vector.append(point["yaw"])
        dlat_vector.append(point["dlat"])
        dlon_vector.append(point["dlon"])
        dalt_vector.append(point["dalt"])
        heading_vector.append(point["heading"])

    return {
        "x_pixel": x_pixel_vector,
        "y_pixel": y_pixel_vector,
        "last_time": last_time_vector,
        "lat": lat_vector,
        "lon": lon_vector,
        "rel_alt": rel_alt_vector,
        "alt": alt_vector,
        "roll": roll_vector,
        "pitch": pitch_vector,
        "yaw": yaw_vector,
        "dlat": dlat_vector,
        "dlon": dlon_vector,
        "dalt": dalt_vector,
        "heading": heading_vector
    }


# Create covariance matrix of observations
def covarianceMatrixObs(size):
    Cl = np.zeros((size, size))
    np.fill_diagonal(Cl, 1)

    return Cl


def parametricAdjustment(xCoordsDrone, yCoordsDrone, zCoordsDrone, xCoordsTarget, yCoordsTarget):
    # Defining u(# of parameters) and n (# of observations)
    # Assuming number of observed horizontal distances is equal to n where a distance is calculating using the 2D vector equation (refer to function name 'model')
    u = 2
    n = xCoordsTarget.size

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
    xTargetEst = np.mean(xCoordsTarget)
    yTargetEst = np.mean(yCoordsTarget)
    # Assuming flat plane of operation z-component will near 0m AGL
    zTargetEst = 0

    # Holds coordinate plot of mean target observations
    plt.scatter(xTargetEst, yTargetEst, label = 'Mean Target Location')

    # Outputs mean values
    print("\nEstimated mean prior to adjustment: ", "Easting: ", xTargetEst, "Northing: ", yTargetEst, "AGL: ", zTargetEst)

    # Iteration count
    iteration = 0

    # Minimum threshold value before adjustment completes (m)
    threshold = 0.0001

    while iteration == 0 or abs(np.max(dHat)) > threshold:
        # Populating design matrix A
        for i in range(A.shape[0]):
            A[i, 0] = fDxTarget(xCoordsDrone[i], yCoordsDrone[i], xTargetEst, yTargetEst)
            A[i, 1] = fDyTarget(xCoordsDrone[i], yCoordsDrone[i], xTargetEst, yTargetEst)

        # Populating misclosure vector w (w = f(xo) - l) convention
        for i in range(w.size):
            w[i] = (model(xCoordsDrone[i], yCoordsDrone[i], xTargetEst, yTargetEst) - model(xCoordsDrone[i], yCoordsDrone[i], xCoordsTarget[i], yCoordsTarget[i]))


        N = (A.transpose()) @ P @ A
        U = (A.transpose()) @ P @ w
        dHat = (-1 * np.linalg.inv(N)) @ U

        xTargetEst += dHat[0]
        yTargetEst += dHat[1]
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
    print("\nEstimated post adjustment: ", "Easting: ", xTargetEst, "Northing: ", yTargetEst, "AGL: ", zTargetEst)

    plt.scatter(xCoordsDrone, yCoordsDrone, label = 'Drone Observation Points')
    plt.scatter(xCoordsTarget, yCoordsTarget, label = 'Target Observations')
    plt.scatter(xTargetEst, yTargetEst, label = 'Adjust Target Point')
  
    for i, (x_val, y_val) in enumerate(zip(xCoordsDrone, yCoordsDrone)): 
        plt.annotate(f'{i}', (x_val, y_val), textcoords="offset points", xytext=(0,10), ha='center')

    for i, (x_val, y_val) in enumerate(zip(xCoordsTarget, yCoordsTarget)): 
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
                xCoordsDrone = np.delete(xCoordsDrone, i)
                yCoordsDrone = np.delete(yCoordsDrone, i)
                zCoordsDrone = np.delete(zCoordsDrone, i)

                xCoordsTarget = np.delete(xCoordsTarget, i)
                yCoordsTarget = np.delete(yCoordsTarget, i)

        return parametricAdjustment(xCoordsDrone, yCoordsDrone, zCoordsDrone, xCoordsTarget, yCoordsTarget)
    
    else:
        return xTargetEst, yTargetEst, zTargetEst
    

#--------------------------------------------------Loading Data-----------------------------------------------------
vectors = parse_json_to_vectors("C:/Users/mfles/OneDrive - University of Calgary/SUAV/Model/test_data_points_with_pixels.json")

eastingDrone = []
northingDrone = []
verticalDrone = []
zoneDrone = []
for lat, lon, alt in zip(vectors['lat'], vectors['lon'], vectors['alt']):
    easting, northing, vertical, zone = lat_long_to_utm(lat, lon, alt)

    eastingDrone.append(easting)
    northingDrone.append(northing)
    verticalDrone.append(vertical)
    zoneDrone.append(zone)

eastingTarget = []
northingTarget = []
for eastingDroneTemp, northingDroneTemp, AGL, xPix, yPix, Yaw in zip(eastingDrone, northingDrone, vectors['rel_alt'], vectors['x_pixel'], vectors['y_pixel'], vectors['yaw']):
    easting, northing = image_to_object_space(eastingDroneTemp, northingDroneTemp, AGL, xPix, yPix, Yaw)
    eastingTarget.append(easting)
    northingTarget.append(northing)

#--------------------------------------------------Main-----------------------------------------------------
xTargetEst, yTargetEst, zTargetEst = parametricAdjustment(np.array(eastingDrone), np.array(northingDrone), np.array(verticalDrone), np.array(eastingTarget), np.array(northingTarget))

print(xTargetEst, yTargetEst, zTargetEst)