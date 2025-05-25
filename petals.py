#--------------------------------------------------Header-----------------------------------------------------
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt
from pyproj import CRS, Transformer
import json

def parametric_model(file_path):
    #--------------------------------------------------Functions-----------------------------------------------------
    def parse_json_to_vectors(file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)

        # If the top-level JSON object is A dict, search for any list value.
        if isinstance(data, dict):
            for value in data.values():
                if isinstance(value, list):
                    data = value
                    break
            else:
                # If no list value is found, assume it's A single record.
                data = [data]

        # Define the keys you want to extract.
        keys = ["image", "x", "y", "lat", "lon", "rel_alt", "alt", "roll", "pitch", "yaw"]

        # Initialize A vector (list) for each key.
        vectors = {key: [] for key in keys}

        # Iterate over each record and append values to corresponding vectors.
        for record in data:
            for key in keys:
                # Using .get(key) in case some records are missing A key.
                vectors[key].append(record.get(key))
        
        return vectors

    def lat_long_to_utm(lat, lon):
        """
        Convert geographic coordinates (latitude, longitude, and altitude) to UTM coordinates.
        
        Parameters:
        lat (float): Latitude in decimal degrees.
        lon (float): Longitude in decimal degrees.
        MSL (float): Altitude (e.g., Mean Sea Level height, usually in meters).
        
        Returns:
        (easting, northing, zone): A three-element tuple, easting/northing (meters), zone (1-60).
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
        
        # Create A Transformer from geographic coordinates (WGS84) to UTM
        transformer = Transformer.from_crs(wgs84, utm_crs, always_xy=True)
        
        # Transform (lon, lat), while passing MSL as the vertical coordinate.
        # Note: UTM is typically 2D but if you pass A third coordinate, it will usually apply an identity transformation on it.
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
        
        # Create A Transformer to convert UTM coordinates back to WGS84.
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
    def image_to_object_space(easting_drone, northing_drone, agl, x_pix, y_pix, yaw, pitch, roll): 

        focal_length = 0.0125 # meters (m)
        pixel_spacing = 0.00345 # mm per pix
        # fiducial centre (mm)
        x_fiducial = 2.5116
        y_fiducial = -1.8768

        scale = agl / focal_length

        # image x and y (mm)
        image = np.array([(x_pix * pixel_spacing) - x_fiducial, ((-y_pix * pixel_spacing) - y_fiducial)])

        # object x and y in m relative to drone
        #                                        check if agl should be negative or positive (was originally positive)
        obj = np.array([((scale * image[0]) / 1000), ((scale * image[1]) / 1000), agl])
        
        # y is direction of travel therefore conventional roll and pitch rotations are swapped
        # roll is rotation about y axis, pitch is rotation about x axis
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(pitch), np.sin(pitch)],
            [0, -np.sin(pitch), np.cos(pitch)]
        ])

        Ry = np.array([
            [np.cos(roll), 0, -np.sin(roll)],
            [0, 1, 0],
            [np.sin(roll), 0, np.cos(roll)]
        ])

        Rz = np.array([
            [np.cos(yaw), np.sin(yaw), 0],
            [-np.sin(yaw), np.cos(yaw), 0],
            [0, 0, 1]
        ])

        R = Rz @ Rx @ Ry

        # Target coordinates relative to drone position, not intersecting ground plane though
        # Therefore must compute intersection coordinates with ground plane by scaling by t
        
        target = R @ obj.transpose()
    

        vertical_depth = target[2]
        
        #                                        check if agl should be negative or positive (was originally positive)
        t = agl / vertical_depth

        target[0] = target[0] * t
        target[1] = target[1] * t

        easting_target = easting_drone + target[0]
        northing_target = northing_drone + target[1]

        return easting_target, northing_target

    # Functional parametric model
    def model(easting_drone, northing_drone, easting_target, northing_target):
        x_delta = easting_drone - easting_target
        y_delta = northing_drone - northing_target
        range = np.sqrt(x_delta**2 + y_delta**2)
        return range

    # df/dxTarget
    def f_dx_target(easting_drone, northing_drone, easting_target, northing_target):
        x_delta = easting_drone - easting_target
        y_delta = northing_drone - northing_target
        return -x_delta / np.sqrt(x_delta**2 + y_delta**2)

    # df/dyTarget
    def f_dy_target(easting_drone, northing_drone, easting_target, northing_target):
        x_delta = easting_drone - easting_target
        y_delta = northing_drone - northing_target
        return -y_delta / np.sqrt(x_delta**2 + y_delta**2)

    # Create covariance matrix of observations
    def covariance_matrix_obs(size):
        Cl = np.zeros((size, size))
        np.fill_diagonal(Cl, 1)
        return Cl

    def parametric_adjustment(easting_drone, northing_drone, agl_drone, easting_target, northing_target):
        # Defining u(# of parameters) and n (# of observations)
        # Assuming number of observed horizontal distances is equal to n where A distance is calculating using the 2D vector equation (refer to function name 'model')
        u = 2
        n = easting_target.size

        # Degrees of freedom
        dof = n - u

        # Compute standard deviation of observations
        
        # Create covariance matrix
        Cl = covariance_matrix_obs(n)

        # Compute weight matrix P
        P = np.linalg.inv(Cl)

        # Declaring design matrix A and misclosure vector w dimensions
        A = np.zeros((n, u))
        w = np.zeros((n))

        # Declaring threshold vector (how much each parameter changes given an iteration)
        d_hat = np.array([])

        # Estimating target coordinates by mean of target coordinate observations
        # Assuming flat plane of operation z-component will near 0m AGL
        easting_target_est = np.mean(easting_target)
        northing_target_est = np.mean(northing_target)

        # Outputs mean values
        print("\nEstimated mean prior to adjustment: ", "Easting: ", easting_target_est, "Northing: ", northing_target_est)

        # Holds coordinate plot of mean target observations
        plt.scatter(easting_target_est, northing_target_est, label = 'Mean Target Location')

        # Iteration count
        iteration = 0

        # Minimum threshold value before adjustment completes (m)
        threshold = 0.0000001

        while iteration == 0 or abs(np.max(d_hat)) > threshold:
            # Populating design matrix A
            for i in range(A.shape[0]):
                A[i, 0] = f_dx_target(easting_drone[i], northing_drone[i], easting_target_est, northing_target_est)
                A[i, 1] = f_dy_target(easting_drone[i], northing_drone[i], easting_target_est, northing_target_est)

            # Populating misclosure vector w (w = l - f(xo)) convention
            for i in range(w.size):
                w[i] = (model(easting_drone[i], northing_drone[i], easting_target[i], northing_target[i]) - model(easting_drone[i], northing_drone[i], easting_target_est, northing_target_est))

            N = (A.transpose()) @ P @ A
            U = (A.transpose()) @ P @ w
            d_hat = (np.linalg.inv(N)) @ U

            easting_target_est += d_hat[0]
            northing_target_est += d_hat[1]
            iteration += 1

        # Residuals
        r = A @ d_hat + w

        # Computed post adjustment (ensures Cl is scaled properly)
        a_posteriori_variance_factor = ((r.transpose()) @ P @ r ) / dof

        # Recomputing Cl to properly scale weight matrix to perform data snooping
        Cl = a_posteriori_variance_factor * covariance_matrix_obs(n)
        
        # Compute weight matrix P
        P = np.linalg.inv(Cl)

        # Should be equal to or near 1 after rescaling (Ensures none bias data analysis)
        a_posteriori_variance_factor = ((r.transpose()) @ P @ r ) / dof

        # Recomputed N
        N = (A.transpose()) @ P @ A

        Cx_hat = a_posteriori_variance_factor * np.linalg.inv(N)

        Cl_hat = A @ Cx_hat @ A.transpose()

        Cr_hat = Cl - Cl_hat

        # Plot data and adjusted target point
        print("\nEstimated post adjustment: ", "Easting: ", easting_target_est, "Northing: ", northing_target_est)

        plt.scatter(easting_drone, northing_drone, label = 'Drone Observation Points')
        plt.scatter(easting_target, northing_target, label = 'Target Observations')
        plt.scatter(easting_target_est, northing_target_est, label = 'Adjust Target Point')
    
        for i, (x_val, y_val) in enumerate(zip(easting_drone, northing_drone)): 
            plt.annotate(f'{i}', (x_val, y_val), textcoords="offset points", xytext=(0,10), ha='center')

        for i, (x_val, y_val) in enumerate(zip(easting_target, northing_target)): 
            plt.annotate(f'{i}', (x_val, y_val), textcoords="offset points", xytext=(0,10), ha='center')

        plt.title('Plot') 
        plt.xlabel('Easting') 
        plt.ylabel('Northing')
        plt.legend()
        plt.show()

        # Data snooping
        standardized_r_hat = r / np.sqrt(np.diag(Cr_hat))

        # Computes critical values of student-t distribution
        # Significance level (alpha)
        alpha = 0.05

        # Critical value for two-tailed test
        critical_value = stats.t.ppf(1 - alpha/2, dof)

        if (np.max(standardized_r_hat) > critical_value) or (np.min(standardized_r_hat) < -critical_value):
            for i in range(standardized_r_hat.size):
                if (standardized_r_hat[i] > critical_value) or (standardized_r_hat[i] < -critical_value):
                    easting_drone = np.delete(easting_drone, i)
                    northing_drone = np.delete(northing_drone, i)
                    agl_drone = np.delete(agl_drone, i)

                    easting_target = np.delete(easting_target, i)
                    northing_target = np.delete(northing_target, i)

            return parametric_adjustment(easting_drone, northing_drone, agl_drone, easting_target, northing_target)
        
        else:
            return easting_target_est, northing_target_est
        
    #--------------------------------------------------Main-----------------------------------------------------
    #--------------------------------------------------Loading Data-----------------------------------------------------
    vectors = parse_json_to_vectors(file_path)
    # "C:/Users/mfles/OneDrive - University of Calgary/SUAV/Model/test_data_points_with_pixels.json"
    easting_drone = []
    northing_drone = []
    zone_drone = []
    for lat, lon in zip(vectors['lat'], vectors['lon']):
        easting, northing, zone = lat_long_to_utm(lat, lon)

        easting_drone.append(easting)
        northing_drone.append(northing)
        zone_drone.append(zone)

    easting_target = []
    northing_target = []
    agl_drone = []

    for easting_drone_temp, northing_drone_temp, agl, x_pix, y_pix, yaw, pitch, roll in zip(easting_drone, northing_drone, vectors['rel_alt'], vectors['x'], vectors['y'], vectors['yaw'], vectors['pitch'], vectors['roll']):
        easting, northing = image_to_object_space(easting_drone_temp, northing_drone_temp, agl, x_pix, y_pix, yaw, pitch, roll)
        easting_target.append(easting)
        northing_target.append(northing)
        agl_drone.append(agl)

    #--------------------------------------------------Adjustment-----------------------------------------------------
    easting_target_est, northing_target_est = parametric_adjustment(np.array(easting_drone), np.array(northing_drone), np.array(agl_drone), np.array(easting_target), np.array(northing_target))

    zone = zone_drone[0]

    # Assuming all drone observations are in the same zone, reasonable assumption for A small area.
    # Convert adjusted UTM coordinates back to latitude and longitude
    lat, lon = utm_to_lat_long(easting_target_est, northing_target_est, zone, northern=True)

    #--------------------------------------------------Output-----------------------------------------------------
    return lat, lon

if "__name__" == "__main__":
    lat, lon = parametric_model("C:/Users/mfles/OneDrive - University of Calgary/SUAV/Model/savedCoords.json")

    print("Adjusted Latitude: ", lat)
    print("Adjusted Longitude: ", lon)