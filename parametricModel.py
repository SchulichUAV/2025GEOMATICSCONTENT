#--------------------------------------------------Header-----------------------------------------------------
import numpy as np
import scipy.stats as stats
import matplotlib.pyplot as plt

#--------------------------------------------------Functions-----------------------------------------------------



# Functional parametric model
def model(xCoordsDrone, yCoordsDrone, xCoordsTarget, yCoordsTarget):
    xDelta = xCoordsDrone - xCoordsTarget
    yDelta = yCoordsDrone - yCoordsTarget
    dObs = np.sqrt(xDelta**2 + yDelta**2)
    
    return dObs


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


# df/dxDrone
def fDxDrone(xCoordsDrone, yCoordsDrone, xCoordsTarget, yCoordsTarget):
    xDelta = xCoordsDrone - xCoordsTarget
    yDelta = yCoordsDrone - yCoordsTarget
    fDxDrone = xDelta / np.sqrt(xDelta**2 + yDelta**2)
    
    return fDxDrone


# df/dyDrone
def fDyDrone(xCoordsDrone, yCoordsDrone, xCoordsTarget, yCoordsTarget):
    xDelta = xCoordsDrone - xCoordsTarget
    yDelta = yCoordsDrone - yCoordsTarget
    fDyDrone = yDelta / np.sqrt(xDelta**2 + yDelta**2)
    
    return fDyDrone


# Load coordinates from file
def load_coordinates(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    xCoords = []
    yCoords = []
    zCoords = []
    for line in lines:
        values = line.split()
        if len(values) == 3:
            x, y, z = map(float, values)
            xCoords.append(x)
            yCoords.append(y)
            zCoords.append(z)
        else:
            print(f"Skipping line: {line.strip()} (expected 3 values, got {len(values)})")
    return np.array(xCoords), np.array(yCoords), np.array(zCoords)



# Compute standard deviation of observations
def compute_stDevObs(xCoordsDrone, yCoordsDrone, zCoordsDrone, xCoordsTarget, yCoordsTarget):
    stDev = np.zeros(len(xCoordsDrone))

    for i in range(len(stDev)):
        stDev[i] = 1
    
    return stDev


# Create covariance matrix of observations
def covarianceMatrixObs(size, stDev):
    Cl = np.zeros((size, size))
    np.fill_diagonal(Cl, stDev**2)

    return Cl


def parametricAdjustment(xCoordsDrone, yCoordsDrone, zCoordsDrone, xCoordsTarget, yCoordsTarget, zCoordsTarget):
    # Defining u(# of parameters) and n (# of observations)
    # Assuming number of observed horizontal distances is equal to n where a distance is calculating using the 2D vector equation (refer to function name 'model')
    u = 2
    n = xCoordsTarget.size

    # Degrees of freedom
    dof = n - u

    # Compute standard deviation of observations
    stDev_array = compute_stDevObs(xCoordsDrone, yCoordsDrone, zCoordsDrone, xCoordsTarget, yCoordsTarget)
    
    # Create covariance matrix
    Cl = covarianceMatrixObs(n, stDev_array)

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

        # Populating misclosure vector w
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
    Cl = aPosterioriVarianceFactor * covarianceMatrixObs(n, stDev_array)
    
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
                zCoordsTarget = np.delete(zCoordsTarget, i)

        return parametricAdjustment(xCoordsDrone, yCoordsDrone, zCoordsDrone, xCoordsTarget, yCoordsTarget, zCoordsTarget)
    
    else:
        return xTargetEst, yTargetEst, zTargetEst
    

#--------------------------------------------------Loading Data-----------------------------------------------------
filePathDrone = "C:/Users/User/OneDrive - University of Calgary/SUAV/Model/data/ctrlPnts_2024.txt"
xCoordsDrone, yCoordsDrone, zCoordsDrone = load_coordinates(filePathDrone)

filePathTarget = "C:/Users/User/OneDrive - University of Calgary/SUAV/Model/data/TargetObservations.txt"
xCoordsTarget, yCoordsTarget, zCoordsTarget = load_coordinates(filePathTarget)


#--------------------------------------------------Main-----------------------------------------------------
xTargetEst, yTargetEst, zTargetEst = parametricAdjustment(xCoordsDrone, yCoordsDrone, zCoordsDrone, xCoordsTarget, yCoordsTarget, zCoordsTarget)

print(xTargetEst, yTargetEst, zTargetEst)




