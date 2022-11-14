import numpy as np
import cv2
import yaml

# Open a sample video available in sample-videos
vcap = cv2.VideoCapture('http://idpcam1.eng.cam.ac.uk:8080/stream/video.mjpeg')
if not vcap.isOpened():
    print ("File Cannot be Opened")

# Defining the dimensions of checkerboard
CHECKERBOARD = (8,5)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = [] 
# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

# Capture frame-by-frame
ret, frame = vcap.read()
#print cap.isOpened(), ret
if frame is not None:
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    cv2.imwrite('gray.png', gray)
    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    """
    If desired number of corner are detected,
    we refine the pixel coordinates and display 
    them on the images of checker board
    """
    if ret == True:
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        
        imgpoints.append(corners2)

        # Draw and display the corners
        frame = cv2.drawChessboardCorners(frame, CHECKERBOARD, corners2, ret)
    
    # Display the resulting frame
    cv2.imshow('frame',frame)
    cv2.imwrite('chessboard.png', frame)
    cv2.destroyAllWindows()
else:
    print ("Frame is None")

"""
Performing camera calibration by 
passing the value of known 3D points (objpoints)
and corresponding pixel coordinates of the 
detected corners (imgpoints)
"""
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

data = {'camera_matrix': np.asarray(mtx).tolist(), 'dist_coeff': np.asarray(dist).tolist()}

with open("calibration.yaml", "w") as f:
    yaml.dump(data, f)

with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)

mtxloaded = loadeddict.get('camera_matrix')
distloaded = loadeddict.get('dist_coeff')

# When everything done, release the capture
vcap.release()
