import numpy as np
import cv2
import glob

checkerboard_dims = (8, 6)

objp = np.zeros((checkerboard_dims[0] * checkerboard_dims[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_dims[0], 0:checkerboard_dims[1]].T.reshape(-1, 2) 

objpoints = [] #3d
imgpoints = [] #2d

images = [cv2.imread(file) for file in glob.glob("camera_calibration_photo_mosaic/calibration photos/*.jpg")] 


for i in range(len(images)): 
    #cv2.imshow("img" + str(i), cv2.resize(images[i], (756,1008)))
    #cv2.waitKey(0)
    grey = cv2.cvtColor(images[i], cv2.COLOR_BGR2GRAY) 
    
    pattern_found, corners = cv2.findChessboardCorners(grey, checkerboard_dims, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)

    if pattern_found:
        objpoints.append(objp)
        refined_corners = cv2.cornerSubPix(grey, corners, (5,5), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))     
        imgpoints.append(refined_corners)

        # **OPTIONAL STEP OF DRAWING**
        #image = cv2.drawChessboardCorners(images[i], checkerboard_dims, refined_corners, pattern_found) 
        #cv2.imshow("img" + str(i), cv2.resize(image, (756, 1008)))
        #cv2.waitKey(0)

cv2.destroyAllWindows() 

ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, grey.shape[::-1],None, None)
np.savez("calibration_results", mtx, dist, rvecs, tvecs)

data = (np.load("calibration_results.npz"))
print(data)
#print(data["arr_1"])


#print(rvecs)
#mean_error = 0
#for i in range(len(objpoints[0])):
#imgpts, jacobian = cv2.projectPoints(np.array(objpoints), np.float32(rvecs), np.float32(tvecs), mtx, None) #this is erroring, may have to do with inliers
#mean_error  = abs(imgpoints[i] - imgpts)

#print(mean_error/len(imgpoints))
#may need to use cv2.solvePnP

'''
imagePoints, jacobian = cv.projectPoints(np.array(obj_points),
                        np.float32(rvec), np.float32(tvec),camera_matrix1, None)

Verify the calibration:
    Initialize mean_error to 0
    For each pair of object points and image points:
        Project the object points to image points using projectPoints
        Compute the error between the projected and actual image points
        Accumulate the error
    Compute the average error
    Print the total average error
'''