import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt

#PLEASE NOTE: this code works up to the calculating homography because I worked on it for many hours and couldn't figure out how it worked 
#also, it is currently formatted to only stitch two images together so testing would be easier 

def load_calibration(calibration_file):
    data = (np.load(calibration_file))
    mtx = data["arr_0"] 
    dist = data["arr_1"]
    return mtx, dist

def undistort_image(img, mtx, dist):
    w = img.shape[1]
    h = img.shape[0] #I SWITCHED THE W AND H AND MESSED EVERYTHING UP AND IVE BEEN CONFUSED FOR TWO HOURS IM--

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

    x, y, w, h = roi #322, 1328, 2343, 1308 from (4032, 3024, 3)
    dst = dst[y:y+h, x:x+w] 
    return dst #undistored image 

def harris_corner_detection(image):
    grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 
    grey = np.float32(grey)
    kernel = np.ones((5, 5), np.uint8) #added this in to dilate detected corners more 

    dst = cv2.cornerHarris(grey,2,3,0.04) # parameters taken from opencv documentation
    dst = cv2.dilate(dst, kernel)

    image[dst>0.01*dst.max()]=[0,0,255]
    return image, dst #returns marked image, array of (dilated) corners

def match_features(img1, img2):
    sift = cv2.SIFT_create()
    kp1, des1 = sift.detectAndCompute(img1,None)
    kp2, des2 = sift.detectAndCompute(img2,None)
    
    bf = cv2.BFMatcher()
    matches = bf.knnMatch(des1,des2,k=2)
    good = []
    for m,n in matches:
        if m.distance < 0.25*n.distance:
            good.append(m) #cuts 49173 matches down to 328, takes 120 seconds

    #img3 = cv2.drawMatchesKnn(img1,kp1,img2,kp2,good,None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    return good, kp1, des1, kp2, des2 #returns list of (good) matches, kps and des

def create_mosaic(images, camera_matrix, dist_coeffs):
    undist = []
    for img in images:
        undist.append(undistort_image(img, camera_matrix, dist_coeffs))
    #return undist // undistort_image working properly

    harris = []
    for img1 in undist:
        marked_img, corners = harris_corner_detection(img1)
        harris.append(marked_img)
    #return harris // harris_corner_detection working properly mostly, some parameters may be off but it is marking points

    good, kp1, des1, kp2, des2 = match_features(harris[0], harris[1]) #good matches of two images, plus kp and des of both
    #finds the matched points using SIFT (though ORB or FAST may be faster), takes 120 seconds to match two imgs
    #its good up to this point
    
    src_pts = np.float32([ kp1[m.queryIdx].pt for m in good ]).reshape(-1,1,2) #source points
    dst_pts = np.float32([ kp2[m.trainIdx].pt for m in good ]).reshape(-1,1,2) #destination points (i guess where you're mapping to)

    
    H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC,5.0)
    
    result = cv2.warpPerspective(undist[0], H, (undist[1].shape[1], undist[1].shape[0])) 
    return result

             
def main(): 
    mtx, dist = load_calibration("calibration_results.npz")
    images = []
    for file in sorted(glob.glob("camera_calibration_photo_mosaic/International Village - 50 Percent Overlap/*.jpg")):
        #print(file) print file name
        images.append(cv2.imread(file))

    mosaic_two_imgs = create_mosaic(images, mtx, dist)
    shape_y = int(mosaic_two_imgs.shape[0] / 4)
    shape_x = int(mosaic_two_imgs.shape[1] / 4)
    cv2.imshow("Mosaic", cv2.resize(mosaic_two_imgs, (shape_x, shape_y)))
    cv2.waitKey(0)


if __name__=="__main__": 
    main() 
