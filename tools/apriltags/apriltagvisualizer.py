import cv2
import apriltag  
import math
import meshcat
import numpy as np

# object size
objectsize = 38.5

# Predefined object = 38.5mm x 38.5mm April Tag PDF
objectPoints = np.array([[[-objectsize/2.0, -objectsize/2.0, 0], \
    [objectsize/2.0, -objectsize/2.0, 0], \
    [objectsize/2.0, objectsize/2.0, 0], \
    [-objectsize/2.0, objectsize/2.0, 0]]], \
    dtype=np.float32)

distCoeff = np.zeros((5,1))

rVec = None
tVec = None
iterate = False
calibrated = 0

# define a video capture object
vid = cv2.VideoCapture(0)
vis = meshcat.Visualizer().open()
box = meshcat.geometry.Box([0.5, 0.5, 0.5])
vis.set_object(box)

while(True):
    # Capture the video frame by frame
    ret, image = vid.read()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    results = detector.detect(gray)
    
    focal_length = image.shape[0]
    center = (image.shape[0]/2, image.shape[1]/2)
    camMatrix = np.array(
        [[focal_length, 0, center[0]],
        [0, focal_length, center[1]],
        [0, 0, 1]], dtype = "double"
        )
    
    # loop over the AprilTag detection results
    for r in results:
        # extract the bounding box (x, y)-coordinates for the AprilTag
        # and convert each of the (x, y)-coordinate pairs to integers
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))

        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)

        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        
        # draw the tag family on the image
        tagFamily = r.tag_family.decode("utf-8")
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        imagePoints = r.corners.reshape(1,4,2)
        good, rVec, tVec = cv2.solvePnP(objectPoints, imagePoints, \
            camMatrix, distCoeff, rVec, tVec, iterate, cv2.SOLVEPNP_ITERATIVE)
        tVec = 0.001 * tVec
        print(tVec)
        vis.set_transform(meshcat.transformations.translation_matrix([tVec[0], tVec[1], tVec[2]]))
    
    # show the output image after AprilTag detection
    cv2.imshow("Image", image)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()