import cv2
from common import cv_contour_list_to_pathsList

'''
    extract(image): Extracts lines from an image using opencv findContours() function
    returns [image, pathlist(mirrored), pathlist]:
        *An image width extracted lines highlighted,
        *A paths list thats mirrored(this is for ZA6 Robot arm),
        *A paths list thats not mirrored
'''
def extract(image, canny_threshold1=50, canny_threshold2 = 50):
    print("Getting ready to extract...")

    # Mirror the image horizontally
    image = cv2.flip(image, 1)

    # Canny Edge Detection
    img_blur = cv2.GaussianBlur(image, (5, 5), 0)


    img_gray = cv2.Canny(image=img_blur, threshold1=canny_threshold1, threshold2=canny_threshold2)

    # apply binary thresholding
    ret, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)

    # visualize the binary image
    # cv2.imshow('Binary image', thresh)
    # cv2.waitKey(0)
    # cv2.imwrite('image_thres1.jpg', thresh)
    # cv2.destroyAllWindows()

    # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
    contours, _ = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)
    
    #----------
    non_mirror_img = cv2.flip(thresh, 1)
    non_mirror_contours, _ = cv2.findContours(image=non_mirror_img, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE)

    final_paths = cv_contour_list_to_pathsList(contours)
    non_mirror_final_paths = cv_contour_list_to_pathsList(non_mirror_contours)
    

    
    print("Extract Done!!")
    
    image_copy = image.copy()
    
    # draw contours on the copy image
    image_copy = cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)
    
    # Mirror the image horizontally
    image_copy = cv2.flip(image_copy, 1)
    
    return image_copy, final_paths, non_mirror_final_paths

# Test
# image = cv2.imread('assets/testmap.png')
# img, _, _ = extract(image)

# cv2.imshow('Binary image', img)
# cv2.waitKey(0)
# # cv2.imwrite('image_thres1.jpg', thresh)
# cv2.destroyAllWindows()