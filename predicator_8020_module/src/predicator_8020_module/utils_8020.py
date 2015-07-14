# im_pos
# im_rgb

import numpy as np

from skimage.measure import label, regionprops
from skimage.morphology import binary_opening, binary_erosion
import skimage.morphology as mo
from skimage.draw import polygon

from sklearn.utils import check_array
from sklearn.cluster import KMeans
from sklearn.svm import LinearSVC

from matplotlib.cm import jet
from skimage.draw import circle
from skimage.color import rgb2lab, rgb2hsv

import cv2

bounding_box = []
def mouse_event(event, x, y, flags=None, params=None):
    global bounding_box
    if event == cv2.EVENT_LBUTTONDOWN:
        bounding_box += [[y, x]]
        print "New point:", bounding_box[-1]

# Don't use: Automatically get table
if 0:
    mask = np.all(np.abs(im_rgb*1. - [255,170,130]) < 50, -1)
    labels = label(mask)
    props = regionprops(labels)
    areas = np.array([x.area for x in props])
    biggest_area = np.argmax([x.area for x in props])
    biggest_label = props[biggest_area].label
    mask = labels==biggest_label
    mask = mo.convex_hull_image(mask)

# ----------------TRAINING------------------------
def pick_bounding_box(im_rgb):
    global bounding_box
    bounding_box = []

    # Have user specify region
    cv2.namedWindow("pick_region")
    cv2.setMouseCallback("pick_region", mouse_event)
    while len(bounding_box) < 4:
        cv2.imshow("pick_region", im_rgb)
        ret = cv2.waitKey(30)
    cv2.destroyWindow("pick_region")

    return np.array(bounding_box)


def extract_foreground_poly(im_rgb, bounding_box):
    # Get polygon for region of interest. Compute mask
    if 1:
        im = rgb2lab(im_rgb[:,:,[2,1,0]])[:,:,:1]
        # im = rgb2hsv(im_rgb)[:,:,:1]
        #im = rgb2hsv(im_rgb)[:,:,:1]
        #im = rgb2hsv(im_rgb[:,:,[2,1,0]])[:,:,:1]
        # im[im[:,:,0]!=0] -= [50,0,0]
    else:
        im = im_rgb

    bounding_box = np.array(bounding_box)
    poly_points = polygon(bounding_box[:,0], bounding_box[:,1])

    mask = np.zeros(im.shape[:2], np.int)
    mask[poly_points[0], poly_points[1]] = True

    im = im*mask[:,:,None]

    return im, mask

def train_clf(im, mask):
    # TRAIN: First cluster background/foregound.
    # Then find binary classifier to seperate them.
    
    # mask = np.all(im!=0, -1)
    n_channels = im.shape[-1]
    train = im.reshape([-1, n_channels])[mask.ravel()==1, :]
    # train = im.reshape([-1, 3])[mask.ravel()!=0, 1:2]
    
    # Unserupervised clustering
    clf = KMeans(2)
    predict = clf.fit_predict(train)
    clf_mean = np.array(clf.cluster_centers_).mean(0)
    
    im_predict = clf.predict(im.reshape([-1, n_channels])).reshape([480,640])
    if clf.cluster_centers_[0][0] < clf.cluster_centers_[1][0]:
        predict = predict == 0
    # im_predict = clf.predict(im.reshape([-1, 3])[:,1:2]).reshape([480,640])

    svm = LinearSVC()
    svm.fit(train - clf_mean, predict)
    clf_w = svm.coef_

    im_predict = (np.dot((im - clf_mean), clf_w) > 0)[:,:,0]

    return clf_mean, clf_w

# TEST: Predict foreground objects
def get_foreground(im, clf_mean, clf_w, min_area=500):
    im_shape = im.shape
    n_channels = im_shape[-1]
    # pred_mask = (np.dot(im.reshape([-1, 3])[:,1:2] - clf_mean, clf_w.squeeze()) > 0).reshape(im_shape[:2])
    pred_mask = (np.dot(im.reshape([-1, n_channels]) - clf_mean, clf_w.squeeze()) > 0).reshape(im_shape[:2])
    # pred_mask = clf.predict(im.reshape([-1, 3])).reshape(im_shape[:2])
    object_labels = label(pred_mask)
    props = regionprops(object_labels)
    areas = np.array([x.area for x in props])

    objects = [x for x in props if min_area < x.area < 10000]
    pred_mask *= 0
    for obj in objects:
        pred_mask[obj.coords[:,0], obj.coords[:,1]] = True

    return pred_mask, objects, props

def extract_holes(im, pred_mask, objects, props, im_pos, im_rgb):
    
    all_holes = []
    all_centroids = []
    # all_masks = []
    # all_feat = []
    for obj in objects:
        # Detect hole locations in each object
        tl_x, tl_y, br_x, br_y = obj.bbox

        if 1:
            im_obj = np.zeros_like(im[tl_x:br_x, tl_y:br_y])
            im_tmp = im[tl_x:br_x, tl_y:br_y]
            im_obj[obj.coords[:,0]-tl_x, obj.coords[:,1]-tl_y] = im_tmp[obj.coords[:,0]-tl_x, obj.coords[:,1]-tl_y]
            obj_mask = np.all(im_obj!=0, -1)

        if 1:
            im_tmp = im_rgb[tl_x:br_x, tl_y:br_y]
            im_obj = np.zeros_like(im_rgb[tl_x:br_x, tl_y:br_y])        
            im_obj[obj.coords[:,0]-tl_x, obj.coords[:,1]-tl_y] = im_tmp[obj.coords[:,0]-tl_x, obj.coords[:,1]-tl_y]
            
            clf = KMeans(2)
            pred = clf.fit_predict(im_obj.reshape([-1, 3])).reshape(im_obj.shape[:2])
            if np.sum(pred==0) > np.sum(pred==1):
            # if clf.cluster_centers_[0][0] > clf.cluster_centers_[1][0]:
                pred = pred==0

            obj_hull = mo.convex_hull_image(obj_mask)
            obj_hull[:,0]=0
            obj_hull[:,-1]=0
            obj_hull[0,:]=0
            obj_hull[-1,:]=0
            obj_hull = cv2.erode(obj_hull.astype(np.uint8), np.ones([7,7]))
            # obj_hull = cv2.erode(obj_hull.astype(np.uint8), np.ones([9,9]))
            obj_mask = (1-pred) * obj_hull
            mask_diff = obj_mask

        # Get convex hull
        
        # obj_hull = mo.convex_hull_image(obj_mask)
        # obj_hull[:,0]=0
        # obj_hull[:,-1]=0
        # obj_hull[0,:]=0
        # obj_hull[-1,:]=0
        # # obj_hull = binary_erosion(obj_hull, np.ones([9,9]))
        # obj_hull = cv2.erode(obj_hull.astype(np.uint8), np.ones([11,11]))
        # # obj_hull = binary_erosion(obj_hull, np.ones([1,1]))
        # mask_diff = -obj_mask*obj_hull

        # 
        # im_pos_tmp = im_pos[tl_x:br_x, tl_y:br_y]
        # im_pos_tmp = im_pos_tmp*obj_hull[:,:,None]

        # # Get min/max metric positions
        # min_pos = im_pos_tmp[im_pos_tmp!=0].reshape([-1, 3]).min(0)
        # max_pos = im_pos_tmp[im_pos_tmp!=0].reshape([-1, 3]).max(0)
        # pos_diff = max_pos - min_pos
        # pos_diff = np.sqrt(np.sum(pos_diff**2))

        hole_labels = label(mask_diff)
        props = regionprops(hole_labels)
        holes = [x for x in props if x.area > 5]
        hole_centroids = np.array([x.centroid for x in holes])

        # print hole_centroids
        # print len(hole_centroids)
        if len(hole_centroids) > 0:
            hole_centroids += [tl_x, tl_y]
            all_holes += [hole_centroids]

            x,y = np.array(obj.centroid, np.int)
            all_centroids += [im_pos[x,y]]

    return all_centroids, all_holes

def plot_holes(im_rgb, all_holes):
    # Plot all holes in the image

    im_display = im_rgb.copy()
    # n_plates = len(all_holes)
    n_plates = 8
    for i, holes in enumerate(all_holes):
        n_holes = len(holes)

        color = np.array(jet(n_holes/float(n_plates))[:3])*255
        color = color.astype(np.int)

        for h in holes:
            pt = np.array(h, np.int)
            circ = circle(pt[0], pt[1], 5)
            im_display[circ[0], circ[1]] = color

    return im_display

def get_closest_part(centroids, holes):
    centroids = np.atleast_2d(centroids)
    print "c", centroids
    # print "s", np.sum(centroids**2, 0)
    # scores = np.sqrt(np.sum(centroids**2, 0))

    classes = np.array([len(h) for h in holes])

    closest_parts = {x:None for x in [3, 4, 8]}
    for c in closest_parts:
        parts = classes==c
        parts_centroids = centroids[parts]
        print "#", len(parts_centroids), parts_centroids

        if len(parts_centroids) == 0:
            closest_parts[c] = None
            continue        

        if len(parts_centroids) == 1:
            closest_parts[c] = parts_centroids[0]
        else:
            parts_scores = np.sum(parts_centroids**2, 1)
            closest_part = np.argmin(parts_scores)
            closest_parts[c] = parts_centroids[closest_part]

        print "set {}:".format(c), closest_parts[c]

    return closest_parts, classes
        

if 0:
    pick_bounding_box()

    im, mask = extract_foreground_poly(im_rgb, bounding_box)
    clf_mean, clf_w = train_clf(im, mask)
    mask_pred, objects, props = get_foreground(im, clf_mean, clf_w)
    all_centroids, all_holes = extract_holes(im, mask_pred, objects, props, im_rgb)
    im_display = plot_holes(im_rgb, all_holes)

    imshow(im_display[:,:,[2,1,0]])



