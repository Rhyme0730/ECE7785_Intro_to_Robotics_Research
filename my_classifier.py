import csv
import cv2 
from sklearn.cluster import KMeans
import numpy as np
import random

def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def test_KNN(knn, test_lines, imageDirectory = './2023Fimgs/', image_type = '.png'):
    k = 7
    correct = 0
    confusion_matrix = np.zeros((6,6))
    for i in range(len(test_lines)):
        test_img = cv2.imread(imageDirectory+test_lines[i][0]+image_type)
        test_img = crop(test_img)
        test_img = test_img.flatten().reshape(1, 30*30*3)
        test_img = test_img.astype(np.float32)
        ret, results, neighbours, dist = knn.findNearest(test_img, k)
        test_label = np.int32(test_lines[i][1])
        if test_label == ret:
            correct += 1
            confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
        else:
            confusion_matrix[test_label][np.int32(ret)] += 1
    acc = correct/len(test_lines)
    print('Test Accuracy: ' + str(acc))
    print('Test Confusion Matrix')
    print(confusion_matrix)

def train_KNN(train_lines, imageDirectory = './2023Fimgs/', image_type = '.png'):
    random.shuffle(train_lines)

    train = []
    for i in range(len(train_lines)):
        image = cv2.imread(imageDirectory+train_lines[i][0]+image_type)
        cropped = crop(image) 
        train.append(cropped)

    train_data = np.array(train)
    train_data = train_data.flatten().reshape(len(train_lines), 30*30*3)

    train_data = train_data.astype(np.float32)

    # read in training labels
    train_labels = np.array([np.int32(train_lines[i][1]) for i in range(len(train_lines))])

    knn = cv2.ml.KNearest_create()
    knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

    k = 7
    correct = 0
    confusion_matrix = np.zeros((6,6))
    for i in range(len(train_lines)):
        test_img = cv2.imread(imageDirectory+train_lines[i][0]+image_type)
        test_img = crop(test_img)
        test_img = test_img.flatten().reshape(1, 30*30*3)
        test_img = test_img.astype(np.float32)
        ret, results, neighbours, dist = knn.findNearest(test_img, k)
        print('truth: ' + str(train_labels[i]))
        print(ret)
        print(results)
        print(neighbours)
        print(dist)
        test_label = np.int32(train_lines[i][1])
        if test_label == ret:
            correct += 1
            confusion_matrix[np.int32(ret)][np.int32(ret)] += 1
        else:
            confusion_matrix[test_label][np.int32(ret)] += 1
    acc = correct/len(train_lines)
    print('Train Accuracy: ' + str(acc))
    print('Train Confusion Matrix')
    print(confusion_matrix)

    return knn


def crop(image, name='default', show=False):
    image = increase_brightness(image)
    h, w, _ = image.shape
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = ~gray 
    
    border = 10
    gray[-int(h/5):,:] = 0

    gray = cv2.dilate(gray, np.ones((5,5)))
    gray = cv2.erode(gray, np.ones((8,8)))

    ret, thresh = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    pot_contours = []
    for contour in contours: 
        if len(contour)>200:
            c = np.array(contour).reshape(-1,2).T

            # filter by closeness to edge
            if len(np.where(c[1:]>h-border)[0])>0 or len(np.where(c[1,:]<border)[0])>0: 
                continue
            pot_contours.append(contour)

    # filter by closest contour to midline of top 3 largest contours
    sorted_contours = sorted(pot_contours, key=cv2.contourArea)
    largest = sorted_contours
    
    closest_r = np.inf
    closest_contour = None
    for contour in largest:
        c = np.array(contour).reshape(-1,2).T
        location = np.mean(c, 1)
        
        if location[0]<15 or abs(location[0]-w)<15:
            continue
        if abs(location[1]-h/2)<closest_r:
            closest_r = abs(location[1]-h/2)
            closest_contour = c
    longest_contour = closest_contour


    if longest_contour is None:
        cropped = np.zeros((30,30,3)).astype('uint8')

    else:
        location = np.mean(longest_contour, 1)       

        corner1 = np.min(longest_contour,1)-10
        corner2 = np.max(longest_contour,1)+10
        corner1[np.where(corner1<0)] = 0
        if corner2[0]>=w:
            corner2[0] = w-1
        if corner2[1]>=h:
            corner2[1] = h-1
        cropped = image[corner1[1]:corner2[1], corner1[0]:corner2[0]]
    
    cropped = cv2.resize(cropped, (30, 30), interpolation = cv2.INTER_LINEAR)
    if show:
        cv2.imshow(name, cropped)
    return cropped

def crop_debug(image, file):
    image = increase_brightness(image)

    h, w, _ = image.shape
    print((h,w))
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = ~gray 
    
    border = 10
    gray[-int(h/5):,:] = 0

    gray = cv2.dilate(gray, np.ones((5,5)))
    gray = cv2.erode(gray, np.ones((8,8)))

    ret, thresh = cv2.threshold(gray, 130, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

    pot_contours = []
    for contour in contours: 
        if len(contour)>50:
            c = np.array(contour).reshape(-1,2).T

            # filter by closeness to edge
            if len(np.where(c[1:]>h-border)[0])>0 or len(np.where(c[1,:]<border)[0])>0: 
                continue
            pot_contours.append(contour)

    # filter by closest contour to midline of top 3 largest contours
    sorted_contours = sorted(pot_contours, key=cv2.contourArea)
    largest = sorted_contours
    
    closest_r = np.inf
    closest_contour = None
    for contour in largest:
        c = np.array(contour).reshape(-1,2).T
        location = np.mean(c, 1)
        
        print(location)
        print(c.shape[1])
        print(abs(location[1]-h/2))
        print()
        if location[0]<15 or abs(location[0]-w)<15:
            continue
        if abs(location[1]-h/2)<closest_r:
            closest_r = abs(location[1]-h/2)
            closest_contour = c
    longest_contour = closest_contour


    if longest_contour is None:
        print('No contour found!')
        return

    location = np.mean(longest_contour, 1)       

    #contours
    cv2.drawContours(image, pot_contours, -1, (0,255,0), 1) 

    #sign contour
    for i in range(longest_contour.shape[1]):
        cv2.circle(image, (int(longest_contour[0,i]), int(longest_contour[1,i])), 1, (255,255,0))
    cv2.circle(image, (int(location[0]), int(location[1])), 5, (0,255,255), -1)

    corner1 = np.min(longest_contour,1)-10
    corner2 = np.max(longest_contour,1)+10
    corner1[np.where(corner1<0)] = 0
    if corner2[0]>=w:
        corner2[0] = w-1
    if corner2[1]>=h:
        corner2[1] = h-1
    
    cv2.rectangle(image, (corner1[0], corner1[1]), (corner2[0], corner2[1]), (0,255,255), 1)

    print(longest_contour.shape[1])
    cv2.imshow('gray', gray)
    cv2.imshow('thresh', thresh)
    cv2.imshow(file, image)


def main():
    directory = './img_directory/'
    with open(directory + 'train.txt', 'r') as f:
        reader = csv.reader(f)
        train = list(reader) 
    with open(directory + 'test.txt', 'r') as f:
        reader = csv.reader(f)
        test = list(reader) 

    knn = train_KNN(train, directory, '.png')
    test_KNN(knn, test, directory, '.png')



if __name__=='__main__':
    main()