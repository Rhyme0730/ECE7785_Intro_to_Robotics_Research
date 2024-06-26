import csv
import cv2 
import numpy as np
import random
from sklearn.model_selection import train_test_split
import sys

global crop_size
crop_size = 30*30

def increase_brightness(img, value=25):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def crop(image, name='default', show=False):
    image = increase_brightness(image)
    h, w, _ = image.shape
    
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = ~gray 
    
    border = 10
    gray[-int(h/5):,:] = 0

    # gray = cv2.dilate(gray, np.ones((5,5)))
    gray = cv2.dilate(gray, np.ones((7,7)))
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
        d = 15
        if location[0]<d or abs(location[0]-w)<d:
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

def test_KNN(knn, test_lines, imageDirectory = './2023Fimgs/', image_type = '.png', k=7):
    correct = 0
    confusion_matrix = np.zeros((6,6))
    for i in range(len(test_lines)):
        test_img = cv2.imread(imageDirectory+test_lines[i][0]+image_type)
        test_img = crop(test_img)
        test_img = test_img.flatten().reshape(1, crop_size*3)
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

def train_KNN(train_lines, imageDirectory = './2023Fimgs/', image_type = '.png', k=7):
    random.shuffle(train_lines)

    train = []
    for i in range(len(train_lines)):
        image = cv2.imread(imageDirectory+train_lines[i][0]+image_type)
        cropped = crop(image) 
        train.append(cropped)

    train_data = np.array(train)
    train_data = train_data.flatten().reshape(len(train_lines), crop_size*3)

    train_data = train_data.astype(np.float32)

    # read in training labels
    train_labels = np.array([np.int32(train_lines[i][1]) for i in range(len(train_lines))])

    knn = cv2.ml.KNearest_create()
    knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

    correct = 0
    confusion_matrix = np.zeros((6,6))
    for i in range(len(train_lines)):
        test_img = cv2.imread(imageDirectory+train_lines[i][0]+image_type)
        test_img = crop(test_img)
        test_img = test_img.flatten().reshape(1, crop_size*3)
        test_img = test_img.astype(np.float32)
        ret, results, neighbours, dist = knn.findNearest(test_img, k)
        # print('truth: ' + str(train_labels[i]))
        # print(ret)
        # print(results)
        # print(neighbours)
        # print(dist)
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

def train_test_data(file_path):

    label_path = file_path + 'labels.txt'

    with open(label_path, 'r') as f:
        reader = csv.reader(f)
        labels_list = list(reader)

    # Split the data into train and test with a 3:1 ratio
    train, test = train_test_split(labels_list, test_size=0.25, random_state=42)  # random_state for reproducibility

    # Save the train and test splits to new text files
    train_file_path = file_path + 'train.txt'
    test_file_path = file_path + 'test.txt'

    with open(train_file_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(train)

    with open(test_file_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(test)

def test():
    # directory = './img_directory/2023Fimgs/'
    directory = './img_directory/2023_imgs/'
    # directory = './img_directory/2022Fimgs/'
    # directory = './img_directory/2022Fheldout/'
    train_test_data(file_path=directory)
    with open(directory + 'train.txt', 'r') as f:
        reader = csv.reader(f)
        train = list(reader) 
    with open(directory + 'test.txt', 'r') as f:
        reader = csv.reader(f)
        test = list(reader) 

    k = 4
    knn = train_KNN(train, directory, '.png', k=k)
    test_KNN(knn, test, directory, '.png', k=k)
    # knn = train_KNN(train, directory, '.jpg', k = k)
    # test_KNN(knn, test, directory, '.jpg', k = k)

def main():
    directory = './img_directory/2022Fheldout/'   ## Assuming photos are in this directory, change if need
    image_type='.png'  # change if need
    # train_test_data(file_path=directory)   # If need to split train and test dataset

    with open(directory + 'train.txt', 'r') as f:
        reader = csv.reader(f)
        train = list(reader) 
    with open(directory + 'test.txt', 'r') as f:
        reader = csv.reader(f)
        test = list(reader) 

    k = 4  # k nearest neighbor
    knn = train_KNN(train, imageDirectory=directory, image_type=image_type, k=k)
    test_KNN(knn, test, imageDirectory=directory, image_type=image_type, k=k)
    

if __name__=='__main__':
    # test()
    main()
