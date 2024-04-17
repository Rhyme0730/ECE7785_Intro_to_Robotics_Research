import csv
import cv2 
import numpy as np
import random
from sklearn.model_selection import train_test_split
import sys


global crop_size
crop_size = 30*30
crop_ratio = 0.2
def find_contour(img):
    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea)
    # print(cv2.contourArea(largest_contour))
    x, y, w, h = cv2.boundingRect(largest_contour)
    img_contour = img[y:y+h, x:x+w]
    return img_contour, x, y, w, h

def filter(img):
    img = cv2.GaussianBlur(img, (5, 5), sigmaX=0)
    img = cv2.Canny(img, 150, 200)
    return img

def image_preprocess(img, size1, size2):
    img = increase_light(img)
    img = img[int(0.05*410):(410-int(0.05*410)), :]
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    initial_img = img
    ret, thresh = cv2.threshold(img, 120, 150, cv2.THRESH_BINARY)
    img = filter(thresh)
    img, x, y, w, h= find_contour(img)
    img = initial_img[y:y+h, x:x+w]
    img = cv2.resize(img, (size1, size2))
    return img

def increase_light(img):
    value = 10
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv[:, :, 2] = np.clip(hsv[:, :, 2] + value, 0, 255)
    img = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    return img

def test_KNN(knn, test_lines, imageDirectory = './2023Fimgs/', image_type = '.png', k=7):
    correct = 0
    confusion_matrix = np.zeros((6,6))
    for i in range(len(test_lines)):
        test_img = cv2.imread(imageDirectory+test_lines[i][0]+image_type)
        test_img = image_preprocess(test_img, 30, 30)
        test_img = test_img.flatten().reshape(1, crop_size)
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
        cropped = image_preprocess(image, 30, 30) 
        train.append(cropped)

    train_data = np.array(train)
    train_data = train_data.flatten().reshape(len(train_lines), crop_size)

    train_data = train_data.astype(np.float32)

    # read in training labels
    train_labels = np.array([np.int32(train_lines[i][1]) for i in range(len(train_lines))])

    knn = cv2.ml.KNearest_create()
    knn.train(train_data, cv2.ml.ROW_SAMPLE, train_labels)

    correct = 0
    confusion_matrix = np.zeros((6,6))
    for i in range(len(train_lines)):
        test_img = cv2.imread(imageDirectory+train_lines[i][0]+image_type)
        test_img = image_preprocess(test_img, 30, 30)
        test_img = test_img.flatten().reshape(1, crop_size)
        test_img = test_img.astype(np.float32)
        ret, results, neighbours, dist = knn.findNearest(test_img, k)
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

def train_test_data(file_path, ratio):

    label_path = file_path + 'labels.txt'

    with open(label_path, 'r') as f:
        reader = csv.reader(f)
        labels_list = list(reader)

    # Split the data into train and test with a 3:1 ratio
    train, test = train_test_split(labels_list, test_size=ratio, random_state=42)  # random_state for reproducibility

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
    # directory = './img_directory/2023_imgs/'
    directory = './img_directory/2022Fimgs/'
    # directory = './img_directory/2022Fheldout/'
    # train_test_data(file_path=directory)
    with open(directory + 'train.txt', 'r') as f:
        reader = csv.reader(f)
        train = list(reader) 
    with open(directory + 'test.txt', 'r') as f:
        reader = csv.reader(f)
        test = list(reader) 

    k = 7
    knn = train_KNN(train, directory, '.png', k=k)
    test_KNN(knn, test, directory, '.png', k=k)
    # knn = train_KNN(train, directory, '.jpg', k = k)
    # test_KNN(knn, test, directory, '.jpg', k = k)

def main():
    directory = './img_directory/2024Simgs/'   ## Assuming photos are in this directory, change if need
    image_type='.png'  # change if need
    # train_test_data(file_path=directory, ratio=0.3)   # If need to split train and test dataset

    with open(directory + 'train.txt', 'r') as f:
        reader = csv.reader(f)
        train = list(reader) 
    with open(directory + 'test.txt', 'r') as f:
        reader = csv.reader(f)
        test = list(reader) 

    k = 6 # k nearest neighbor
    knn = train_KNN(train, imageDirectory=directory, image_type=image_type, k=k)
    test_KNN(knn, test, imageDirectory=directory, image_type=image_type, k=k)
    

if __name__=='__main__':
    main()
    # test()
