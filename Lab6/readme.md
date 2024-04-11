## Requirements
- cv2
- sklearn
- numpy

To run KNN.py

1.Change the path to directory stored photos, e.g. `directory = './img_directory/2022Fheldout/'` 

2.Change the type of image if need, e.g. `image_type='.png'` or `image_type='.jpg'`

(Optional)3.We need to comment out the first line of main: `train_test_data(file_path=directory)` to split train and test dataset (if there exist no train.txt and test.txt)

The scripts will output confusion matrix and test and train accuracy, respectively.
