# Underwater Symbols Detection

## Overview
Our team developed an AI model aimed at accurately identifying symbols displayed on a banner. This task is crucial for earning points based on the correct recognition of all symbols present.

## Table of Contents
- [Approach](#approach)
- [Dataset](#dataset)
- [Data Preparation](#data-preparation)
- [Results](#results)
- [Scripts](#scripts)
- [Requirements](#requirements)

___

## Approach
We utilized a YOLOv8-based computer vision model trained on a custom dataset containing six hieroglyphic symbols. The development process involved iterative rounds of image preprocessing and model fine-tuning to achieve satisfactory performance aligned with the competition's objectives.
- The dataset used in training is located at `data/processed`.
- The results of training are located at `results/`.
- The best trained model is located at `models/best-models/best.pt`.

___

## Dataset
- The dataset consists of images containing six hieroglyphic symbols. Each image in the dataset is annotated with bounding boxes that precisely delineate the location of each symbol.
- The images were collected from these sources:
     - [Source 1](https://github.com/morrisfranken/glyphreader)
     - [Source 2](https://www.metmuseum.org/about-the-met/collection-areas/egyptian-art)
     - [Source 3](https://www.flickr.com/photos/profzucker/)
     - [Source 4](https://stock.adobe.com/search?k=egypt+hieroglyphics)
- Roboflow was used for annotating the images and preparing them for training.
- There are six classes in the data set: Ankh, Basket, Mouth, Owl, Reed, and Water.
- Total Number of Images: 1599 images, Train Set: 70% (1120 images), Validation Set: 15% (239 images), Test Set: 15% (240 images).

___

## Data Preparation

Before training, we conducted extensive preprocessing on the dataset to enhance the model's performance:

- **Cropping**: Adjusting the size of images to focus on relevant parts containing symbols.
- **Rotation**: Correcting image orientation to standardize input for the model.
- **Grayscale Conversion**: Converting images to grayscale to reduce computation complexity and focus on symbol shape.
- **Adaptive Thresholding**: Applying adaptive thresholding techniques to binarize images based on local pixel intensities, enhancing symbol visibility.
- **Data Augmentation**: The following augmentation parameters were applied:
     - Rotation: Between −14◦ and +14◦ (Each image is rotated randomly within this range to create variations).
     - Blur: Up to 1.9 px (Each image is blurred with a random blur intensity up to 1.9 pixels).
     - Outputs per training example: 3 (For each original image in the dataset, the data augmentation process generates three different augmented versions. These versions are variations of the original image, each modified according to the specified augmentation parameters: rotation and blur).

___

## Results

The following table shows the values of the precision and recall of the validation set on each class.
<p align="center">
  <img src="images/results.png" width="500" height="300">
</p>

In our hieroglyphic symbol detection task, precision and recall are crucial. The model must be accurate and
reliable to prevent mislabeling symbols, which could distort our interpretation of ancient inscriptions. Additionally, the model needs to recall all instances of the classes in an image to ensure a comprehensive understanding of the inscriptions.
<br> <br>

From the table, we can see that our model was able to recall almost all the symbols of the Mouth, Reed, and Owl symbols with slight loss in the other classes. Regarding precision, the model accurately detected most of the instances with few occurrences of false positives for the classes of Owl and Mouth with minor losses in the Water, Ankh, Reed, and Basket classes. The slight reductions in precision and recall are acceptable given the competition’s conditions and the hardware limitations.

___

## Scripts

1. **`src/symbols_detection.py`**:
   - This script provides functionality for symbol detection using the YOLOv8 trained model.
   - It includes methods for image preprocessing, obtaining predictions, and drawing detections on frames.

2. **`src/real_time_symbols_detection.py`**:
   - This script enables real-time detection and display of symbols using a camera feed.
   - It utilizes multiprocessing for efficient frame capture and symbol detection.
   - It performs symbol detection using the `SymbolsDetector` class, and displays the frames with detected symbols in real-time.
   - It also provides options to save frames and videos upon user input.

___

## Requirements
- **`ultralytics`**: (YOLOv8): Deep learning library for object detection.
- **`opencv-python`**: (cv2): OpenCV library for computer vision tasks.
- **`multiprocessing`**: Python module for spawning processes using an API similar to threading.
