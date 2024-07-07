# Symbols Detection

## Overview
our team developed an AI model aimed at accurately identifying symbols displayed on a banner. This task is crucial for earning points based on the correct recognition of all symbols present.


## Approach
We utilized a YOLOv8-based computer vision model trained on a custom dataset containing six hieroglyphic symbols. The development process involved iterative rounds of image preprocessing and model fine-tuning to achieve satisfactory performance aligned with the competition's objectives.

## Dataset
- The dataset consists of images containing six hieroglyphic symbols. Each image in the dataset is annotated with bounding boxes that precisely delineate the location of each symbol.
- The images were collected from these sources:
     - [Source 1](https://github.com/morrisfranken/glyphreader)
     - [Source 2](https://www.metmuseum.org/about-the-met/collection-areas/egyptian-art)
     - [Source 3](https://www.flickr.com/photos/profzucker/)
     - [Source 4](https://stock.adobe.com/search?k=egypt+hieroglyphics)

