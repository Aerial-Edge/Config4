import cv2                      # Import OpenCV package for image processing
import cvzone                   # Import cvzone package for additional image processing capabilities
from cvzone.ColorModule import ColorFinder  # Import ColorFinder module from cvzone package for color detection
from cvzone.FPS import FPS       # Import FPS module from cvzone package for calculating frames per second
import time                     # Import time package for time operations
import numpy as np              # Import numpy package for numerical operations
import math
from picamera2 import Picamera2
from libcamera import controls
import xml.etree.ElementTree as ET
import os

def is_circle(cnt, threshold=0.6):
    area = cv2.contourArea(cnt)
    perimeter = cv2.arcLength(cnt, True)
    if perimeter == 0:
        return False
    circularity = 4 * np.pi * area / (perimeter * perimeter)
    return circularity >= threshold

def parse_label(xml_file):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    label = {}
    for obj in root.findall('object'):
        name = obj.find('name').text
        box = obj.find('bndbox')
        xmin = int(box.find('xmin').text)
        xmax = int(box.find('xmax').text)
        ymin = int(box.find('ymin').text)
        ymax = int(box.find('ymax').text)
        
        label[name] = [(xmin, ymin, xmax, ymax)]
    return label

def calculate_f1_score(precision, recall):
    if precision + recall == 0:  # to avoid division by zero
        return 0
    else:
        f1_score = 2 * (precision * recall) / (precision + recall)
        return f1_score
    
def read_image(image_file):
    return cv2.imread(image_file)

def detect_color(image):
    # Define color range for detection
    myColorFinder = ColorFinder(False)
    hsvVals = {'hmin': 49, 'smin': 69, 'vmin': 17, 'hmax': 108, 'smax': 255, 'vmax': 181}

    imgColor, mask = myColorFinder.update(image, hsvVals)
    imgContour, contours = cvzone.findContours(image, mask)

    circular_contours = [cnt for cnt in contours if is_circle(cnt['cnt'])]

    results = []
    if circular_contours:
        for cnt in circular_contours:
            x, y, w, h = cv2.boundingRect(cnt['cnt'])
            results.append(('green', (x, y, x+w, y+h)))  # replace 'green' with the actual color name
    return results

def calculate_iou(box1, box2):
    x1, y1, w1, h1 = box1
    x2, y2, w2, h2 = box2

    xi1 = max(x1, x2)
    yi1 = max(y1, y2)
    xi2 = min(x1+w1, x2+w2)
    yi2 = min(y1+h1, y2+h2)

    inter_area = max(xi2 - xi1, 0) * max(yi2 - yi1, 0)

    box1_area = (w1 - x1) * (h1 - y1)
    box2_area = (w2 - x2) * (h2 - y2)
    union_area = box1_area + box2_area - inter_area

    return inter_area / union_area if union_area > 0 else 0

def calculate_precision_recall(predictions, ground_truth, iou_threshold=0.5):
    TP = FP = FN = 0

    for image_predictions, image_ground_truth in zip(predictions, ground_truth):
        for pred_color, pred_box in image_predictions:
            if pred_color in image_ground_truth:
                ious = [calculate_iou(pred_box, truth_box) for truth_box in image_ground_truth[pred_color]]
                max_iou = max(ious) if ious else 0

                if max_iou >= iou_threshold:
                    TP += 1
                else:
                    FP += 1
            else:
                FP += 1

        for truth_color, truth_boxes in image_ground_truth.items():
            if truth_color not in [pred_color for pred_color, _ in image_predictions]:
                FN += len(truth_boxes)
            else:
                for truth_box in truth_boxes:
                    ious = [calculate_iou(pred_box, truth_box) for pred_color, pred_box in image_predictions if pred_color == truth_color]
                    max_iou = max(ious) if ious else 0
                    if max_iou < iou_threshold:
                        FN += 1

    precision = TP / (TP + FP) if TP + FP > 0 else 0
    recall = TP / (TP + FN) if TP + FN > 0 else 0

    return precision, recall


# Replace with the path to your dataset
image_dir = '/home/vaffe/RandomStuff/dataset/greenball/'
label_dir = '/home/vaffe/RandomStuff/dataset/greenball/labels/'

image_files = sorted(os.listdir(image_dir))
label_files = sorted(os.listdir(label_dir))

predictions = []
ground_truths = []

for image_file, label_file in zip(image_files, label_files):
    image_path = os.path.join(image_dir, image_file)
    label_path = os.path.join(label_dir, label_file)

    image = read_image(image_path)
    label = parse_label(label_path)

    prediction = detect_color(image)

    predictions.append(prediction)
    ground_truths.append(label)

precision, recall = calculate_precision_recall(predictions, ground_truths)
f1_score = calculate_f1_score(precision, recall)

print('Precision: ', precision)
print('Recall: ', recall)
print('F1 Score: ', f1_score)

