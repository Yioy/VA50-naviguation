
import cv2 as cv
from trafficsign import TrafficSign
import csv
import easyocr
import re
import numpy as np
from Levenshtein import distance as levenshtein_distance
from ultralytics import YOLO
import rospkg

reader = easyocr.Reader(['fr', 'en'])  
model_path = "best.pt"
class_mapping = {0: 'city-signboard', 1: 'direction-leftOrRight', 2: 'direction-straight', 3: 'direction-straightOrLeft',
                 4: 'direction-straightOrRight', 5: 'direction-turnLeft', 6: 'direction-turnRight', 7: 'end-priority-road',
                 8: 'intersection-with-priority', 9: 'no-entry', 10: 'no-left-turn', 11: 'no-right-turn', 12: 'no-through-road',
                 13: 'no-u-turn', 14: 'no-vehicles', 15: 'right-priority', 16: 'road-priority', 17: 'stop', 18: 'yield'}

# Mapping of traffic sign labels to their categories and actions
# TO DO : implement vehicles priority and vehicles (<-) and STOP + direction (stop of the vehicles and restarts)
sign_info = {
        "direction-leftOrRight": ("signs", ["left, right"]),
        "direction-straight": ("signs", ["straight"]),
        "direction-straightOrLeft": ("signs", ["straight", "left"]),
        "direction-straightOrRight": ("signs", ["straight", "right"]),
        "direction-turnLeft": ("signs", ["left"]),
        "direction-turnRight": ("signs", ["right"]),
        "end-priority-road": ("signs", "Fin de route prioritaire, soyez prudent"),  #<-
        "intersection-with-priority": ("signs", "Approchez l'intersection avec priorité"),
        "no-entry": ("signs", ["left", "right"]),
        "no-left-turn": ("signs", ["straight", "right"]),
        "no-motor-vehicules": ("signs", ["left", "right"]),
        "no-right-turn": ("signs", ["left", "straight"]),
        "no-through-road": ("signs", ["left", "right"]),
        "no-u-turn": ("signs", ["left", "right", "straight"]),    
        "no-vehicules": ("signs", ["left", "right"]),
        "right-priority": ("signs", ["left", "right", "straight"]),  #<-
        "road-priority": ("signs", ["left", "right", "straight"]),   #<-
        "stop": ("signs", ["left", "right", "straight"]),        #<-
        "yield": ("signs", ["left", "right", "straight"])        #<-
    }

citiesName = []

class TrafficSignDetector(object):
    def __init__(self, model_name, cities_file_name):

        rospack = rospkg.RosPack()
        model_path = rospack.get_path('trafficsigns') + '/models/' + model_name
        cities_name_path = rospack.get_path('trafficsigns') + '/models/' + cities_file_name

        self.model = YOLO(model_path)
        self.confidence_threshold = 0.4
        self.font = cv.FONT_HERSHEY_SIMPLEX
        readAllCities(cities_name_path)

    def get_traffic_sign(self, image):
        #image = cv.imread(file_path)
        annotated_image = image.copy()

        # Perform object detection using the Roboflow model
        result = self.model(image, show=False, conf=0.4, save=False)
        # while(True):0
        # print("a")
        bounding_boxes = []
        conf = []
        class_ = []
        for r in result:
            #print(r.boxes)
            boxes = r.boxes.cpu().numpy()
            bounding_boxes = boxes.xywh
            conf = boxes.conf
            class_ = boxes.cls
        
        #bounding_boxes = [box.tolist() for box in bounding_boxes]
        predictions = []
        #[{'x': box[0], 'y': box[1], 'width': box[2], 'height': box[3]} for box in bounding_boxes]
        for box, conf, class_ in zip(bounding_boxes.tolist(), conf.tolist(), class_.tolist()):
            x, y, width, height = box
            prediction = {
                'x': x,
                'y': y,
                'width': width/2,
                'height': height/2,
                'confidence': conf,
                'class': int(class_)
            }
            predictions.append(prediction)

        traffic_sign = []

        # Assuming result is a list and the names field is a dictionary inside each item of the list

        # Extract class names from predictions
        #labels = [class_mapping[item["class"]] for item in predictions]

        # Loop through detections and draw bounding boxes and labels
        for detection in predictions:
            # cv.imshow('Panneaux', annotated_image)
            # cv.waitKey(5)
            x, y, w, h = detection["x"], detection["y"], detection["width"], detection["height"]
            class_id = detection["class"]
            confidence = detection["confidence"]

            if (confidence < self.confidence_threshold):
                break

            # Round coordinates to integers
    
            x, y, w, h = round(x), round(y), round(w), round(h)
            # Crop the region defined by the bounding box
            cropped_region = image[y-h:y+h,x-w:x+w]
            
            # Draw a bounding box
            cv.rectangle(annotated_image, (x-w,y-h), (x+w,y+h), (0, 0, 255), 2)

            # Format the text (class name and confidence score)
            class_name = class_mapping[detection["class"]]
            text = f"{class_name}: {confidence:.2f}"  # Rounds confidence to 2 decimal places

            # Position for the text: slightly above the top-left corner of the box
            text_position = (int(x), int(y) - 2)

            # Calculate the width and height of the text box   
            (text_width, text_height), _ = cv.getTextSize(text, self.font, 0.5, 1)

            # Create a rectangle for the text background
            text_bg_position = (text_position[0], text_position[1] - text_height - 5, text_width + 5 , text_height + 10)

            # Draw the text background rectangle
            cv.rectangle(annotated_image, text_bg_position, (128, 0, 128), -1)  # -1 fills the rectangle

            # Draw the text on the image
            cv.putText(annotated_image, text, text_position, self.font, 0.5, (255, 255, 255), 1)

            if (class_name == 'city-signboard'):
                # Convert the cropped region to grayscale
                gray_cropped = cv.cvtColor(cropped_region, cv.COLOR_BGR2GRAY)

                # Noise reduction
                gray_cropped = cv.GaussianBlur(gray_cropped, (5, 5), 0)

                # Using Canny edge detection as an example
                gray_cropped = cv.Canny(gray_cropped, 100, 200)

                # Show the edge region
                # plt.imshow(cv.cvtColor(gray_cropped, cv.COLOR_BGR2RGB))
                # plt.title('Original Image')
                # plt.show()
                # dilateImg = cv.dilate(gray_cropped, (3,3))

                # # Define the structuring element (kernel)
                # kernel_size = 3
                # kernel = cv.getStructuringElement(cv.MORPH_RECT, (1, kernel_size))

                # # Apply morphological closing
                # closed_image = cv.morphologyEx(gray_cropped, cv.MORPH_CLOSE, kernel)

                # image_with_border = cv.copyMakeBorder(gray_cropped, 5, 5, 5, 5, cv.BORDER_CONSTANT, value=0)
                newImage = edge_connection(gray_cropped)
                # Find contours in the binary image
                contours, _ = cv.findContours(newImage, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

                # plt.imshow(cv.cvtColor(newImage, cv.COLOR_BGR2RGB))
                # plt.show()
                # Initialize the arrowhead direction
                direction = None

                if contours:
                    # Initialize variables to keep track of the largest closed contour and its area
                    largest_contour = max(contours, key=cv.contourArea)

                    # Simplify the contour
                    approx = cv.approxPolyDP(largest_contour, 0.01 * cv.arcLength(largest_contour, True), True)

                    # plt.imshow(cv.cvtColor(cropped_region, cv.COLOR_BGR2RGB))
                    # plt.show()
                    # plt.imshow(cv.cvtColor(approx, cv.COLOR_BGR2RGB))
                    # plt.show()
                    # Check if the contour could be an arrowhead (5 sides)
                    if len(approx) >= 5:
                        # Calculate the centroid of the arrowhead
                        M = cv.moments(approx)
                        if M["m00"] != 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            centroid = np.array([cx, cy])

                            # Find the furthest point from the centroid
                            furthest_distance = 0
                            tip_index = None
                            for i, point in enumerate(approx):
                                distance = np.linalg.norm(point[0] - centroid)
                                if distance > furthest_distance:
                                    furthest_distance = distance
                                    tip_index = i

                            if tip_index is not None:
                                # Get the tip of the arrowhead
                                tip = approx[tip_index][0]

                                # Determine the direction of the arrowhead
                                direction = 'right' if tip[0] > centroid[0] else 'left'

                        # Draw the contour and the box on the original image
                        cv.drawContours(annotated_image, [approx + [x-w, y-h]], -1, (0, 255, 0), 2)

                # Use EasyOCR to recognize text in the cropped region
                results = reader.readtext(cropped_region)

                #valid_texts = []

                # Check the number of OCR results
                if len(results) > 1:
                    # More than one result, combine the valid texts
                    combined_city_names = [city_name.upper() for (bbox, city_name, prob) in results if prob > 0.2 and re.match("^[A-Za-z ]+$", city_name)]
                    #combined_city_names = ' '.join(valid_texts)
                elif len(results) == 1:
                    # Only one result, use it if it's valid
                    bbox, city_name, prob = results[0]
                    if prob > 0.2 and re.match("^[A-Za-z ]+$", city_name):
                        combined_city_names = city_name.upper()
                    else:
                        combined_city_names = ''
                else:
                    # No valid results
                    combined_city_names = ''

                if combined_city_names:
                    if len(results) > 1:
                        for city_name in combined_city_names:
                            new_city_name = detectCity(city_name)
                            #print(new_city_name)
                            #print(new_city)
                            if direction is not None:
                                text = f'Tournez à {direction} pour la direction {new_city_name.upper()}'
                            else:
                                text = f"Direction indéterminée vers {new_city_name.upper()}"
                                continue
                            print(text)
                            traffic_sign.append(TrafficSign(type=class_name, cityName=new_city_name.upper(), direct = direction, x=x, y=y, width=w, height=h, confidence=confidence))
                    else:
                        new_city_name = detectCity(combined_city_names)
                        #print(new_city)
                        #print(new_city_name)
                        if direction is not None:
                            text = f'Tournez à {direction} pour la direction {new_city_name.upper()}'
                        else:
                            text = f"Direction indéterminée vers {new_city_name.upper()}"
                            continue
                        print(text)
                        traffic_sign.append(TrafficSign(type=class_name, cityName=new_city_name.upper(), direct = direction, x=x, y=y, width=w, height=h, confidence=confidence))
            else:
                sign = TrafficSign(type=class_name, cityName="", direct="", x=x, y=y, width=w, height=h, confidence=confidence)
                traffic_sign.append(sign)
            # cv.imshow('Panneaux', annotated_image)
            # cv.waitKey(5)           

        return annotated_image, traffic_sign

# connect pixel at the border of the image
def edge_connection(b_image):
    newImage = b_image.copy()
    height, width = b_image.shape

    # start edge if the following order : left, right, top, bot
    start_edge = [0, 0, 0, 0]

    end_edge = [0, 0, 0, 0]

    #detect 2 points on the border left and right
    for i in range(0, height):
        if b_image[i, 0] == 255:
            if not start_edge[0]:
                start_edge[0] = i
            end_edge[0] = i
        if b_image[i, width-1] == 255:
            if not start_edge[1]:
                start_edge[1] = i
            end_edge[1] = i

    #detect 2 points on the border top and bot
    for j in range(0, width):
        if b_image[0, j] == 255:
            if not start_edge[2]:
                start_edge[2] = j
            end_edge[2] = j
        if b_image[height-1, j] == 255:
            if not start_edge[3]:
                start_edge[3] = j
            end_edge[3] = j

    #print(start_edge, end_edge)
    #draw lines
   # cv.line(newImage, (start_edge[0], 0), (end_edge[0], 0), 255, 1))
    if start_edge[0]:
        cv.line(newImage, (0, start_edge[0]), (0, end_edge[0]), 255, 1)      
    if start_edge[1]:
        cv.line(newImage, (width-1, start_edge[1]), (width-1, end_edge[1]), 255, 1)
    if start_edge[2]:
        cv.line(newImage, (start_edge[2], 0), (end_edge[2], 0), 255, 1)
    if start_edge[3]:  
        cv.line(newImage, (start_edge[3], height-1), (end_edge[3], height-1), 255, 1)
    
    return newImage


def determine_action(traffic_sign):
    """
    Classify the traffic sign and determine the action for an autonomous car based on the detected traffic sign.

    :param traffic_sign: TrafficSign namedtuple
    :return: Tuple containing the classification and action to be taken by the autonomous car
    """
    type = traffic_sign.type

    category, action = sign_info.get(type, ("other", "Panneau de circulation inconnu, procédez avec prudence"))
    sign = TrafficSign(category=category, type=type, label=action, x=traffic_sign.x, y=traffic_sign.y, width=traffic_sign.width, height=traffic_sign.height, confidence=traffic_sign.confidence)
    return sign

def readAllCities(csv_path):
    # Open the CSV file and read its contents
    with open(csv_path, 'r') as file:
        # Create a CSV reader object
        csv_reader = csv.reader(file)

        # Iterate through the rows in the CSV file
        for row in csv_reader:
            # Each 'row' variable contains a list of values from a single row
            #print(row[0])
            citiesName.append(row[0].upper())

# detect the most probable city in the list of all cities in France
def detectCity(cityNameDetected):
    dist = 0
    minDist = 100
    cityMin = ""
    for city in citiesName:
        #compute levenshtein distance
        dist = levenshtein_distance(cityNameDetected, city)

        #find the minimum distance
        if minDist > dist:
            minDist = dist
            cityMin = city

    if minDist > 2:
        cityMin = cityNameDetected
    return cityMin

