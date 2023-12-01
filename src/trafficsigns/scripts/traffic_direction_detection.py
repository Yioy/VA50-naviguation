import cv2 as cv
from trafficsign import TrafficSign
from roboflow import Roboflow
import easyocr

class TrafficDirectionDetector(object):
    def __init__(self):
        # Initialize the Roboflow model
        rf = Roboflow(api_key="R0kjYGtH3txBw2OYEWIH")
        project = rf.workspace().project("signboard-detection-smrhy")
        self.model = project.version(3).model

        self.confidence_threshold = 0.3
        self.font = cv.FONT_HERSHEY_SIMPLEX

    def get_traffic_direction(self, image):

        # Perform object detection using the Roboflow model
        result = self.model.predict(image, confidence=40, overlap=30).json()

        traffic_direction = []

        # Extract labels and detections
        labels = [item["class"] for item in result["predictions"]]
        detections = result["predictions"]

        # Initialize EasyOCR reader
        reader = easyocr.Reader(['fr'])  

        # Loop through detections and draw bounding boxes and labels
        for detection in detections:
            x, y, w, h = detection["x"], detection["y"], detection["width"], detection["height"]
            class_id = labels.index(detection["class"])
            confidence = detection["confidence"]

            # Draw a bounding box
            cv.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # Crop the region defined by the bounding box
            cropped_region = image[y:y + h, x:x + w]

            # Convert the cropped region to grayscale
            gray_cropped = cv.cvtColor(cropped_region, cv.COLOR_BGR2GRAY)

            # Apply thresholding to create a binary image
            _, binary_cropped = cv.threshold(gray_cropped, 127, 255, cv.THRESH_BINARY)

            # Find contours in the binary image
            contours, _ = cv.findContours(binary_cropped, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

            # Initialize the arrowhead direction
            direction = None

            for contour in contours:
                # Calculate the centroid of the contour
                M = cv.moments(contour)
                if M["m00"] != 0:  # Avoid division by zero
                    cX = int(M["m10"] / M["m00"])
                else:
                    cX = 0

                # Determine the arrowhead direction based on centroid position
                if cX < cropped_region.shape[1] / 2:
                    direction = "Left"
                else:
                    direction = "Right"

            # Use EasyOCR to recognize text in the cropped region
            results = reader.readtext(cropped_region)

            for (text, _, prob) in results:
                if prob > 0.5:
                    # Put the recognized text on the image
                    cv.putText(image, text.toLower(), (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

                    traffic_direction.append(TrafficSign(category="trafficdirection", type=text.toLower(), label=direction, x=x, y=y, width=w, height=h, confidence=confidence))
        
        return image, traffic_direction