
import cv2
from ultralytics import YOLO
import yaml


# Read config file
with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

print(config)
# Initialize YOLOv8
yolo = YOLO("yolov8n.pt").to(config["device"])
print("YOLOv8 initialized...")


if config['source_type'] == 'Image':

    # Read the image

    image_path = config['source']

    image = cv2.imread(image_path)

    # Run YOLOv8 inference on the image
    results = yolo.predict(source = image_path,classes=config['classes'],save = config['save'], device=config['device'],imgsz=config['imgsz'],save_crop=config['save_crop'])

    # # Display the annotated image

    if config['show_results']:

        # Visualize the results on the image

        annotated_image = results[0].plot()

        cv2.imshow("YOLOv8 Inference", annotated_image)

        cv2.waitKey(0)

        cv2.destroyAllWindows()

elif config['source_type'] == 'Webcam' or config['source_type'] == 'Video':

    # Loop through the webcam frames

    # Open the webcam

    video_path = config['source']

    if config['source_type'] == 'Video':

        cap = cv2.VideoCapture(video_path)

    else:

        cap = cv2.VideoCapture(0)

    while cap.isOpened():

        # Read a frame from the webcam

        success, frame = cap.read()

        print(success)

        if success:

            # Run YOLOv8 inference on the frame

            #results = model(frame)

            results = yolo.predict(source = frame,classes=config['classes'],save = config['save'], device=config['device'],imgsz=config['imgsz'],save_crop=config['save_crop'])

            # check if there are people
            # for r in results:
            print(results)  # print the Boxes object containing the detection bounding boxes

            if config['show_results']:

                # Visualize the results on the frame

                annotated_frame = results[0].plot()

                # Display the annotated frame

                cv2.imshow("YOLOv8 Inference", annotated_frame)

                # Break the loop if 'q' is pressed

                if cv2.waitKey(1) & 0xFF == ord("q"):

                    break

        else:

            # Break the loop if the end of the video is reached

            break

    # Release the video capture object and close the display window

    cap.release()

    cv2.destroyAllWindows()
