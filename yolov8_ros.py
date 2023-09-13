
import cv2
import rospy
from std_msgs.msg import String 
from geometry_msgs.msg import Point, PoseArray, Pose,Quaternion
from ultralytics import YOLO
import yaml
from std_msgs.msg import Header


# Read config file
with open("config.yaml", "r") as f:
    config = yaml.safe_load(f)

print(config)
# Initialize YOLOv8
yolo = YOLO("yolov8n.pt").to(config["device"])
print("YOLOv8 initialized...")


# Prepare ROS
pub = rospy.Publisher('object_detections', PoseArray, queue_size=10)
rospy.init_node('yolov8', anonymous=True)
rate = rospy.Rate(config['rate']) 


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
    
    # check the camera fps and set it to config fps 
    fps = cap.get(cv2.CAP_PROP_FPS)
    cap.set(cv2.CAP_PROP_FPS, config['fps'])
    print("FPS: ", fps,cap.get(cv2.CAP_PROP_FPS))

    while cap.isOpened():

        # Read a frame from the webcam
        success, frame = cap.read()
        frame_id = 0
        if success:

            # Run YOLOv8 inference on the frame
            results = yolo.predict(source = frame,classes=config['classes'],conf = config['conf'],save = config['save'], device=config['device'],imgsz=config['imgsz'],save_crop=config['save_crop'])
            # Create a PoseArray message
            pose_array_msg = PoseArray()
            pose_array_msg.header = Header()
            pose_array_msg.header.stamp = rospy.Time.now()
            pose_array_msg.header.frame_id = str(frame_id)
            # results is for all detections more info -> https://docs.ultralytics.com/reference/engine/results/#ultralytics.engine.results.Results
            # results contains boxes and other info for all classes. Box has the following attributes ->xyxy,conf,cls,id,xywh,xywhn,xyxyn,data ->cls is class and conf is detection confidence

            # Use list comprehension to construct Pose messages and assign to poses field  - > the four value is put on orientation field
            for pedes in results:
                # To change to cpu use bbox[0].cpu() instead of bbox[0]

                # Fill pose array
                pose_array_msg.poses = [Pose(position=Point(x=bbox[0], 
                                                            y=bbox[1], 
                                                            z=bbox[2]),
                                            orientation= Quaternion(x=bbox[3], y=0.0, z=0.0, w=1.0)) 
                                        for bbox in getattr(pedes.boxes, config['bbox'])]
            
            print(pose_array_msg)
            if config['show_results']:

                # Visualize the results on the frame

                annotated_frame = results[0].plot()

                # Display the annotated frame

                cv2.imshow("YOLOv8 Inference", annotated_frame)

                # Break the loop if 'q' is pressed

                if cv2.waitKey(1) & 0xFF == ord("q"):

                    break
            pub.publish(pose_array_msg)
            rate.sleep()
            frame_id += 1

        else:

            # Break the loop if the end of the video is reached

            break

    # Release the video capture object and close the display window

    cap.release()

    cv2.destroyAllWindows()
