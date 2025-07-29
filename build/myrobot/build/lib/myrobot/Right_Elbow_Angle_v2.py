import cv2
import mediapipe as mp
import math

mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose
R_ANGLE = 0
POSE_STATUS = ""

# Calculate Angles
def calculate_angle(a, b, c):
    x_a, y_a = a[0:2]
    x_b, y_b = b[0:2]
    x_c, y_c = c[0:2]

    ba_x = x_a - x_b
    ba_y = y_a - y_b
    ba_size = math.sqrt((ba_x**2) + (ba_y**2))

    bc_x = x_c - x_b
    bc_y = y_c - y_b
    bc_size = math.sqrt((bc_x**2) + (bc_y**2))

    radians = math.acos(((ba_x * bc_x) + (ba_y * bc_y)) / (ba_size * bc_size))
    angle = int(180 - math.degrees(radians))

    return angle


def RightElbow():
    
    global R_ANGLE, POSE_STATUS
    
    def check_right_arm(results, image):
        landmarks = results.pose_landmarks.landmark

        # Get coordinates
        # Right Arm
        # Right Shoulder Index = 12

        R_shoulder = [landmarks[12].x, landmarks[12].y, landmarks[12].visibility]

        # Right Elbow Index = 14

        R_elbow = [landmarks[14].x, landmarks[14].y, landmarks[14].visibility]

        # Right Wrist Index = 16

        R_wrist = [landmarks[16].x, landmarks[16].y, landmarks[16].visibility]

        R_vis = all(list(map(lambda i: i > 0.5, [R_shoulder[2], R_elbow[2], R_wrist[2]])))

        if R_vis == True:
            R_angle = calculate_angle(R_shoulder, R_elbow, R_wrist)
            cv2.putText(image, f"Right Elbow Angle: {str(R_angle)}", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, lineType=cv2.LINE_AA)
        else:
            R_angle = 0
            cv2.putText(image, f"Right Elbow Not Detected", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, lineType=cv2.LINE_AA)
        return R_angle
    
    cap = cv2.VideoCapture(0)

    with mp_pose.Pose(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
        while cap.isOpened():
            ret, frame = cap.read()

            # Convert from BGR to RGB and flip the video
            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # image = cv2.flip(image, 1)
            image.flags.writeable = False

            # Detection
            results = pose.process(image)

            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # Extract landmarks
            if not results.pose_landmarks:
                R_ANGLE = 0
                cv2.putText(image, "Pose Not Detected", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, lineType=cv2.LINE_AA)
            else:
                # landmarks = results.pose_landmarks.landmark

                # # Get coordinates
                # # Right Arm
                # # Right Shoulder Index = 12

                # R_shoulder = [landmarks[12].x, landmarks[12].y, landmarks[12].visibility]

                # # Right Elbow Index = 14

                # R_elbow = [landmarks[14].x, landmarks[14].y, landmarks[14].visibility]

                # # Right Wrist Index = 16

                # R_wrist = [landmarks[16].x, landmarks[16].y, landmarks[16].visibility]

                # R_vis = all(list(map(lambda i: i > 0.5, [R_shoulder[2], R_elbow[2], R_wrist[2]])))

                # if R_vis == True:
                #     R_angle = calculate_angle(R_shoulder, R_elbow, R_wrist)
                #     cv2.putText(image, f"Right Elbow Angle: {str(R_angle)}", (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                R_ANGLE = check_right_arm(results, image)
                # # Render detections
                mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS, mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2), mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2))

            if POSE_STATUS:
                cv2.putText(image, POSE_STATUS, (100, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # image = cv2.flip(image, 1)
            cv2.imshow("Mediapipe Feed", image)
            
            # print(R_ANGLE)
            
            if cv2.waitKey(10) & 0xFF == ord("q"):
                # print(image.shape)
                # print(results.pose_landmarks.landmark)
                break

        cap.release()
        cv2.destroyAllWindows()


#RightElbow()
