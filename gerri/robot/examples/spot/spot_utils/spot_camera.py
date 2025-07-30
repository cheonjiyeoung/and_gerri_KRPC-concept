import datetime, time
import cv2
import sys
import queue
import numpy as np
import threading
from pubsub import pub
from bosdyn.api import image_pb2
from bosdyn.client.world_object import WorldObjectClient
from bosdyn.api import world_object_pb2
from bosdyn.client.image import ImageClient, build_image_request

class SpotCamera():
    def __init__(self,robot,spot_info):
        self.spot_info = spot_info
        self.frame_queue = queue.Queue()
        self.image_client = robot.ensure_client(ImageClient.default_service_name)
        self.world_object_client = robot.ensure_client(WorldObjectClient.default_service_name)
        self.last_image_time = 0
        # self.last_frame = None
        self.receive_thread = threading.Thread(target=self.get_frame,daemon=True)
        self.receive_thread.start()

    def get_camera_image(self, image_client, camera_type):
        image_request = [
            build_image_request(image_source_name=camera_type,
                                pixel_format=image_pb2.Image.PIXEL_FORMAT_RGB_U8)
        ]
        try:
            image_responses = image_client.get_image(image_request)
        except Exception as e:
            print(e)
        ####
        ### camera type : 'hand_color_image' = gripper , 'back_fisheye_image' = back, 'frontleft_fisheye_image' = front

        # image_responses = image_client.get_image_from_sources([camera_type])
        if len(image_responses) != 1:
            print('Error: did not get exactly one image response.')
            sys.exit(1)

        resp = image_responses[0]
        image = image_responses[0]

        try:
            if image.shot.image.pixel_format == image_pb2.Image.PIXEL_FORMAT_RGB_U8:
                # dtype = np.uint16
                dtype=np.uint8
            else:
                dtype = np.uint8
        except:
            pass
        try:
            img = np.fromstring(image.shot.image.data, dtype=dtype)
        except:
            pass
        try:
            if image.shot.image.format == image_pb2.Image.FORMAT_RAW:
                img = img.reshape(image.shot.image.rows, image.shot.image.cols, 3)
            else:
                img = cv2.imdecode(img, -1)
        except Exception as e:
            print(e)
        return img

    def get_frame(self):
        try:
            if self.spot_info["using_arm"]:
                gripper_camera = self.get_camera_image(self.image_client, 'hand_color_image')  ##(480, 640, 3)
                gripper_camera_resize = cv2.resize(gripper_camera,(320,360))
                pub.sendMessage("gripper_camera",frame=gripper_camera_resize)
        
            image_back = self.get_camera_image(self.image_client, 'back_fisheye_image')   ##(480, 640, 3)
            image_back = cv2.putText(image_back, "back", (10, image_back.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)

            image_left = self.get_camera_image(self.image_client, 'left_fisheye_image')
            image_left = cv2.putText(image_left, "left", (10, image_back.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)

            image_right = self.get_camera_image(self.image_client, 'right_fisheye_image')
            image_right = cv2.rotate(image_right,cv2.ROTATE_180)
            image_right = cv2.putText(image_right, "right", (10, image_back.shape[0] - 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)

            multi_image = np.hstack((image_left,image_back,image_right))
            # multi_image = cv2.cvtColor(multi_image,cv2.COLOR_BGR2RGB)
            # h,w,c = multi_image.shape
            # getimagetime = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3] + str(f'    ')+ str(f'size: {int(w)}X{int(h)}')
            # cv2.putText(multi_image, getimagetime, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
            return multi_image
        except Exception as e:
            print(f"spot_camera : get_frame() error ({e})")

    def get_fiducial_objects(self,dock=False):
        """Get all fiducials that Spot detects with its perception system."""
        # Get all fiducial objects (an object of a specific type).
        request_fiducials = [world_object_pb2.WORLD_OBJECT_APRILTAG]
        fiducial_objects = self.world_object_client.list_world_objects(
            object_type=request_fiducials).world_objects
        if len(fiducial_objects) > 0:
            # Return the first detected fiducial.
            if dock:
                for fiducial_object in fiducial_objects:
                    if fiducial_object.apriltag_properties.tag_id > 499:
                        return fiducial_object.apriltag_properties.tag_id
                    else:
                        continue
            else:
                return fiducial_objects[0].apriltag_properties.tag_id
        # Return none if no fiducials are found.
        return None