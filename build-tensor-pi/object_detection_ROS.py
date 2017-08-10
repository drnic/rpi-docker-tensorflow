#!/usr/bin/env python
# coding: utf-8

# ## Object detection through ROS
# This script is for performing object detection on a raw image from a camera in real time using the Google Tensorflow API based on Faster RCNN

import rospy
from sensor_msgs.msg import Image as ImageMessage
from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2

## TensorFlow related imports
import numpy as np
import tensorflow as tf

## Object detection imports
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

# ## Path to models

MODEL_NAME = '/mobilenet'
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_NAME + '/frozen_inference_graph.pb'

# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = '/tensorflow_models/object_detection/data/mscoco_label_map.pbtxt'

NUM_CLASSES = 90


# ## Starting with ROS node initially
class RosTensorFlow():

    def __init__(self):

        """
        Class for the ROS node which runs tensorflow for object detection.

            Subscribes to: The node Subscribes to the topic image
            Publish:
                boxes: The boxes of objects detected
                classes: The classes of objects detected as numbers
                scores: The scores on each of the class detected
                num_detections: The number of object detected
        """

        self._cv_bridge = CvBridge()

        self._sub = rospy.Subscriber('image', ImageMessage, self.callback)
        self.image_pub = rospy.Publisher('output/image_raw', ImageMessage, queue_size=2)

        ## Load the tensorflow model into memory
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
          od_graph_def = tf.GraphDef()
          with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
            od_graph_def.ParseFromString(fid.read())
            tf.import_graph_def(od_graph_def, name='')


        ## Load the label map
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)

        self._session = tf.Session(graph=self.detection_graph)


    def callback(self, image_msg):
        cv_image = self._cv_bridge.imgmsg_to_cv2(image_msg, "bgr8")

        ## To send the image as a topic with the boxes, convert it into numpy array
        #img2= cv2.resize(cv_image,dsize=(224,224), interpolation = cv2.INTER_CUBIC)
        #Numpy array
        image_np = np.asarray(cv_image)

        # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        # Each box represents a part of the image where a particular object was detected.
        boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        # Each score represent how level of confidence for each of the objects.
        # Score is shown on the result image, together with the class label.
        scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        # Actual detection.
        (boxes, scores, classes, num_detections) = self._session.run(
            [boxes, scores, classes, num_detections],
            feed_dict={image_tensor: image_np_expanded})

            # Visualization of the results of a detection.
        vis_util.visualize_boxes_and_labels_on_image_array(
          image_np,
          np.squeeze(boxes),
          np.squeeze(classes).astype(np.int32),
          np.squeeze(scores),
          self.category_index,
          use_normalized_coordinates=True,
          line_thickness=8)


        # Publish new image
        msg = self._cv_bridge.cv2_to_imgmsg(image_np,encoding="bgr8")
        self.image_pub.publish(msg)

    def main(self):
        rospy.spin()


# ## Main loop

if __name__ == '__main__':
    tensor = RosTensorFlow()
    rospy.init_node('rostensorflow')
    tensor.main()
