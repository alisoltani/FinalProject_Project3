from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2
import os

MAX_IMAGE_WIDTH = 300
MAX_IMAGE_HEIGHT = 300
MIN_CONFIDENCE = 0.5

class TLClassifier(object):
    def __init__(self):
        self.session = None
        self.model_graph = None
	self.tl_classes = {1: TrafficLight.RED, 2: TrafficLight.YELLOW, 3: TrafficLight.GREEN, 4: TrafficLight.UNKNOWN}

        self.classifier_model_path = os.path.dirname(os.path.realpath(__file__)) + "/../model/tl_classifier.pb"
        self.load_model(self.classifier_model_path)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        image_tensor = self.model_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.model_graph.get_tensor_by_name('detection_boxes:0')
        detection_confidence = self.model_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.model_graph.get_tensor_by_name('detection_classes:0')

        image = cv2.resize(image, (MAX_IMAGE_WIDTH, MAX_IMAGE_HEIGHT))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        (boxes, confidence, classes) = self.session.run(
            [detection_boxes, detection_confidence, detection_classes],
            feed_dict={image_tensor: np.expand_dims(image, axis=0)})
	
	boxes = np.squeeze(boxes)
        confidence = np.squeeze(confidence)
        classes = np.squeeze(classes)
        

        for i, box in enumerate(boxes):
            if confidence[i] > MIN_CONFIDENCE:
                light_class = self.tl_classes[classes[i]] 
                return light_class, confidence[i]

        return TrafficLight.UNKNOWN, -1


    def load_model(self, model_path):

        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        self.model_graph = tf.Graph()

        with tf.Session(graph=self.model_graph, config=config) as sess:
            self.session = sess
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(model_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

