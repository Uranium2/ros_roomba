import sys
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
import rospy
import tensorflow as tf

# This is needed since the notebook is stored in the object_detection folder.
sys.path.append("..")
from object_detection.utils import label_map_util
from object_detection.utils import ops as utils_ops
from object_detection.utils import visualization_utils as vis_util

# EDIT TO THE FOLDER OF THIS FILE. Since this file is copied to "~.ros/"
PROJECT_FOLDER = Path("/ROS", "src", "mr_vision", "src", "mr_vision")
MODEL_NAME = "fine_tuned_model1"
# Path to frozen detection graph.
# This is the actual model that is used for the object detection.
PATH_TO_CKPT = Path(PROJECT_FOLDER, MODEL_NAME, "frozen_inference_graph.pb")
# List of the strings that is used to add correct label for each box.
PATH_TO_LABELS = Path(PROJECT_FOLDER, MODEL_NAME, "label_map.pbtxt")

assert PATH_TO_CKPT.is_file()
assert PATH_TO_LABELS.is_file()


class Predict:
    def __init__(self) -> None:
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(str(PATH_TO_CKPT.resolve()), "rb") as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name="")

        self.category_index = Predict._get_category_index()

    @staticmethod
    def _get_category_index() -> Dict[int, Dict[str, Any]]:
        """
        Exemple of label dict :
        {1: {'id': 1, 'name': 'Stop_sign'}}
        """
        label_map = label_map_util.load_labelmap(str(PATH_TO_LABELS.resolve()))
        categories = label_map_util.convert_label_map_to_categories(
            label_map, max_num_classes=1, use_display_name=True
        )
        return label_map_util.create_category_index(categories)

    def run_inference_for_single_image(self, image: np.ndarray) -> Dict[str, Any]:
        with self.detection_graph.as_default():
            with tf.Session() as sess:
                # Get handles to input and output tensors
                ops = tf.get_default_graph().get_operations()
                all_tensor_names = {output.name for op in ops for output in op.outputs}
                tensor_dict = {}
                for key in [
                    "num_detections",
                    "detection_boxes",
                    "detection_scores",
                    "detection_classes",
                    "detection_masks",
                ]:
                    tensor_name = f"{key}:0"
                    if tensor_name in all_tensor_names:
                        tensor_dict[key] = tf.get_default_graph().get_tensor_by_name(
                            tensor_name
                        )
                if "detection_masks" in tensor_dict:
                    # The following processing is only for single image
                    detection_boxes = tf.squeeze(tensor_dict["detection_boxes"], [0])
                    detection_masks = tf.squeeze(tensor_dict["detection_masks"], [0])
                    # Reframe is required to translate mask from box coordinates
                    # to image coordinates and fit the image size.
                    real_num_detection = tf.cast(
                        tensor_dict["num_detections"][0], tf.int32
                    )
                    detection_boxes = tf.slice(
                        detection_boxes, [0, 0], [real_num_detection, -1]
                    )
                    detection_masks = tf.slice(
                        detection_masks, [0, 0, 0], [real_num_detection, -1, -1]
                    )
                    detection_masks_reframed = (
                        utils_ops.reframe_box_masks_to_image_masks(
                            detection_masks,
                            detection_boxes,
                            image.shape[0],
                            image.shape[1],
                        )
                    )
                    detection_masks_reframed = tf.cast(
                        tf.greater(detection_masks_reframed, 0.5), tf.uint8
                    )
                    # Follow the convention by adding back the batch dimension
                    tensor_dict["detection_masks"] = tf.expand_dims(
                        detection_masks_reframed, 0
                    )
                image_tensor = tf.get_default_graph().get_tensor_by_name(
                    "image_tensor:0"
                )

                # Run inference
                output_dict = sess.run(
                    tensor_dict, feed_dict={image_tensor: np.expand_dims(image, 0)}
                )

                # all outputs are float32 numpy arrays, so convert types as appropriate
                output_dict["num_detections"] = int(output_dict["num_detections"][0])
                output_dict["detection_classes"] = output_dict["detection_classes"][
                    0
                ].astype(np.uint8)
                output_dict["detection_boxes"] = output_dict["detection_boxes"][0]
                output_dict["detection_scores"] = output_dict["detection_scores"][0]
                if "detection_masks" in output_dict:
                    output_dict["detection_masks"] = output_dict["detection_masks"][0]
        return output_dict

    def predict_result(self, image_np: np.ndarray) -> Tuple[np.ndarray, bool]:
        rospy.loginfo("Making a prediction")
        # Actual detection.
        output_dict = self.run_inference_for_single_image(image_np)
        index_best_prediction = np.argmax(output_dict["detection_scores"])
        score_best_prediction = output_dict["detection_scores"][index_best_prediction]
        vis_util.visualize_boxes_and_labels_on_image_array(
            image_np,
            output_dict["detection_boxes"],
            output_dict["detection_classes"],
            output_dict["detection_scores"],
            self.category_index,
            instance_masks=output_dict.get("detection_masks"),
            use_normalized_coordinates=True,
            line_thickness=8,
        )
        return image_np, score_best_prediction >= 0.5
