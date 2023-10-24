# https://github.com/NVIDIA/TensorRT/tree/release/8.4/samples/python/yolov3_onnx

import tensorrt as trt
import numpy as np
import common
import os
import argparse

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import cv2

from PIL import ImageDraw, ImageFont
from data_processing import PreprocessYOLO

TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

def build_engine(onnx_file_path, engine_file_path="./lab8_model.trt"):
    if os.path.exists(engine_file_path):
        print("Reading Engine From File {}".format(engine_file_path))

        with open(engine_file_path, "rb") as f:
            runtime = trt.Runtime(TRT_LOGGER)

            return runtime.deserialize_cuda_engine(f.read())

    builder = trt.Builder(TRT_LOGGER)
    config = builder.create_builder_config()
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, TRT_LOGGER)
    runtime = trt.Runtime(TRT_LOGGER)


    config.max_workspace_size = 1 << 20
    builder.max_batch_size = 1
    # Enable FP16
    config.set_flag(trt.BuilderFlag.FP16)

    with open(onnx_file_path, 'rb') as model:
        if not parser.parse(model.read()):
            print("ERROR: Failed to parse the ONNX file.")

            return None

    network.get_input(0).shape = [1, 3, 180, 320]

    print("Complete parsing ONNX file.")
    print("Building engine from file {}".format(onnx_file_path))

    plan = builder.build_serialized_network(network, config)
    engine = runtime.deserialize_cuda_engine(plan)

    save_engine(plan=plan, file_name=engine_file_path)

    # engine = builder.build_cuda_engine(network)
    return engine

def save_engine(plan, file_name):
    with open(file_name, 'wb') as f:
        f.write(plan)

def preprocess(img):
    pass

def postprocess(img):
    pass

def draw_bboxes(image_raw, bboxes, confidences, categories, all_categories, bbox_color='blue'):
    """Draw the bounding boxes on the original input image and return it.
    Keyword arguments:
    image_raw -- a raw PIL Image
    bboxes -- NumPy array containing the bounding box coordinates of N objects, with shape (N,4).
    categories -- NumPy array containing the corresponding category for each object,
    with shape (N,)
    confidences -- NumPy array containing the corresponding confidence for each object,
    with shape (N,)
    all_categories -- a list of all categories in the correct ordered (required for looking up
    the category name)
    bbox_color -- an optional string specifying the color of the bounding boxes (default: 'blue')
    """
    np.random.seed(1)
    colors = [[np.random.randint(0, 255) for _ in range(3)] for _ in range(len(all_categories))]
    text_size = 20
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", text_size)

    draw = ImageDraw.Draw(image_raw)
    print(bboxes, confidences, categories)
    for box, score, category in zip(bboxes, confidences, categories):
        x_coord, y_coord, width, height = box
        left = max(0, np.floor(x_coord + 0.5).astype(int))
        top = max(0, np.floor(y_coord + 0.5).astype(int))
        right = min(image_raw.width, np.floor(x_coord + width + 0.5).astype(int))
        bottom = min(image_raw.height, np.floor(y_coord + height + 0.5).astype(int))

        bbox_color = tuple(colors[category]) or bbox_color
        draw.rectangle(((left, top), (right, bottom)), outline=bbox_color, width=3)
        draw.text((left, top - 20), '{0} {1:.2f}'.format(all_categories[category], score), fill=bbox_color, font=font)

    return image_raw

def voting_suppression(result_box, iou_threshold = 0.5):
    votes = np.zeros(result_box.shape[0])
    for ind, box in enumerate(result_box):
        for box_validation in result_box:
            if IoU(box_validation, box) > iou_threshold:
                votes[ind] += 1
    return (-votes).argsort()

def postprocess(result, threshold=0.9):
    validation_result = []
    result_prob = []
    final_dim = [5, 10]
    input_dim = [180, 320]
    anchor_size = [(input_dim[0] / final_dim[0]), (input_dim[1] / final_dim[1])]

    for ind_row in range(final_dim[0]):
        for ind_col in range(final_dim[1]):
            grid_info = grid_cell(ind_col, ind_row)
            validation_result_cell = []
            if result[0, ind_row, ind_col] >= threshold:
                c_x = grid_info[0] + anchor_size[1]/2 + result[1, ind_row, ind_col]
                c_y = grid_info[1] + anchor_size[0]/2 + result[2, ind_row, ind_col]
                w = result[3, ind_row, ind_col] * input_dim[1]
                h = result[4, ind_row, ind_col] * input_dim[0]
                x1, y1, x2, y2 = bbox_convert(c_x, c_y, w, h)
                x1 = np.clip(x1, 0, input_dim[1])
                x2 = np.clip(x2, 0, input_dim[1])
                y1 = np.clip(y1, 0, input_dim[0])
                y2 = np.clip(y2, 0, input_dim[0])
                validation_result_cell.append(x1)
                validation_result_cell.append(y1)
                validation_result_cell.append(x2)
                validation_result_cell.append(y2)
                result_prob.append(result[0, ind_row, ind_col])
                validation_result.append(validation_result_cell)
    validation_result = np.array(validation_result)
    result_prob = np.array(result_prob)
    return validation_result, result_prob


def DisplayLabel(img, bboxs):
    # image = np.transpose(image.copy(), (1, 2, 0))
    # fig, ax = plt.subplots(1, figsize=(6, 8))
    image = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)
    fig, ax = plt.subplots(1)
    edgecolor = [1, 0, 0]
    if len(bboxs) == 1:
        bbox = bboxs[0]
        ax.add_patch(patches.Rectangle((bbox[0] - bbox[2]/2, bbox[1] - bbox[3]/2), bbox[2], bbox[3], linewidth=1, edgecolor=edgecolor, facecolor='none'))
    elif len(bboxs) > 1:
        for bbox in bboxs:
            ax.add_patch(patches.Rectangle((bbox[0] - bbox[2]/2, bbox[1] - bbox[3]/2), bbox[2], bbox[3], linewidth=1, edgecolor=edgecolor, facecolor='none'))
    ax.imshow(image)
    plt.show()

# convert feature map coord to image coord
def grid_cell(cell_indx, cell_indy):
    final_dim = [5, 10]
    input_dim = [180, 320]
    anchor_size = [(input_dim[0] / final_dim[0]), (input_dim[1] / final_dim[1])]

    stride_0 = anchor_size[1]
    stride_1 = anchor_size[0]
    return np.array([cell_indx * stride_0, cell_indy * stride_1, cell_indx * stride_0 + stride_0, cell_indy * stride_1 + stride_1])

# convert from [c_x, c_y, w, h] to [x_l, y_l, x_r, y_r]
def bbox_convert(c_x, c_y, w, h):
    return [c_x - w/2, c_y - h/2, c_x + w/2, c_y + h/2]

# convert from [x_l, y_l, x_r, x_r] to [c_x, c_y, w, h]
def bbox_convert_r(x_l, y_l, x_r, y_r):
    return [x_l/2 + x_r/2, y_l/2 + y_r/2, x_r - x_l, y_r - y_l]

# calculating IoU
def IoU(a, b):
    # referring to IoU algorithm in slides
    inter_w = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    inter_h = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter_ab = inter_w * inter_h
    area_a = (a[3] - a[1]) * (a[2] - a[0])
    area_b = (b[3] - b[1]) * (b[2] - b[0])
    union_ab = area_a + area_b - inter_ab
    return inter_ab / union_ab

def assign_label(label):
    label_gt = np.zeros((5, final_dim[0], final_dim[1]))
    IoU_threshold = 0.01
    IoU_max = 0
    IoU_max_ind = [0, 0]

    for ind_row in range(final_dim[0]):
        for ind_col in range(final_dim[1]):
            label_assign = 0
            grid_info = grid_cell(ind_col, ind_row)
            label_bbox = bbox_convert(label[0], label[1], label[2], label[3])
            IoU_value = IoU(label_bbox, grid_info)
            if IoU_value > IoU_threshold:
                label_assign = 1
            if IoU_value > IoU_max:
                IoU_max = IoU_value
                IoU_max_ind[0] = ind_row
                IoU_max_ind[1] = ind_col

            # construct the gt vector
            if label_assign == 1:
                label_gt[0, ind_row, ind_col] = 1
                label_gt[1, ind_row, ind_col] = label[0] - (grid_info[0] + anchor_size[1]/2)
                label_gt[2, ind_row, ind_col] = label[1] - (grid_info[1] + anchor_size[0]/2)
                label_gt[3, ind_row, ind_col] = label[2] / float(input_dim[1])
                label_gt[4, ind_row, ind_col] = label[3] / float(input_dim[0])
    
    grid_info = grid_cell(IoU_max_ind[0], IoU_max_ind[1])
    label_gt[0, IoU_max_ind[0], IoU_max_ind[1]] = 1
    label_gt[1, IoU_max_ind[0], IoU_max_ind[1]] = label[0] - (grid_info[0] + anchor_size[1]/2)
    label_gt[2, IoU_max_ind[0], IoU_max_ind[1]] = label[1] - (grid_info[1] + anchor_size[0]/2)
    label_gt[3, IoU_max_ind[0], IoU_max_ind[1]] = label[2] / float(input_dim[1])
    label_gt[4, IoU_max_ind[0], IoU_max_ind[1]] = label[3] / float(input_dim[0])
    return label_gt

def detect(onnx_file_path='f110_model.onnx', trt_file_path="", input_img_path = "", input_img=None, cv2window=""):
    # Download a dog image and save it to the following file path:
    # Two-dimensional tuple with the target network's (spatial) input resolution in HW ordered
    input_resolution = (180, 320)
    # Create a pre-processor object by specifying the required input resolution for YOLOv3
    preprocessor = PreprocessYOLO(input_resolution)
    # Load an image from the specified input path, and return it together with  a pre-processed version

    image_raw, image = preprocessor.process(input_img_path, input_img)
    # Store the shape of the original input image in WH format, we will need it for later
    shape_orig_WH = image_raw.size

    output_shapes = [(1, 5, 5, 10)]
    
    engine = build_engine(onnx_file_path, trt_file_path)
    context = engine.create_execution_context()
    # save_engine(engine, trt_file_path)

    inputs, outputs, bindings, stream = common.allocate_buffers(engine)
    print("Running inference on image {}".format(input_img_path))
    inputs[0].host = image
    trt_outputs = common.do_inference_v2(context, bindings, inputs, outputs, stream)

    trt_outputs = [output.reshape(shape) for output, shape in zip(trt_outputs, output_shapes)]

    bboxs, result_prob = postprocess(trt_outputs[0][0], 0.9)
    vote_rank = voting_suppression(bboxs, 0.5)
    if vote_rank.shape[0] == 0:
        print("ERROR: Found nothing")
        return -1, -1

    bbox = bboxs[vote_rank[0]]
    [c_x, c_y, w, h] = bbox_convert_r(bbox[0], bbox[1], bbox[2], bbox[3])
    bboxs_2 = np.array([[c_x, c_y, w, h]])

    ratio = 960.0 / 320

    c_x *= ratio
    c_y *= ratio

    print("Center Point: ({}, {})".format(c_x, c_y))
    img = np.transpose(image[0], (1, 2, 0))
    image_new = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)
    edgecolor = [1, 0, 0]
    if len(bboxs_2) == 1:
        cv2.rectangle(image_new, (int(bbox[0]), int(bbox[1])), (int(bbox[2]), int(bbox[3])), edgecolor, 4)
    elif len(bboxs_2) > 1:
        for i in range(len(bboxs_2)):
            cv2.rectangle(image_new, (bbox[0][i], bbox[1][i]), (bbox[2][i], bbox[3][i]), edgecolor, 4)
    cv2.imshow(cv2window, image_new)
    # cv2.waitKey(0)

    # DisplayLabel(np.transpose(image[0], (1, 2, 0)), bboxs_2)

    # boxes, classes, scores = postprocess(trt_outputs)
    # obj_detected_img = draw_bboxes(image, boxes=boxes, scores=scores, classes=classes

    return c_x, c_y


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Sample of YOLOv3 TensorRT.')
    parser.add_argument('--width', type=int, default=608, help='image width')
    parser.add_argument('--height', type=int, default=608, help='image height')
    parser.add_argument('--batch_size', type=int, default=1, help='image height')
    parser.add_argument('--dataset', type=str, default='coco_labels.txt', help='dataset classes names label')
    parser.add_argument('--int8', action='store_true', help='set int8 mode')
    parser.add_argument('--calib_file', type=str, default='yolo_calibration.cache', help='int8 calibration file')
    parser.add_argument('--onnx_file', type=str, default='lab8model.onnx', help='yolo onnx file')
    parser.add_argument('--engine_file', type=str, default='lab8model.trt', help='yolo tensorrt file')
    parser.add_argument('--image_file', type=str, default='./resource/1.jpg', help='image file')
    parser.add_argument('--result_file', type=str, default='output_bboxes.png', help='result file')
    args = parser.parse_args()

    detect(onnx_file_path=args.onnx_file, trt_file_path=args.engine_file, input_img_path=args.image_file, cv2window="test")
