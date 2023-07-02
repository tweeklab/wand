from collections import deque
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
from time import time, sleep
import collections
import itertools
import tensorflow as tf
from typing import NamedTuple
import os
import uuid

reverse_label_map = {
    0: "arresto-momentum",
    1: "descendo",
    2: "incendio",
    3: "mimblewimble",
    4: "locomotor",
    5: "tarantallegra",
    6: "gonadium",
}


def create_pass1_detector():
    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 100
    params.maxThreshold = 255
    params.thresholdStep = 50
    params.minDistBetweenBlobs = 3
    params.minRepeatability = 3
    params.filterByColor = True
    params.blobColor = 255
    params.filterByArea = True
    params.minArea = 5
    params.filterByCircularity = False
    params.minCircularity = 0.3
    params.filterByConvexity = False
    params.minConvexity = 0.87
    params.filterByInertia = False
    params.minInertiaRatio = 0.01
    return cv2.SimpleBlobDetector_create(params)


class ModelInto(NamedTuple):
    input: list
    output: list
    interpreter: tf.lite.Interpreter


def load_model():
    interpreter = tf.lite.Interpreter("model.tflite")
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    interpreter.allocate_tensors()

    return ModelInto(
        input=input_details, output=output_details, interpreter=interpreter
    )


def setup_camera():
    w = 400
    h = 400
    camera = PiCamera(sensor_mode=4, framerate=30, resolution=(w, h))
    rawCapture = PiRGBArray(camera, size=(w, h))
    # allow the camera to warmup
    sleep(1)

    return (camera, rawCapture)


def displacement(v0, v1):
    angle = np.arctan2(np.linalg.det([v0, v1]), np.dot(v0, v1))
    disp = 180 - np.abs(np.degrees(angle))
    return disp


def render(raw_data_grouped):
    global clear_reason
    global dwell_timer
    global loser_bin_evictions_counter
    global loser_bin_stats_counter
    global detect_queue_pass1
    ##### PASTE
    min_frame = raw_data_grouped[0][0][2]
    filtered_frames = []
    frame_q = collections.deque(maxlen=5)
    bin_size = 20

    def append_frame(f):
        if len(filtered_frames) > 0 and filtered_frames[-1][0] == f[0]:
            return
        filtered_frames.append(f)

    def is_loser(pt):
        global loser_bin_evictions_counter
        ret = (int(pt[0][0] / bin_size), int(pt[0][1] / bin_size)) in loser_bin
        if ret:
            loser_bin_evictions_counter += 1
        return ret

    def add_loser(pt):
        loser_bin[(int(pt[0][0] / bin_size), int(pt[0][1] / bin_size))] += 1
        loser_timestamps[(int(pt[0][0] / bin_size), int(pt[0][1] / bin_size))] = time()

    def trim_losers():
        to_remove = []
        for loser, timestamp in loser_timestamps.items():
            if (time() - timestamp) > 30:
                to_remove.append(loser)
        for winner in to_remove:
            del loser_bin[winner]
            del loser_timestamps[winner]

    for frame in raw_data_grouped:
        formatted_frame = [[(pt[0], pt[1]), pt[2] - min_frame] for pt in frame]
        frame_q.append(formatted_frame)
        if len(frame_q) < 3:
            # Don't do anything until we have enough points to vote
            continue

        if len(frame_q[0]) == 1:
            # If the frame we are processing only has one hit
            # pass it through assuming it's the wand
            # There is safely only one point here so we can just index it
            # directly
            # XXX noticed while porting...Is loser filtering already handled by filtering the raw keypoints?
            if not is_loser(frame_q[0][0]):
                append_frame(frame_q[0][0])
            continue

        # Choose the point that results in the lowest angular displacement
        # from the previous point
        # Right now we can't tolerate starting with a frame that has multiple
        # blob hits.  Skip until we get something that is not ambiguous.
        if len(filtered_frames) == 0 and len(frame_q[0]) > 1:
            # print("Ambiguous start frame")
            for candidate in frame_q[0]:
                # XXX noticed while porting...Is loser filtering already handled by filtering the raw keypoints?
                if not is_loser(candidate):
                    append_frame(candidate)
            continue

        # The grid is made up the of the array indices for each frame so we can
        # easily locate the points in the queue that we need later on.
        # This will always reference filtered_frames[-1] as the previous frame
        # to create the first angle...
        grid = [
            x
            for x in itertools.product(
                [i for i in range(len(frame_q[0])) if not is_loser(frame_q[0][i])],
                [i for i in range(len(frame_q[1])) if not is_loser(frame_q[1][i])],
                [i for i in range(len(frame_q[2])) if not is_loser(frame_q[2][i])],
            )
        ]
        min_grid_idx = None
        min_value = None
        ref_frame = filtered_frames[-1]
        debug_out = []
        for grid_idx, candidate in enumerate(grid):
            all_disp = []
            for i in range(len(candidate) - 1):
                if i == 0:
                    v1 = np.array(ref_frame[0]) - np.array(frame_q[i][candidate[i]][0])  # type: ignore
                else:
                    v1 = np.array(frame_q[i - 1][candidate[i - 1]][0]) - np.array(frame_q[i][candidate[i]][0])  # type: ignore
                v0 = np.array(frame_q[i + 1][candidate[i + 1]][0]) - np.array(frame_q[i][candidate[i]][0])  # type: ignore
                this_disp = displacement(v0, v1) * np.linalg.norm(v1)
                all_disp.append(this_disp)

            total_disp = sum(all_disp)
            debug_out.append(
                f"{[ref_frame[0]] + [frame_q[i][candidate[i]][0] for i in range(len(candidate))]} = {total_disp}"
            )
            if not min_value or total_disp < min_value:
                min_value = total_disp
                min_grid_idx = grid_idx

        if min_grid_idx is not None:
            new_point = frame_q[0][grid[min_grid_idx][0]]
            for idx, pt in enumerate(frame_q[0]):
                if idx != grid[min_grid_idx][0]:
                    add_loser(pt)
            append_frame(new_point)

    loser_bin_stats_counter += 1
    if (loser_bin_stats_counter % 30) == 0:
        print(f"loser bin: {loser_bin_evictions_counter}")
    trim_losers()
    ##### PASTE END

    if len(filtered_frames) >= 10:
        # Test for motion
        strides = np.array(
            [i for i in zip(filtered_frames[-10:], filtered_frames[-9:])]
        )
        vectors = []
        for stride in strides:
            vectors.append(np.array(stride[1][0]) - np.array(stride[0][0]))
        resultant_vector = sum(vectors)
        total_dist = np.linalg.norm(resultant_vector)
        if total_dist > 10:
            dwell_timer = 0
            return

        xmax = max([frame[0][0] for frame in filtered_frames])
        ymax = max([frame[0][1] for frame in filtered_frames])
        xmin = min([frame[0][0] for frame in filtered_frames])
        ymin = min([frame[0][1] for frame in filtered_frames])

        dominant_dimension = np.max([xmax - xmin, ymax - ymin])
        dominant_dimension += 40 - (dominant_dimension % 40)

        # Frames smaller than this are unlikely to have anything
        # interesting so don't process them
        if dominant_dimension <= 80:
            if dwell_timer < 2:
                dwell_timer += 1
                if dwell_timer == 2:
                    # Only allow this once and then not again
                    # until we detect more movement
                    clear_reason = "dwell"
                    detect_queue_pass1.clear()
            return

        xpad = int((dominant_dimension - (xmax - xmin)) / 2)
        ypad = int((dominant_dimension - (ymax - ymin)) / 2)

        processed_output_image = np.zeros(
            (dominant_dimension, dominant_dimension), dtype=np.uint8
        )

        prev_x = None
        prev_y = None
        for frame in filtered_frames:
            if prev_x is None or prev_y is None:
                prev_x = frame[0][0] - xmin + xpad
                prev_y = frame[0][1] - ymin + ypad
                continue
            start = (prev_x, prev_y)
            end = (frame[0][0] - xmin + xpad, frame[0][1] - ymin + ypad)
            cv2.line(
                processed_output_image, start, end, color=(255, 255, 255), thickness=2
            )
            prev_x = end[0]
            prev_y = end[1]

        try:
            disp_process_image = cv2.resize(
                processed_output_image,
                (40, 40),
                # interpolation = cv2.INTER_CUBIC
                interpolation=cv2.INTER_AREA,
            )
        except Exception as e:
            print(f"RESIZE FAIL")
            return
        cv2.imshow("Processed", processed_output_image)
        disp_process_image[disp_process_image > 0] = 255
        formatted_img_array = np.expand_dims(
            disp_process_image.reshape(40, 40, 1), axis=0
        )
        model_info.interpreter.set_tensor(
            model_info.input[0]["index"], formatted_img_array
        )
        model_info.interpreter.invoke()
        output_data = model_info.interpreter.get_tensor(model_info.output[0]["index"])[
            0
        ]
        pred_max = np.max(output_data)
        pred_argmax = np.argmax(output_data)
        print(f"pred: {pred_argmax} ({reverse_label_map[pred_argmax]}): {pred_max}")

        label_image = np.zeros((200, 400), dtype=np.uint8)
        cv2.putText(
            label_image,
            f"{reverse_label_map[pred_argmax]} ({pred_max/255:.02f})",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )
        cv2.imshow("Label", label_image)

        outfiledir = os.path.join("captures", reverse_label_map[pred_argmax])
        outfilename = os.path.join(outfiledir, f"{uuid.uuid4()}.png")
        os.makedirs(outfiledir, exist_ok=True)
        cv2.imwrite(filename=outfilename, img=disp_process_image)

        clear_reason = "inference"
        detect_queue_pass1.clear()


def is_global_loser(kp):
    bin_size = 20
    ret = (int(kp.pt[0] / bin_size), int(kp.pt[1] / bin_size)) in loser_bin
    return ret


detector_pass1 = create_pass1_detector()
detect_queue_pass1 = deque(maxlen=100)
global_losers = None
model_info = load_model()
camera, rawCapture = setup_camera()
i = 0
frame_count = 0
loser_bin = collections.Counter()
loser_timestamps = {}
loser_bin_evictions_counter = 0
loser_bin_stats_counter = 0
clear_reason = ""
dwell_timer = 0
miss_counter = 0


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image_pass1 = cv2.cvtColor(cv2.flip(frame.array, 1), cv2.COLOR_BGR2GRAY)
    raw_points_image = np.zeros((400, 400, 3), dtype=np.uint8)
    keypoints = detector_pass1.detect(image_pass1)
    try:
        formatted_new_point = []
        if keypoints:
            formatted_new_point = [
                (int(kp.pt[0]), int(kp.pt[1]), frame_count)
                for kp in keypoints
                if not is_global_loser(kp)
            ]
            if formatted_new_point:
                detect_queue_pass1.append(formatted_new_point)
                miss_counter = 0
        if not keypoints or not formatted_new_point:
            if miss_counter < 30:
                miss_counter += 1
            if miss_counter == 30:
                if len(detect_queue_pass1):
                    clear_reason = "idle"
                detect_queue_pass1.clear()
    except IndexError:
        pass
    frame_count += 1

    for pt in [(p[0], p[1]) for frame in detect_queue_pass1 for p in frame]:
        cv2.circle(
            raw_points_image,
            (pt[0], pt[1]),
            radius=4,
            color=(255, 255, 255),
            thickness=-1,
        )
    cv2.putText(
        raw_points_image,
        f"{len(detect_queue_pass1)}",
        (50, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (255, 255, 255),
        2,
    )
    cv2.putText(
        raw_points_image,
        f"{clear_reason}",
        (200, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1,
        (0, 0, 255),
        2,
    )
    cv2.imshow("Raw Points", raw_points_image)

    i += 1
    if i == 10:
        if len(detect_queue_pass1) > 0:
            render(detect_queue_pass1)
        i = 0

    cv2.waitKey(1)
    rawCapture.truncate(0)
