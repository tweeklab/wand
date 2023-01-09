from collections import deque
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import cv2
from time import time, sleep
import collections
import itertools
from concurrent.futures import ThreadPoolExecutor
import colorsys

# initialize the camera and grab a reference to the raw camera capture
w = 400
h = 400
camera = PiCamera(sensor_mode=4, framerate=30, resolution=(w, h))


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


detector_pass1 = create_pass1_detector()
detect_queue_pass1 = deque(maxlen=100)
detect_queue_pass2 = deque(maxlen=50)

rawCapture = PiRGBArray(camera, size=(w, h))
# allow the camera to warmup
sleep(1)

dumpf = open("points.txt", "w")

i = 0
frame_count = 0

def displacement(v0, v1):
    angle = np.arctan2(np.linalg.det([v0, v1]), np.dot(v0, v1))
    disp = 180 - np.abs(np.degrees(angle))
    return disp

loser_bin = collections.Counter()
loser_timestamps = {}
loser_bin_evictions_counter = 0
loser_bin_stats_counter = 0
counter = 0

def render(raw_data_grouped):
    raw_output_image = np.zeros((400, 450, 3), dtype=np.uint8)
    processed_output_image = np.zeros((400, 400), dtype=np.uint8)

    ##### PASTE
    min_frame = raw_data_grouped[0][0][2]
    filtered_frames = []
    frame_q = collections.deque(maxlen=5)
    bin_size = 20
    x = []
    y = []

    start = time()

    def append_frame(f):
        if len(filtered_frames) > 0 and filtered_frames[-1][0] == f[0]:
            return
        filtered_frames.append(f)

    for frame in raw_data_grouped:
        formatted_frame = [
            [(pt[0], pt[1]), pt[2] - min_frame] for pt in frame
        ]
        frame_q.append(formatted_frame)
        if len(frame_q) < 3:
            # Don't do anything until we have enough points to vote
            continue

        def is_loser(pt):
            ret = (int(pt[0][0] / bin_size), int(pt[0][1] / bin_size)) in loser_bin
            if ret:
                global loser_bin_evictions_counter
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

        if len(frame_q[0]) == 1:
            # If the frame we are processing only has one hit
            # pass it through assuming it's the wand
            # There is safely only one point here so we can just index it
            # directly
            if not is_loser(frame_q[0][0]):
                append_frame(frame_q[0][0])
            continue

        # Choose the point that results in the lowest angular displacement
        # from the previous point
        # Right now we can't tolerate starting with a frame that has multiple
        # blob hits.  Skip until we get something that is not ambiguous.
        if len(filtered_frames) == 0 and len(frame_q[0]) > 1:
            # print("Ambiguous start frame")
            if not is_loser(frame_q[0][0]):
                append_frame(frame_q[0][0])
            continue

        # The grid is made up the of the array indices for each frame so we can
        # easily locate the points in the queue that we need later on.
        # This will always reference filtered_framts[-1] as the previous frame
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
            debug_out.append(f"{[ref_frame[0]] + [frame_q[i][candidate[i]][0] for i in range(len(candidate))]} = {total_disp}")
            if not min_value or total_disp < min_value:
                min_value = total_disp
                min_grid_idx = grid_idx

        if min_grid_idx is not None:
            new_point = frame_q[0][grid[min_grid_idx][0]]
            for idx, pt in enumerate(frame_q[0]):
                if idx != grid[min_grid_idx][0]:
                    add_loser(pt)
            append_frame(new_point)

    global loser_bin_stats_counter
    loser_bin_stats_counter += 1
    if (loser_bin_stats_counter % 30) == 0:
        print(f"loser bin: {loser_bin_evictions_counter}")
    trim_losers()
    print(f"{time()-start}")
    ##### PASTE END

    prev_x = None
    prev_y = None
    for frame in filtered_frames:
        if prev_x is None or prev_y is None:
            prev_x = frame[0][0]
            prev_y = frame[0][1]
            continue
        start = (prev_x, prev_y)
        end = (frame[0][0], frame[0][1])
        cv2.line(processed_output_image, start, end, color=(255, 255, 255), thickness=3)
        prev_x = end[0]
        prev_y = end[1]

    # cv2.putText(
    #     processed_output_image,
    #     f"{len(filtered_frames)}",
    #     (50, 50),
    #     cv2.FONT_HERSHEY_SIMPLEX,
    #     1,
    #     (255, 255, 255),
    #     2,
    # )
    cv2.imshow("Processed", processed_output_image)
    if len(filtered_frames) > 0:
        global counter
        cv2.imwrite(filename=f"captures/{counter}.png", img=processed_output_image)
        counter += 1

    prev_x = None
    prev_y = None
    for frame in [(p[0], p[1]) for frame in raw_data_grouped for p in frame]:
        if prev_x is None or prev_y is None:
            prev_x = frame[0]
            prev_y = frame[1]
            continue
        start = (prev_x, prev_y)
        end = (frame[0], frame[1])
        cv2.line(raw_output_image, start, end, color=(255, 255, 255), thickness=3)
        prev_x = end[0]
        prev_y = end[1]

    for i in range(255):
        color = np.array(colorsys.hsv_to_rgb(i / 255, 1, 1)) * 255
        color = color.astype(np.uint8).tolist()
        cv2.rectangle(
            raw_output_image,
            (445, 399 - ((2 * i) + 1)),
            (449, 399 - ((2 * i))),
            color=color,
            thickness=-1,
        )
        cv2.putText(
            raw_output_image,
            "255",
            (400, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            .5,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            raw_output_image,
            "1",
            (425, 390),
            cv2.FONT_HERSHEY_SIMPLEX,
            .5,
            (255, 255, 255),
            1,
        )

    for frame in [(p[0], p[1], v) for p, v in loser_bin.items()]:
        if frame[2] < 255:
            color = np.array(colorsys.hsv_to_rgb(frame[2] / 255, 1, 1)) * 255
        else:
            color = np.array(colorsys.hsv_to_rgb(1, 1, 1)) * 255
        color = color.astype(np.uint8).tolist()
        cv2.rectangle(
            raw_output_image,
            (frame[0] * bin_size, frame[1] * bin_size),
            (int((frame[0] * bin_size) + bin_size), int((frame[1] * bin_size) + bin_size)),
            color=color,
            thickness=-1,
        )

    cv2.imshow("Raw", raw_output_image)


executor = ThreadPoolExecutor(max_workers=1)

# capture frames from the camera
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    image_pass1 = cv2.cvtColor(cv2.flip(frame.array, 1), cv2.COLOR_BGR2GRAY)
    raw_points_image = np.zeros((400, 400), dtype=np.uint8)
    keypoints = detector_pass1.detect(image_pass1)
    try:
        if keypoints:
            detect_queue_pass1.append([(int(kp.pt[0]), int(kp.pt[1]), frame_count) for kp in keypoints])
        else:
            detect_queue_pass1.popleft()
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
    cv2.imshow("Raw Points", raw_points_image)

    i += 1
    if i == 15:
        if len(detect_queue_pass1) > 40:
            # executor.submit(render, detect_queue_pass1)
            render(detect_queue_pass1)
            # print("[")
            # for frame in detect_queue_pass1:
            #     print(f"{frame},")
            # print("]")
            # print(f"===== {time.ctime()} ======")
        i = 0

    # for pt in detect_queue_pass2:
    #     cv2.circle(
    #         final_image, (pt[0], pt[1]), radius=3, color=(255, 255, 255), thickness=-1
    #     )
    # cv2.imshow("Final Image", final_image)

    cv2.waitKey(1)
    rawCapture.truncate(0)

    # i += 1
    # if i == 30:
    #     print(
    #         f"{[(pt[0], pt[1], image[pt[1]][pt[0]]) for pt in detect_queue_pass1]}",
    #         file=dumpf
    #     )
    #     i = 0
#    pt0 = None
#    for pt in detect_queue:
#        if not pt0:
#            pt0 = pt
#            continue
#        cv2.line(image, pt0, pt, 200, 3)
#        pt0 = pt
