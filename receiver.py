# SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
# SPDX-License-Identifier: Unlicense OR CC0-1.0
import socketserver
import cv2
import cv2 as cv
import numpy as np
from time import monotonic
from sklearn.cluster import KMeans
import json
import collections

# frame_len = 144*176
frame_len = 240*240

def create_pass1_detector():
    params = cv2.SimpleBlobDetector_Params()
    # params.minThreshold = 100
    # params.maxThreshold = 255
    # params.thresholdStep = 50
    # params.minDistBetweenBlobs = 3
    # params.minRepeatability = 3
    # params.filterByColor = True
    # params.blobColor = 255
    # params.filterByArea = True
    # params.minArea = 5
    # params.filterByCircularity = False
    # params.minCircularity = 0.3
    # params.filterByConvexity = False
    # params.minConvexity = 0.87
    # params.filterByInertia = False
    # params.minInertiaRatio = 0.01

# Normal video
    # params.minThreshold = 70;
    # params.maxThreshold = 71;
    # params.thresholdStep = 1;
    # params.minDistBetweenBlobs = 3;
    # params.minRepeatability = 1;
    # params.filterByColor = True;
    # params.blobColor = 255;
    # params.filterByArea = False;
    # params.minArea = 2;
    # params.filterByCircularity = False;
    # params.minCircularity = 0.3;
    # params.filterByConvexity = False;
    # params.minConvexity = 0.87;
    # params.filterByInertia = False;
    # params.minInertiaRatio = 0.01;

# Inverted video
    params.minThreshold = 70;
    params.maxThreshold = 71;
    params.thresholdStep = 1;
    params.minDistBetweenBlobs = 3;
    params.minRepeatability = 1;
    params.filterByColor = True;
    params.blobColor = 0;
    params.filterByArea = False;
    params.minArea = 2;
    params.filterByCircularity = False;
    params.minCircularity = 0.3;
    params.filterByConvexity = False;
    params.minConvexity = 0.87;
    params.filterByInertia = False;
    params.minInertiaRatio = 0.01;

    return cv2.SimpleBlobDetector_create(params)

class MyTCPHandler(socketserver.BaseRequestHandler):
    """
    The request handler class for our server.

    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """

    def handle(self):
        print("Got one")
        self.recv_this_frame = 0
        self.data = b''
        i = 0

        detector_pass1 = create_pass1_detector()
        frames = collections.deque(maxlen=50)

        recv = ""
        while True:
            # self.request is the TCP socket connected to the client
            recv += self.request.recv(1).decode('utf-8')
            if recv.endswith('\n'):
                d = json.loads(recv)
                recv = ""
                # output = np.zeros((480,640,3))
                output = np.zeros((296,400,3))
                point_count = len(d[1:])
                frames.append(d[1:])
                start = None
                for point in d[1:]:
                    # cv2.rectangle(output, (point[0], point[1]), (point[2], point[3]), (0, 255, 0), 1, cv2.LINE_AA)
                    # cv2.circle(output, (point[0], point[1]), 2, (0, 255, 0))
                    if start is not None:
                        cv2.line(
                            output, start, (point[0], point[1]), color=(255, 255, 255), thickness=2
                        )
                    start = (point[0], point[1])
                cv2.putText(output, str(d[0]), (10,30), cv2.FONT_HERSHEY_SIMPLEX, 
                   .4, (0, 255, 0), 1, cv2.LINE_AA)
                cv2.putText(output, str(point_count), (10,50), cv2.FONT_HERSHEY_SIMPLEX, 
                   .4, (0, 0, 255), 1, cv2.LINE_AA)
                cv2.imshow("Test", output)
                k = cv2.waitKey(20)
                if (k & 0xFF) == ord('d'):
                    with open("./frames_dump.json", "w") as df:
                        print("Dumping.")
                        json.dump(list(frames), df)
            # while self.recv_this_frame < frame_len:
            #     want_recv = frame_len - self.recv_this_frame
            #     if want_recv > 4096:
            #         d = self.request.recv(4096)
            #     else:
            #         d = self.request.recv(want_recv)
            #     self.data += d
            #     self.recv_this_frame += len(d)
            # # frame = np.frombuffer(self.data, dtype=np.uint8).reshape(144, 176)
            # frame = np.frombuffer(self.data, dtype=np.uint8).reshape(240, 240)
            # # keypoints = detector_pass1.detect(frame)
            # # if (i%20) == 0:
            # #     print([(kp.pt[0], kp.pt[1], kp.size) for kp in keypoints])
            # # i+=1
            # # ret,thresh_frame = cv2.threshold(frame,70,255,cv2.THRESH_BINARY)
            # # output_image = cv2.drawKeypoints(thresh_frame, keypoints, 0, (0, 255, 0),
            # #    flags=cv2.DRAW_MATCHES_FLAGS_NOT_DRAW_SINGLE_POINTS)
            # ret,binary = cv2.threshold(frame,70,255,cv2.THRESH_BINARY)
            # output = np.zeros((240,240,3))
            # # print(monotonic())
            # ret = np.argwhere(binary == 0)
            # if len(ret) == 0:
            #     ret = []
            # else:
            #     criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 10, 1.0)
            #     flags = cv.KMEANS_PP_CENTERS
            #     compactness,labels,centers = cv.kmeans(ret.astype(np.float32),min(max(len(ret), 1), 5),None,criteria,10,flags)
            #     ret = centers
            #     # print(ret)
            #     # model = KMeans(min(max(len(ret), 1), 5), n_init='auto', random_state=3).fit(ret)
            #     # ret = model.cluster_centers_
            # # print(len(ret))
            # # print(monotonic())
            # # print("----")
            # for point in ret:
            #     cv2.circle(frame, (int(point[1]), int(point[0])), 2, (0, 255, 0))
            # cv2.imshow("Test", frame)
            # cv2.waitKey(1)
            # cv2.waitKey(1)
            # cv2.waitKey(1)
            # self.recv_this_frame = 0
            # self.data = b''

if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 3333

    # Create the server, binding to localhost on port 9999
    with socketserver.TCPServer((HOST, PORT), MyTCPHandler) as server:
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()
