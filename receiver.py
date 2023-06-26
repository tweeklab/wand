import socketserver
import os
import cv2
import numpy as np
import json
import yaml
from uuid import uuid4
from enum import Enum
from PIL import Image
import socket


class FilterState(Enum):
    IDLE = 0
    ACTIVE = 1
    DWELL = 2
    COMMIT = 3


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
        self.data = b""

        winners = []
        losers = []
        rects = []
        filter_state = 0
        frame_no = -1
        v = -1

        with open("labels.yaml") as f:
            label_map = yaml.load(f, Loader=yaml.SafeLoader)['labels']
        reverse_label_map = {v: k for k, v in label_map.items()}

        for label in label_map:
            p = f"images/esp32/live/{label}"
            os.makedirs(p, exist_ok=True)

        recv = ""
        pred = "None"
        pred_score = "NA"
        cv2.imshow("Final", np.zeros((29, 40)))
        while True:
            # self.request is the TCP socket connected to the client
            try:
                recv += self.request.recv(1, socket.MSG_DONTWAIT).decode("utf-8")
            except BlockingIOError:
                pass
            except UnicodeDecodeError:
                pass
            except:
                print(recv)
                raise
            if "\n" in recv:
                try:
                    end = recv.index("\n")
                except ValueError:
                    end = None
                try:
                    d = json.loads(recv[:end])
                except json.decoder.JSONDecodeError as e:
                    d = None
                    print(recv)
                    print(e)
                if end is not None:
                    recv = recv[end + 1 :]
                else:
                    recv = ""
                output = np.zeros((296, 400, 3))
                if d is None:
                    continue
                if d[0] == "STAT":
                    filter_state = d[1]
                    frame_no = d[2]
                    v = d[3]
                if d[0] == "COMMIT_IMG":
                    raw = bytes()
                    pred = reverse_label_map[d[2]]
                    pred_score = d[3]
                    filename = f"images/esp32/live/{pred}/{uuid4()}.png"
                    print(f"Receiving {d[1]} bytes")
                    while len(raw) < d[1]:
                        raw += self.request.recv(d[1] - len(raw))
                    final_image = np.array(list(raw), dtype=np.uint8).reshape(29, 40)
                    img = Image.fromarray(final_image)
                    img.save(filename, "png")
                    print(f"Got image! {final_image.shape} ({pred})")
                    cv2.imshow("Final", final_image)
                if d[0] == "WINNERS":
                    winners = d[1]
                if d[0] == "LOSERS":
                    losers = d[1]
                if d[0] == "RECTS":
                    rects = d[1]

                cv2.putText(
                    output,
                    str(frame_no),
                    (5, 20),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    output,
                    str(FilterState(filter_state).name),
                    (5, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    output,
                    str(v),
                    (5, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    output,
                    f"{pred} ({pred_score})",
                    (5, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 255),
                    1,
                    cv2.LINE_AA,
                )

                for point in winners:
                    cv2.circle(output, (point[0], point[1]), 1, (0, 255, 0))
                for point in losers:
                    # cv2.circle(output, (point[0] * 20, point[1] * 20), 1, (0, 0, 255))
                    cv2.putText(
                        output,
                        str(point[2]),
                        (point[0] * 20, point[1] * 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        (0, 0, 255),
                        1,
                        cv2.LINE_AA,
                    )

                for rect in rects:
                    tl = rect[0]
                    br = rect[1]
                    cv2.rectangle(output, tl, br, (0, 255, 255), 1, cv2.LINE_AA)

                cv2.imshow("Test", output)
                cv2.waitKey(1)


if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 3333

    # Create the server, binding to localhost on port 9999
    with socketserver.TCPServer((HOST, PORT), MyTCPHandler) as server:
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()
