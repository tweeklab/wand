import socketserver
import cv2
import numpy as np
import json
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

        frame_points = []
        winners = []
        losers = []
        point_count = -1
        filter_state = 0
        frame_no = -1
        v = -1

        recv = ""
        cv2.imshow("Final", np.zeros((29, 40)))
        while True:
            # self.request is the TCP socket connected to the client
            try:
                recv += self.request.recv(100, socket.MSG_DONTWAIT).decode("utf-8")
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
                if d[0] == "COMMIT":
                    frame_points = d[1]
                if d[0] == "COMMIT_IMG":
                    filename = f"images/esp32/live/{uuid4()}.png"
                    raw = bytes()
                    print(f"Receiving {d[1]} bytes")
                    while len(raw) < d[1]:
                        raw += self.request.recv(d[1] - len(raw))
                    final_image = np.array(list(raw), dtype=np.uint8).reshape(29, 40)
                    img = Image.fromarray(final_image)
                    img.save(filename, "png")
                    print(f"Got image! {final_image.shape}")
                    cv2.imshow("Final", final_image)
                if d[0] == "WINNERS":
                    winners = d[1]
                if d[0] == "LOSERS":
                    losers = d[1]

                point_count = len(frame_points)
                start = None
                cv2.rectangle(output, (0, 0), (42, 31), (0, 150, 150), 1, cv2.LINE_AA)
                for point in frame_points:
                    if start is not None:
                        cv2.line(
                            output,
                            start,
                            (point[0] + 1, point[1] + 1),
                            color=(255, 255, 255),
                            thickness=1,
                        )
                    start = (point[0] + 1, point[1] + 1)
                cv2.putText(
                    output,
                    str(frame_no),
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 255, 0),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    output,
                    str(point_count),
                    (10, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    output,
                    str(FilterState(filter_state).name),
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
                cv2.putText(
                    output,
                    str(v),
                    (10, 110),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4,
                    (255, 255, 0),
                    1,
                    cv2.LINE_AA,
                )

                for point in winners:
                    cv2.circle(output, (point[0], point[1]), 2, (0, 255, 0))
                for point in losers:
                    cv2.circle(output, (point[0] * 20, point[1] * 20), 2, (0, 0, 255))

                cv2.imshow("Test", output)
                cv2.waitKey(1)


if __name__ == "__main__":
    HOST, PORT = "0.0.0.0", 3333

    # Create the server, binding to localhost on port 9999
    with socketserver.TCPServer((HOST, PORT), MyTCPHandler) as server:
        # Activate the server; this will keep running until you
        # interrupt the program with Ctrl-C
        server.serve_forever()
