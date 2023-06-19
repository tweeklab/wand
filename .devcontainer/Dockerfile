FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

RUN --mount=type=cache,target=/var/cache/apt apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        git \
        python3.10 \
        python3.10-dev \
        python3.10-distutils \
        python3-tk \
        sudo \
        curl

RUN ln -s /usr/bin/python3.10 /usr/bin/python

COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache \
    curl https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py && \
    python /tmp/get-pip.py && \
    pip install -r /tmp/requirements.txt

RUN useradd -m -u 1000 -s /bin/bash vscode
RUN echo "vscode    ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers.d/vscode

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-12.1/targets/x86_64-linux/lib
ENV PATH=${PATH}:/home/vscode/.local/bin