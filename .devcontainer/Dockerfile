FROM nvidia/cuda:11.3.0-devel-ubuntu20.04

RUN --mount=type=cache,target=/var/cache/apt apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        git \
        python3.9 \
        python3.9-dev \
        python3.9-distutils \
        sudo \
        curl \
        libcudnn8

RUN ln -s /usr/bin/python3.9 /usr/bin/python

COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache \
    curl https://bootstrap.pypa.io/get-pip.py -o /tmp/get-pip.py && \
    python /tmp/get-pip.py && \
    pip install -r /tmp/requirements.txt

RUN useradd -m -u 1000 -s /bin/bash vscode
RUN echo "vscode    ALL=(ALL:ALL) NOPASSWD: ALL" >> /etc/sudoers.d/vscode

ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/cuda-11.3/targets/x86_64-linux/lib
ENV PATH=${PATH}:/home/vscode/.local/bin