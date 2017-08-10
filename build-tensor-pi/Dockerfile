FROM fjctp/armhf-ros-kinetic-base:latest

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        curl \
        libfreetype6-dev \
        libpng12-dev \
        libzmq3-dev \
        pkg-config \
        python \
        python-dev \
        python-numpy \
        python-scipy \
        python-pil \
        python-lxml \
        rsync \
        unzip \
        protobuf-compiler \
        ros-kinetic-cv-camera \
        ros-kinetic-cv-bridge \
        ros-kinetic-web-video-server

RUN  apt-get clean && \
        rm -rf /var/lib/apt/lists/*

RUN curl -O https://bootstrap.pypa.io/get-pip.py && \
    python get-pip.py && \
        rm get-pip.py

RUN pip --no-cache-dir install \
        ipykernel \
        jupyter \
        matplotlib && \
            python -m ipykernel.kernelspec

ADD tensorflow-1.1.0-cp27-none-linux_armv7l.whl .

RUN pip install tensorflow-1.1.0-cp27-none-linux_armv7l.whl

COPY models /tensorflow_models
RUN /bin/bash -c "cd /tensorflow_models && \
                  protoc object_detection/protos/*.proto --python_out=."

COPY jupyter_notebook_config.py /root/.jupyter/

# Copy sample notebooks.
COPY notebooks /notebooks

# Jupyter has issues with being run directly:
#   https://github.com/ipython/ipython/issues/7062
# We just add a little wrapper script.
COPY run_jupyter.sh /

ADD object_detection_ROS.py /
COPY run_detection.sh /
COPY mobilenet /mobilenet

# TensorBoard
EXPOSE 6006
# IPython
EXPOSE 8888

ENV PYTHONPATH /tensorflow_models:/tensorflow_models/slim:$PYTHONPATH

WORKDIR "/"

CMD ["/run_detection.sh"]
