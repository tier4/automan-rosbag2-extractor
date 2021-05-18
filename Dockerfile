FROM ros:foxy


SHELL ["/bin/bash", "-l", "-c"]

RUN sed -i -e "1i . /opt/ros/foxy/setup.bash" ~/.bashrc
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
   wget \
   ros-foxy-console-bridge-vendor \
   ros-foxy-cv-bridge \
   libsm6 libxrender1 libxext-dev \
   python3-pip \
 && apt-get update && apt-get upgrade -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install "pybind11[global]"

RUN cd /tmp && git clone -b foxy https://github.com/ito-san/rosbag2.git
RUN cd /tmp/rosbag2 && git checkout feature/backport-latest-to-foxy
RUN cd /tmp/rosbag2 && colcon build --merge-install
RUN cd /tmp/rosbag2/rosbag2_py && colcon build


ENV WORKDIR /app/
WORKDIR ${WORKDIR}

COPY requirements.txt ${WORKDIR}
RUN pip3 install -r requirements.txt

COPY . ${WORKDIR}

RUN ln -s /usr/bin/python3 /usr/bin/python
RUN echo $LD_LIBRARY_PATH > ${WORKDIR}/ld_path
RUN cp -r /tmp/rosbag2/install/* /opt/ros/foxy/ && chmod 755 /opt/ros/foxy/setup.bash

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/app/bin/docker-entrypoint.bash"]
CMD ["python libs/rosbag_extractor.py --help"]
