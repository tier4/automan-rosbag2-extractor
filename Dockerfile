FROM ros:galactic

SHELL ["/bin/bash", "-l", "-c"]

RUN sed -i -e "1i . /opt/ros/galactic/setup.bash" ~/.bashrc
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
   wget \
   ros-galactic-console-bridge-vendor \
   ros-galactic-cv-bridge \
   libsm6 libxrender1 libxext-dev \
   python3-pip \
 && apt-get update && apt-get upgrade -y \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/*

RUN pip3 install "pybind11[global]"

ENV WORKDIR /app/
WORKDIR ${WORKDIR}

COPY requirements.txt ${WORKDIR}
RUN pip3 install -r requirements.txt

COPY . ${WORKDIR}

RUN ln -s /usr/bin/python3 /usr/bin/python
RUN echo $LD_LIBRARY_PATH > ${WORKDIR}/ld_path

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/app/bin/docker-entrypoint.bash"]
CMD ["python libs/rosbag_extractor.py --help"]
