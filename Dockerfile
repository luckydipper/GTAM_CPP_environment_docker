FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

ENV DIRPATH /root/
WORKDIR $DIRPATH

RUN apt-get update && apt-get install -y --no-install-recommends apt-utils
RUN apt-get install -y \
    build-essential \
    vim \
    cmake \
    git \
    libeigen3-dev \ 
    libboost-all-dev \
    libtbb-dev 

RUN git clone https://github.com/borglab/gtsam.git &&\
    cd gtsam &&\
    git fetch origin --tags 4.2 &&\
    git checkout 4.2 &&\
    mkdir build &&\
    cd build &&\
    cmake -DGTSAM_USE_SYSTEM_EIGEN=ON .. &&\
    make . -j8 &&\
    make install 

RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc

#CMD export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
CMD ["/bin/bash", "--rcfile", "/root/.bashrc"]