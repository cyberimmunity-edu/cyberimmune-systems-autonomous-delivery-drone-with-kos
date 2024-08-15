FROM cr.yandex/mirror/ubuntu:22.04

ENV DEBIAN_FRONTEND noninteractive
ENV PATH="${PATH}:/opt/KasperskyOS-Community-Edition-1.2.0.89/toolchain/bin:/home/user/.local/bin"
RUN apt-get update && \
    apt upgrade -y && \
    apt install -y \
        net-tools \
        python3 \
        python3-dev \
        python3-pip \
        python3-serial \
        python3-opencv \
        python3-wxgtk4.0 \
        python3-matplotlib \
        python3-lxml \
        python3-pygame \
        sudo \
        mc \
        vim \
        curl \
        tar \
        expect \
        build-essential \
        device-tree-compiler \
        parted \
        fdisk \
        dosfstools \
        && adduser --disabled-password --gecos "" user \
        && echo 'user ALL=(ALL) NOPASSWD: ALL' >> /etc/sudoers

COPY ./KasperskyOS-Community-Edition-1.2.0.89_en.deb /tmp
COPY ./KasperskyOS-Community-Edition-1.2.0.45.zip /tmp

RUN apt install /tmp/KasperskyOS-Community-Edition-1.2.0.89_en.deb -y
RUN rm -f /opt/KasperskyOS-Community-Edition-1.2.0.89/sysroot-aarch64-kos/bin/dnet_entity \
    && unzip -j /tmp/KasperskyOS-Community-Edition-1.2.0.45.zip KasperskyOS-Community-Edition-1.2.0.45/sysroot-aarch64-kos/bin/dnet_entity -d /opt/KasperskyOS-Community-Edition-1.2.0.89/sysroot-aarch64-kos/bin
RUN rm /tmp/KasperskyOS-Community-Edition-1.2.0.89_en.deb \
    && rm /tmp/KasperskyOS-Community-Edition-1.2.0.45.zip \
    && echo '/opt/KasperskyOS-Community-Edition-1.2.0.89/toolchain/lib' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && echo '/opt/KasperskyOS-Community-Edition-1.2.0.89/toolchain/x86_64-pc-linux-gnu/aarch64-kos/lib/' >> /etc/ld.so.conf.d/KasperskyOS.conf \
    && ldconfig

RUN su -c 'pip3 install PyYAML mavproxy pymavlink --user --upgrade' user

COPY ./ardupilot /home/user/ardupilot
COPY ./kos /home/user/kos
COPY ./planner /home/user/planner
COPY ./tests /home/user/tests

RUN chown -R 1000:1000 /home/user

CMD ["bash"]
