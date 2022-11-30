FROM ubuntu:20.04

RUN apt-get update && apt-get install -y \
		scons wget \
		python3 python3-pip python3-serial python3-intelhex python3-magic

WORKDIR /opt
RUN wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2 \
    && tar xvf gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2 \
    && rm gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2
ENV PATH $PATH:/opt/gcc-arm-none-eabi-6-2017-q2-update/bin

WORKDIR /app

COPY . .

ENTRYPOINT ["./entrypoint.sh"]
