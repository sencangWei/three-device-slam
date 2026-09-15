FROM three-device-covins:melodic-c5b180b-20260913

ARG NR_JOBS=8

COPY covins-g-c5b180b-runtime.patch /tmp/covins-g-c5b180b-runtime.patch

RUN cd /root/covins_ws/src/covins && \
    git apply --check /tmp/covins-g-c5b180b-runtime.patch && \
    git apply /tmp/covins-g-c5b180b-runtime.patch && \
    cd /root/covins_ws && \
    catkin build -j "${NR_JOBS}" covins_frontend covins_backend
