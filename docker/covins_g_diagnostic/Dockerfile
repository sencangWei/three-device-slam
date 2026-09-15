FROM three-device-covins:melodic-c5b180b-runtime1

ARG NR_JOBS=8

COPY placerec_gen_be.cpp /root/covins_ws/src/covins/covins_backend/src/covins_backend/placerec_gen_be.cpp
COPY RelNonCentralPosSolver.cpp /root/covins_ws/src/covins/covins_backend/src/covins_backend/RelNonCentralPosSolver.cpp

RUN cd /root/covins_ws && catkin build -j "${NR_JOBS}" covins_backend
