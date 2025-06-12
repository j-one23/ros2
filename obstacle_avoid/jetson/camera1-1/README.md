gstreamer

> gst-launch-1.0 -v udpsrc port=8001 ! ‘application/x-rtp,encodingname=(string)H264,payload=(int)96’ ! rtph264depay ! queue !
avdec_h264 ! videoconvert ! autovideosink

subscriber 노드에서 영상원본을 그레이영상,
이진영상으로 각각 변환하고 3가지 영상을 PC로 전송
