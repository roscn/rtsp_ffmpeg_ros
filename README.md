# rtsp_ffmpeg_ros

 Setup

Install libraries to use ffmpeg

```
sudo apt-get install libavcodec-dev
sudo apt-get install libavformat-dev
sudo apt-get install libavutil-dev
sudo apt-get install libswscale-dev
sudo apt-get install libswresample-dev
sudo apt-get install libavdevice-dev
sudo apt-get install libavfilter-dev
```

## Usage

Example usages in launch folder (**only the argument `rtsp_url` is mandatory**):

```xml
 roslaunch rtsp_ffmpeg  run_state.launch  
```
