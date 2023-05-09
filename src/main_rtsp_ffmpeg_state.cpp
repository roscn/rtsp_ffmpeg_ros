#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sstream>
#include <stdexcept>
#include <iostream>
#include <time.h>

#include <thread>
#include <queue>
#include <mutex>

#include <rtsp_ffmpeg/ffmpegdecoder.h>
#include "glog/logging.h"

namespace rtsp_ffmpeg
{

  enum RunState
  {
    Idle = 0,
    Init = 1,
    Connect = 2,
    Disconnect = 3,
    PreRun = 4,    
    Run = 5,
    WaitReConnect = 6
  };


  class RTSPFFmpegConfig
  {
  public:
    RTSPFFmpegConfig(){};

    std::string camera_name = "camera";
    double set_camera_fps = 30.0;
    int buffer_queue_size = 100;
    double fps = 240.0;
    std::string frame_id = "camera";
    std::string camera_info_url = "";
    bool flip_horizontal = false;
    bool flip_vertical = false;
    int width = 0;
    int height = 0;
    double brightness = 0.5019607843137255;
    double contrast = 0.12549019607843137;
    double hue = 0.5;
    double saturation = 0.64;
    double exposure = 0.5;
    bool auto_exposure = true;
    bool loop_videofile = false;
    bool reopen_on_read_failure = false;
    std::string output_encoding = "bgr8";
    int start_frame = 0;
    int stop_frame = -1;
  };

  class RTSPFFmpeg
  {
  public:
    RTSPFFmpeg() : pnh_("~"),ffmpeg_decoder_(nullptr)
    {
      LOG(INFO)<<"RTSPFFmpeg onInit: ";  
      pnh_.param<std::string>("rtsp_url", rtsp_url, "rtmp://ns8.indexforce.com/home/mystream");
      LOG(INFO)<<"rtsp_url: " << rtsp_url;
      pnh_.param<std::string>("img_topic", img_topic, "/image_raw");
      LOG(INFO)<<"img_topic: " << img_topic;
      pnh_.param<double>("rate", rate_, 10.0);
      LOG(INFO)<<"rate: " << rate_;

      img_publisher_ = pnh_.advertise<sensor_msgs::Image>(img_topic, 10);
 
      LoopRun();
    }

    ~RTSPFFmpeg()
    {
      if(ffmpeg_decoder_ != nullptr)
      {
        ffmpeg_decoder_->stop();
        decode_thread.join();
        delete ffmpeg_decoder_;
      }
    }

  protected:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher img_publisher_;

    RTSPFFmpegConfig config;
    std::mutex s_mutex, c_mutex, p_mutex;
    std::string rtsp_url;
    std::string img_topic;    
    FFmpegDecoder* ffmpeg_decoder_;
 
    std::thread decode_thread;
 
    double rate_;    
    RunState run_state_ = RunState::Init;
 

    virtual void do_publish(cv::Mat frame)
    {
      sensor_msgs::Image msg;
      std_msgs::Header header;
      RTSPFFmpegConfig latest_config = config;
      {
        std::lock_guard<std::mutex> lock(p_mutex);
        latest_config = config;
      }

      header.frame_id = latest_config.frame_id;

      if (!frame.empty())
      {
        // From http://docs.opencv.org/modules/core/doc/operations_on_arrays.html#void flip(InputArray src, OutputArray dst, int flipCode)
        // FLIP_HORIZONTAL == 1, FLIP_VERTICAL == 0 or FLIP_BOTH == -1
        // Flip the image if necessary
        if (latest_config.flip_horizontal && latest_config.flip_vertical)
          cv::flip(frame, frame, -1);
        else if (latest_config.flip_horizontal)
          cv::flip(frame, frame, 1);
        else if (latest_config.flip_vertical)
          cv::flip(frame, frame, 0);

        cv_bridge::CvImagePtr cv_image = boost::make_shared<cv_bridge::CvImage>(header, "bgr8", frame);
        if (latest_config.output_encoding != "bgr8")
        {
          try
          {
            // https://github.com/ros-perception/vision_opencv/blob/melodic/cv_bridge/include/cv_bridge/cv_bridge.h#L247
            cv_image = cv_bridge::cvtColor(cv_image, latest_config.output_encoding);
          }
          catch (std::runtime_error &ex)
          {
            //ROS_ERROR_STREAM("cannot change encoding to " << latest_config.output_encoding << ": " << ex.what());
          }
        }
        msg = *(cv_image->toImageMsg());
        LOG(INFO)<< "Publishing msg";
        img_publisher_.publish(msg);
      }
    }

    void LoopRun()
    {
      cv::Mat frame;
      ros::Rate loop_rate(rate_);
      while (ros::ok() )
      {
        LOG(INFO)<<"==>cur_run_state_: " << (int)run_state_<<std::endl;      
        switch (run_state_)
        {
        case RunState::Idle :
          {

          }
          break;
        case RunState::Init :
          {
            run_state_ = RunState::Connect;
          }
          break;
        case RunState::Connect :
          {
            LOG(INFO)<<"subscribe";     
            ffmpeg_decoder_ = new FFmpegDecoder(rtsp_url, nh_);
            LOG(INFO)<<"Connecting to decoder with url: " << rtsp_url<<std::endl;

            ffmpeg_decoder_->connect();
            LOG(INFO)<<"Finish Connecting to decoder with url: "<< rtsp_url;          
            if (ffmpeg_decoder_->isConnected())
            {
              LOG(INFO)<<"Decoder has connected, starting threads!";
              run_state_ = RunState::PreRun;            
            }
            else
            {
              LOG(INFO)<< "Decoder failed to connect!";   
              run_state_ = RunState::WaitReConnect;
            }
          }
          break;
        case RunState::WaitReConnect :
          //sleep(1.0);
          run_state_ = RunState::Init;        
          break;         
        case RunState::Disconnect :
          {
            LOG(INFO)<<"Disconnect" ;   
            if(ffmpeg_decoder_!= nullptr)
            {
              ffmpeg_decoder_->stop();
              LOG(INFO)<< "finish Unsubscribe1" ;              
              decode_thread.join();
              LOG(INFO)<< "finish Unsubscribe2" ;          
              delete ffmpeg_decoder_;
              ffmpeg_decoder_ = nullptr;
              LOG(INFO)<< "finish Unsubscribe3" ;
            }

            run_state_ = RunState::WaitReConnect;
          }
          break;
        case RunState::PreRun:
          {
            try
            {
              decode_thread = std::thread(&FFmpegDecoder::decode, ffmpeg_decoder_);
              LOG(INFO)<<"finish create decode_thread"; 
              run_state_ = RunState::Run;                                 
            }
            catch (std::exception &e)
            {
              LOG(INFO)<<"Failed to start capture thread: %s", e.what();
              run_state_ = RunState::Disconnect;   
            }         
          }
          break;           
        case RunState::Run :
          {
            if(!ffmpeg_decoder_->isConnected())
            {
              run_state_ = RunState::Disconnect;   
              continue;
            }
      
            ffmpeg_decoder_->mtx.lock();
            if (!ffmpeg_decoder_->decodedImgBuf.empty())
            {
              frame = ffmpeg_decoder_->decodedImgBuf.front().clone();
              ffmpeg_decoder_->decodedImgBuf.pop_front();
            }
            ffmpeg_decoder_->mtx.unlock();

            if (!frame.empty())
            {
              // Publish
              LOG(INFO)<<"Publish one frame";
              do_publish(frame);
              // time_t t = time(NULL);
              // struct tm* stime=localtime(&t);
              // char time_str[20]{0};
              // snprintf(time_str,sizeof(time_str),"%04d_%02d_%02d_%02d_%02d_%02d",1900+stime->tm_year,1+stime->tm_mon,stime->tm_mday, stime->tm_hour,stime->tm_min,stime->tm_sec);
              // std::cout<<time_str<<std::endl;
              // std::string pic = "/tmp/"+std::string(time_str)+".jpg";
              // //cv::imwrite("/tmp/show.jpg", frame);
              // cv::imwrite(pic, frame);              
              frame.release();
            }          
          }
          break;                
        default:
          break;
        }
        //std::cout<<"==>run_state_: " << (int)run_state_<<std::endl;     
        loop_rate.sleep();
      }
    }
 
  };
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rtsp_ffmpeg_node");

  rtsp_ffmpeg::RTSPFFmpeg rtsp_ffmpeg;
  ros::spin();

  return 0;
}
