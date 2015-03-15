#define __STDC_CONSTANT_MACROS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/fill_image.h>

#include <realsense_cam/realsense_cam.h>

#define CLEAR(x) memset (&(x), 0, sizeof (x))

namespace usb_cam {

static void errno_exit(const char * s)
{
  ROS_ERROR("%s error %d, %s", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

static int xioctl(int fd, int request, void * arg)
{
  int r;

  do
    r = ioctl(fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

UsbCam::UsbCam()
  : io_(IO_METHOD_MMAP), fd_(-1), buffers_(NULL), n_buffers_(0), avframe_camera_(NULL),
    avframe_rgb_(NULL), avcodec_(NULL), avoptions_(NULL), avcodec_context_(NULL),
    avframe_camera_size_(0), avframe_rgb_size_(0), video_sws_(NULL), image_(NULL) {
}

UsbCam::~UsbCam()
{
  shutdown();
}

void UsbCam::process_image(const void * src, int len, camera_image_t *dest)
{
  memcpy (dest->image, src, (dest->width * dest->height) << 1);
  for (unsigned int i = 0; i < ((dest->width * dest->height) << 1); i += 2)
    *((unsigned short*)(&dest->image[i])) /= 32;
}

int UsbCam::read_frame()
{
  struct v4l2_buffer buf;
  unsigned int i;
  int len;

  switch (io_)
  {
    case IO_METHOD_READ:
      len = read(fd_, buffers_[0].start, buffers_[0].length);
      if (len == -1)
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit("read");
        }
      }

      process_image(buffers_[0].start, len, image_);

      break;

    case IO_METHOD_MMAP:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit("VIDIOC_DQBUF");
        }
      }

      assert(buf.index < n_buffers_);
      len = buf.bytesused;
      process_image(buffers_[buf.index].start, len, image_);

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");

      break;

    case IO_METHOD_USERPTR:
      CLEAR(buf);

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_USERPTR;

      if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf))
      {
        switch (errno)
        {
          case EAGAIN:
            return 0;

          case EIO:
            /* Could ignore EIO, see spec. */

            /* fall through */

          default:
            errno_exit("VIDIOC_DQBUF");
        }
      }

      for (i = 0; i < n_buffers_; ++i)
        if (buf.m.userptr == (unsigned long)buffers_[i].start && buf.length == buffers_[i].length)
          break;

      assert(i < n_buffers_);
      len = buf.bytesused;
      process_image((void *)buf.m.userptr, len, image_);

      if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");

      break;
  }

  return 1;
}

void UsbCam::stop_capturing(void)
{
  enum v4l2_buf_type type;

  switch (io_)
  {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");

      break;
  }
}

void UsbCam::start_capturing(void)
{
  unsigned int i;
  enum v4l2_buf_type type;

  switch (io_)
  {
    case IO_METHOD_READ:
      /* Nothing to do. */
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i)
      {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF");
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");

      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i)
      {
        struct v4l2_buffer buf;

        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.index = i;
        buf.m.userptr = (unsigned long)buffers_[i].start;
        buf.length = buffers_[i].length;

        if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf))
          errno_exit("VIDIOC_QBUF");
      }

      type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");

      break;
  }
}

void UsbCam::uninit_device(void)
{
  unsigned int i;

  switch (io_)
  {
    case IO_METHOD_READ:
      free(buffers_[0].start);
      break;

    case IO_METHOD_MMAP:
      for (i = 0; i < n_buffers_; ++i)
        if (-1 == munmap(buffers_[i].start, buffers_[i].length))
          errno_exit("munmap");
      break;

    case IO_METHOD_USERPTR:
      for (i = 0; i < n_buffers_; ++i)
        free(buffers_[i].start);
      break;
  }

  free(buffers_);
}

void UsbCam::init_read(unsigned int buffer_size)
{
  buffers_ = (buffer*)calloc(1, sizeof(*buffers_));

  if (!buffers_)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  buffers_[0].length = buffer_size;
  buffers_[0].start = malloc(buffer_size);

  if (!buffers_[0].start)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }
}

void UsbCam::init_mmap(void)
{
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req))
  {
    if (EINVAL == errno)
    {
      ROS_ERROR_STREAM(camera_dev_ << " does not support memory mapping");
      exit(EXIT_FAILURE);
    }
    else
    {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  if (req.count < 2)
  {
    ROS_ERROR_STREAM("Insufficient buffer memory on " << camera_dev_);
    exit(EXIT_FAILURE);
  }

  buffers_ = (buffer*)calloc(req.count, sizeof(*buffers_));

  if (!buffers_)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_)
  {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;

    if (-1 == xioctl(fd_, VIDIOC_QUERYBUF, &buf))
      errno_exit("VIDIOC_QUERYBUF");

    buffers_[n_buffers_].length = buf.length;
    buffers_[n_buffers_].start = mmap(NULL /* start anywhere */, buf.length, PROT_READ | PROT_WRITE /* required */,
				      MAP_SHARED /* recommended */,
				      fd_, buf.m.offset);

    if (MAP_FAILED == buffers_[n_buffers_].start)
      errno_exit("mmap");
  }
}

void UsbCam::init_userp(unsigned int buffer_size)
{
  struct v4l2_requestbuffers req;
  unsigned int page_size;

  page_size = getpagesize();
  buffer_size = (buffer_size + page_size - 1) & ~(page_size - 1);

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_USERPTR;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req))
  {
    if (EINVAL == errno)
    {
      ROS_ERROR_STREAM(camera_dev_ << " does not support "
                "user pointer i/o");
      exit(EXIT_FAILURE);
    }
    else
    {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  buffers_ = (buffer*)calloc(4, sizeof(*buffers_));

  if (!buffers_)
  {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  for (n_buffers_ = 0; n_buffers_ < 4; ++n_buffers_)
  {
    buffers_[n_buffers_].length = buffer_size;
    buffers_[n_buffers_].start = memalign(/* boundary */page_size, buffer_size);

    if (!buffers_[n_buffers_].start)
    {
      ROS_ERROR("Out of memory");
      exit(EXIT_FAILURE);
    }
  }
}

void UsbCam::init_device(int image_width, int image_height, int framerate)
{
  struct v4l2_capability cap;
  struct v4l2_cropcap cropcap;
  struct v4l2_crop crop;
  struct v4l2_format fmt;
  unsigned int min;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCAP, &cap))
  {
    if (EINVAL == errno)
    {
      ROS_ERROR_STREAM(camera_dev_ << " is no V4L2 device");
      exit(EXIT_FAILURE);
    }
    else
    {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
  {
    ROS_ERROR_STREAM(camera_dev_ << " is no video capture device");
    exit(EXIT_FAILURE);
  }

  switch (io_)
  {
    case IO_METHOD_READ:
      if (!(cap.capabilities & V4L2_CAP_READWRITE))
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support read i/o");
        exit(EXIT_FAILURE);
      }

      break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
      if (!(cap.capabilities & V4L2_CAP_STREAMING))
      {
        ROS_ERROR_STREAM(camera_dev_ << " does not support streaming i/o");
        exit(EXIT_FAILURE);
      }

      break;
  }

  /* Select video input, video standard and tune here. */

  CLEAR(cropcap);

  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (0 == xioctl(fd_, VIDIOC_CROPCAP, &cropcap))
  {
    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    crop.c = cropcap.defrect; /* reset to default */

    if (-1 == xioctl(fd_, VIDIOC_S_CROP, &crop))
    {
      switch (errno)
      {
        case EINVAL:
          /* Cropping not supported. */
          break;
        default:
          /* Errors ignored. */
          break;
      }
    }
  }
  else
  {
    /* Errors ignored. */
  }

  CLEAR(fmt);

//  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
//  fmt.fmt.pix.width = 640;
//  fmt.fmt.pix.height = 480;
//  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
//  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = image_width;
  fmt.fmt.pix.height = image_height;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (-1 == xioctl(fd_, VIDIOC_S_FMT, &fmt))
    errno_exit("VIDIOC_S_FMT");

  /* Note VIDIOC_S_FMT may change width and height. */

  /* Buggy driver paranoia. */
  min = fmt.fmt.pix.width * 2;
  if (fmt.fmt.pix.bytesperline < min)
    fmt.fmt.pix.bytesperline = min;
  min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
  if (fmt.fmt.pix.sizeimage < min)
    fmt.fmt.pix.sizeimage = min;

  image_width = fmt.fmt.pix.width;
  image_height = fmt.fmt.pix.height;

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_PARM, &stream_params) < 0)
    errno_exit("Couldn't query v4l fps!");

  ROS_DEBUG("Capability flag: 0x%x", stream_params.parm.capture.capability);

  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = framerate;
  if (xioctl(fd_, VIDIOC_S_PARM, &stream_params) < 0)
    ROS_WARN("Couldn't set camera framerate");
  else
    ROS_DEBUG("Set framerate to be %i", framerate);

  switch (io_)
  {
    case IO_METHOD_READ:
      init_read(fmt.fmt.pix.sizeimage);
      break;

    case IO_METHOD_MMAP:
      init_mmap();
      break;

    case IO_METHOD_USERPTR:
      init_userp(fmt.fmt.pix.sizeimage);
      break;
  }
}

void UsbCam::close_device(void)
{
  if (-1 == close(fd_))
    errno_exit("close");

  fd_ = -1;
}

void UsbCam::open_device(void)
{
  struct stat st;

  if (-1 == stat(camera_dev_.c_str(), &st))
  {
    ROS_ERROR_STREAM("Cannot identify '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (!S_ISCHR(st.st_mode))
  {
    ROS_ERROR_STREAM(camera_dev_ << " is no device");
    exit(EXIT_FAILURE);
  }

  fd_ = open(camera_dev_.c_str(), O_RDWR /* required */| O_NONBLOCK, 0);

  if (-1 == fd_)
  {
    ROS_ERROR_STREAM("Cannot open '" << camera_dev_ << "': " << errno << ", " << strerror(errno));
    exit(EXIT_FAILURE);
  }
}

void UsbCam::start(const std::string& dev, io_method io_method, int image_width, int image_height,
		   int framerate)
{
  camera_dev_ = dev;

  io_ = io_method;

  open_device();
  init_device(image_width, image_height, framerate);
  start_capturing();

  image_ = (camera_image_t *)calloc(1, sizeof(camera_image_t));

  image_->width = image_width;
  image_->height = image_height;
  image_->bytes_per_pixel = 16;

  image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
  image_->is_new = 0;
  image_->image = (char *)calloc(image_->image_size, sizeof(char));
  memset(image_->image, 0, image_->image_size * sizeof(char));
}

void UsbCam::shutdown(void)
{
  stop_capturing();
  uninit_device();
  close_device();

  if (avcodec_context_)
  {
    avcodec_close(avcodec_context_);
    av_free(avcodec_context_);
    avcodec_context_ = NULL;
  }
  if (avframe_camera_)
    av_free(avframe_camera_);
  avframe_camera_ = NULL;
  if (avframe_rgb_)
    av_free(avframe_rgb_);
  avframe_rgb_ = NULL;
  if(image_)
    free(image_);
  image_ = NULL;
}

void UsbCam::grab_image(sensor_msgs::Image* msg)
{
  // grab the image
  grab_image();
  // stamp the image
  msg->header.stamp = ros::Time::now();
  // fill the info
  fillImage(*msg, "rgb8", image_->height, image_->width, 2 * image_->width, image_->image);
}

void UsbCam::grab_image()
{
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(fd_ + 1, &fds, NULL, NULL, &tv);

  if (-1 == r)
  {
    if (EINTR == errno)
      return;

    errno_exit("select");
  }

  if (0 == r)
  {
    ROS_ERROR("select timeout");
    exit(EXIT_FAILURE);
  }

  read_frame();
  image_->is_new = 1;
}

// enables/disables auto focus
void UsbCam::set_auto_focus(int value)
{
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl))
  {
    if (errno != EINVAL)
    {
      perror("VIDIOC_QUERYCTRL");
      return;
    }
    else
    {
      ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
      return;
    }
  }
  else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
    return;
  }
  else
  {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == xioctl(fd_, VIDIOC_S_CTRL, &control))
    {
      perror("VIDIOC_S_CTRL");
      return;
    }
  }
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param, int value)
{
  set_v4l_parameter(param, boost::lexical_cast<std::string>(value));
}
/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param, const std::string& value)
{
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << camera_dev_ << " -c " << param << "=" << value << " 2>&1";
  std::string cmd = ss.str();

  // capture the output
  std::string output;
  int buffer_size = 256;
  char buffer[buffer_size];
  FILE *stream = popen(cmd.c_str(), "r");
  if (stream)
  {
    while (!feof(stream))
      if (fgets(buffer, buffer_size, stream) != NULL)
        output.append(buffer);
    pclose(stream);
    // any output should be an error
    if (output.length() > 0)
      ROS_WARN("%s", output.c_str());
  }
  else
    ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());
}

UsbCam::io_method UsbCam::io_method_from_string(const std::string& str)
{
  if (str == "mmap")
    return IO_METHOD_MMAP;
  else if (str == "read")
    return IO_METHOD_READ;
  else if (str == "userptr")
    return IO_METHOD_USERPTR;
  else
    return IO_METHOD_UNKNOWN;
}
}
