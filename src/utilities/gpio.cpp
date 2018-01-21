#include "gpio.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#define GPIO_BASE_DIR "/sys/class/gpio"
#define DIR_IN "in\n"
#define DIR_OUT "out\n"
#define EDGE_RISING "rising\n"
#define EDGE_FALLING "falling\n"
#define EDGE_NONE "none\n"
#define LOGIC_LOW "0\n"
#define LOGIC_HIGH "1\n"
#define STATE_ACTIVE_HIGH "0\n"
#define STATE_ACTIVE_LOW "1\n"

GPIO::GPIO(unsigned int pin) : pin_(pin), owner_(false) { openFiles(); }

GPIO::~GPIO() { cleanup(); }

void GPIO::cleanup() {
  closePinFiles();
  if (owner_) {
    // Files should not be exported on destruction. If another instance opened
    // the files after the owner, it will lose access to the files it needs.
    // Better to leave the files hanging around then cause other things to error
    // out. unexport();
    closeExportFiles();
  }
}

void GPIO::closePinFiles() {
  close(direction_fd_);
  close(edge_fd_);
  close(value_fd_);
  close(active_low_fd_);
}

void GPIO::unexport() {
  char pin[10];
  snprintf(pin, sizeof(pin), "%d", pin_);
  // Dont check error here
  ssize_t err = write(unexport_fd_, pin, strlen(pin));
  if ( err == -1){
    throw ros::Exception("Failed to write pin to the file");
  }
}

void GPIO::closeExportFiles() {
  close(export_fd_);
  close(unexport_fd_);
}

void GPIO::openFiles() {
  char path[255];
  struct stat sb;

  snprintf(path, sizeof(path), GPIO_BASE_DIR "/gpio%d", pin_);
  if (stat(path, &sb) != 0) {
    owner_ = true;
    ROS_DEBUG("Owner of %s\n", path);
  } else {
    ROS_DEBUG("Not owner of %s\n", path);
  }

  if (owner_) {
    snprintf(path, sizeof(path), GPIO_BASE_DIR "/export");
    export_fd_ = open(path, O_WRONLY);
    if (export_fd_ < 0) {
      ROS_ERROR("Failed to open export pin: %s\n", strerror(errno));
      exit(-1);
    }

    snprintf(path, sizeof(path), GPIO_BASE_DIR "/unexport");
    unexport_fd_ = open(path, O_WRONLY);
    if (export_fd_ < 0) {
      ROS_ERROR("Failed to open unexport pin: %s\n", strerror(errno));
      exit(-1);
    }

    char pin_num[10];
    snprintf(pin_num, sizeof(pin_num), "%d", pin_);
    int len = strlen(pin_num);

    int ret;
    ret = write(export_fd_, pin_num, len);
    if (ret < len) {
      ROS_ERROR("Failed to export pin: %s\n", strerror(errno));
      exit(-1);
    }
  }
  // GPIO should now be exported, meaning that /sys/class/gpio### exists

  int path_start = snprintf(path, sizeof(path), GPIO_BASE_DIR "/gpio%d", pin_);

  sprintf(&path[path_start], "/edge");
  edge_fd_ = open(path, O_RDWR);
  if (edge_fd_ < 0) {
    ROS_ERROR("Failed to open edge: %s\n", strerror(errno));
    exit(-1);
  }

  sprintf(&path[path_start], "/direction");
  direction_fd_ = open(path, O_RDWR);
  if (direction_fd_ < 0) {
    ROS_ERROR("Failed to open direction: %s\n", strerror(errno));
    exit(-1);
  }

  sprintf(&path[path_start], "/value");
  value_fd_ = open(path, O_RDWR);
  if (value_fd_ < 0) {
    ROS_ERROR("Failed to open value: %s\n", strerror(errno));
    exit(-1);
  }

  sprintf(&path[path_start], "/active_low");
  active_low_fd_ = open(path, O_RDWR);
  if (active_low_fd_ < 0) {
    ROS_ERROR("Failed to open active_low: %s\n", strerror(errno));
    exit(-1);
  }
}

void GPIO::SetActiveState(ActiveState state) {
  int ret = 0;
  switch (state) {
    case ACTIVE_HIGH:
      ret = write(active_low_fd_, STATE_ACTIVE_HIGH, sizeof(STATE_ACTIVE_HIGH));
      break;
    case ACTIVE_LOW:
      ret = write(active_low_fd_, STATE_ACTIVE_LOW, sizeof(STATE_ACTIVE_LOW));
      break;
    default:
      return;
  }
  if (ret < 0) {
    ROS_ERROR("Failed to set active state: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  }
}

void GPIO::SetDirection(Direction dir) {
  int ret = 0;
  switch (dir) {
    case IN:
      ret = write(direction_fd_, DIR_IN, sizeof(DIR_IN));
      break;
    case OUT:
      ret = write(direction_fd_, DIR_OUT, sizeof(DIR_OUT));
      break;
    default:
      ROS_ERROR("Invalid direction: %d\n", dir);
      cleanup();
      exit(-1);
      break;
  }

  if (ret < 0) {
    ROS_ERROR("Failed to set direction of pin: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  }
}

bool GPIO::WaitOn(Edge edge) { return WaitOn(edge, -1); }

bool GPIO::WaitOn(Edge edge, int timeout_ms) {
  int ret = 0;

  // Set the correct edge in the edge file
  switch (edge) {
    case RISING:
      ret = write(edge_fd_, EDGE_RISING, sizeof(EDGE_RISING));
      break;
    case FALLING:
      ret = write(edge_fd_, EDGE_FALLING, sizeof(EDGE_FALLING));
      break;
    case NONE:
    default:
      // Nothing to wait for
      ROS_ERROR("Invalid edge specified: %d", edge);
      cleanup();
      exit(-1);
  }
  if (ret < 0) {
    ROS_ERROR("Failed to set edge before waiting: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  }

  // Preread the value file to remove any pending interrupts
  char dummy;
  ssize_t err = read(value_fd_, &dummy, 1);
  if ( err == -1){
    throw ros::Exception("Failed to read from the value file");
  }
  // Poll the value file for changes. Only pins with interrupt capabilities
  // available can be polled on. For many processors, any pin will be usable
  struct pollfd pollfds;
  memset(&pollfds, 0, sizeof(pollfds));
  pollfds.fd = value_fd_;
  pollfds.events = POLLPRI | POLLERR;

  ret = poll(&pollfds, 1, timeout_ms);

  if (ret == -1) {
    ROS_ERROR("Poll Failed: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  } else if (ret == 0) {
    // Timeout was reached
    return false;
  } else if (ret & POLLPRI && ret & POLLERR) {
    // Successful wait. Edge detected
    return true;
  }
  // Failed for other reasons. Not going to retry here
  return false;
}

GPIO::Direction GPIO::GetDirection() {
  char direction[10];
  int ret = read(direction_fd_, direction, sizeof(direction));
  if (ret <= 0) {
    ROS_ERROR("Failed to read direction: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  }
  // Add a terminating NULL char to make it a proper string
  direction[ret] = '\0';
  if (strcmp(direction, DIR_IN) == 0) {
    return IN;
  } else if (strcmp(direction, DIR_OUT) == 0) {
    return OUT;
  } else {
    ROS_ERROR("Unknown direction: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  }
}

void GPIO::SetValue(LogicLevel val) {
  if (GetDirection() == IN) {
    ROS_ERROR("Can't set value of an input pin: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  }
  int ret;
  switch (val) {
    case HIGH:
      ret = write(value_fd_, LOGIC_HIGH, sizeof(LOGIC_HIGH));
      break;
    case LOW:
      ret = write(value_fd_, LOGIC_LOW, sizeof(LOGIC_LOW));
      break;
    default:
      ROS_ERROR("Invalid logic level: %d\n", val);
      cleanup();
      exit(-1);
      break;
  }
  if (ret == -1) {
    ROS_ERROR("Unable to set pin value: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  }
}

GPIO::LogicLevel GPIO::GetValue() {
  char value[10];
  int ret;

  // Have to get back to the start of the file, otherwise the read does nothing
  ret = lseek(value_fd_, 0, SEEK_SET);
  if (ret < 0) {
    ROS_ERROR("Failed to lseek() value: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  }

  // Value returned will either be a "0" or a "1"
  ret = read(value_fd_, value, sizeof(value));
  if (ret < 0) {
    ROS_ERROR("Failed to read value: %s\n", strerror(errno));
    cleanup();
    exit(-1);
  }

  // Add a terminating NULL char to make it a proper string
  value[ret] = '\0';
  if (strncmp(value, LOGIC_HIGH, 1) == 0) {
    return HIGH;
  } else if (strncmp(value, LOGIC_LOW, 1) == 0) {
    return LOW;
  } else {
    ROS_ERROR("Unknown value read: %s\n", value);
    cleanup();
    exit(-1);
  }
}
