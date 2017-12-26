#include "gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <poll.h>
#include <fcntl.h>


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

GPIO::GPIO(unsigned int pin) : pin_(pin), owner_(false) {
  openFiles();
}

GPIO::~GPIO() {
  cleanup();
}

void GPIO::cleanup() {
  closePinFiles();
  if(owner_) {
      unexport();
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
  write(unexport_fd_, pin, strlen(pin));
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
      printf("Owner of %s\n", path);
  } else {
      printf("Not owner of %s\n", path);
  }

  if(owner_) {
      snprintf(path, sizeof(path), GPIO_BASE_DIR "/export");
      export_fd_ = open(path, O_WRONLY);
      if(export_fd_ < 0) {
          perror("Failed to open export\n");
          exit(-1);
      }

      snprintf(path, sizeof(path), GPIO_BASE_DIR "/unexport");
      unexport_fd_ = open(path, O_WRONLY);
      if(export_fd_ < 0) {
          perror("Failed to open unexport\n");
          exit(-1);
      }

      char pin_num[10];
      snprintf(pin_num, sizeof(pin_num), "%d", pin_);
      int len = strlen(pin_num);

      int ret;
      ret = write(export_fd_, pin_num, len);
      if(ret < len) {
          perror("Failed to export pin");
          exit(-1);
      }

  }
  // GPIO should now be exported, meaning that /sys/class/gpio### exists

  int path_start = snprintf(path, sizeof(path), GPIO_BASE_DIR "/gpio%d", pin_);


  sprintf(&path[path_start], "/edge");
  edge_fd_ = open(path, O_RDWR);
  if(edge_fd_ < 0) {
    perror("Failed to open edge\n");
    exit(-1);
  }

  sprintf(&path[path_start], "/direction");
  direction_fd_ = open(path, O_RDWR);
  if(direction_fd_ < 0) {
    perror("Failed to open direction\n");
    exit(-1);
  }

  sprintf(&path[path_start], "/value");
  value_fd_ = open(path, O_RDWR);
  if(value_fd_ < 0) {
    perror("Failed to open value\n");
    exit(-1);
  }

  sprintf(&path[path_start], "/active_low");
  active_low_fd_ = open(path, O_RDWR);
  if(active_low_fd_ < 0) {
    perror("Failed to open active_low\n");
    exit(-1);
  }
}

void GPIO::SetActiveState(ActiveState state) {
  int ret = 0;
  switch(state) {
  case ACTIVE_HIGH:
    ret = write(active_low_fd_, STATE_ACTIVE_HIGH, sizeof(STATE_ACTIVE_HIGH));
    break;
  case ACTIVE_LOW:
    ret = write(active_low_fd_, STATE_ACTIVE_LOW, sizeof(STATE_ACTIVE_LOW));
    break;
  default:
    return;
  }
  if(ret < 0) {
    perror("Failed to set active state\n");
    cleanup();
    exit(-1);
  }
}

void GPIO::SetDirection(Direction dir) {
  int ret = 0;
  switch(dir) {
  case IN:
    ret = write(direction_fd_, DIR_IN, sizeof(DIR_IN));
    break;
  case OUT:
    ret = write(direction_fd_, DIR_OUT, sizeof(DIR_OUT));
    break;
  default:
    // Maybe should return an error here?
    break;
  }

  if(ret < 0) {
    perror("Failed to set direction of pin\n");
    cleanup();
    exit(-1);
  }
}

void GPIO::WaitOn(Edge edge) {
  WaitOn(edge, -1);
}

void GPIO::WaitOn(Edge edge, int timeout_ms) {
  int ret = 0;

  switch(edge) {
  case RISING:
    ret = write(edge_fd_, EDGE_RISING, sizeof(EDGE_RISING));
    break;
  case FALLING:
    ret = write(edge_fd_, EDGE_FALLING, sizeof(EDGE_FALLING));
    break;
  case NONE:
  default:
    // Nothing to wait for
    return;
  }
  if(ret < 0) {
    perror("Failed to set edge before waiting\n");
    cleanup();
    exit(-1);
  }

  // Preread the value file to remove any pending interrupts
  char dummy;
  read(value_fd_, &dummy, 1);

  struct pollfd pollfds;
  memset(&pollfds, 0, sizeof(pollfds));
  pollfds.fd = value_fd_;
  pollfds.events = POLLPRI | POLLERR;

  ret = poll(&pollfds, 1, timeout_ms);

  if(ret == -1) {
    perror("Poll Failed\n");
    cleanup();
    exit(-1);
  } else if(ret == 0) {
    // Timeout was reached
    return;
  } else if(ret & POLLPRI && ret & POLLERR) {
    // Successful wait. Edge detected
    return;
  }
}

GPIO::Direction GPIO::GetDirection() {
  char direction[10];
  int ret = read(direction_fd_, direction, sizeof(direction));
  if(ret <= 0) {
    perror("Failed to read direction\n");
    cleanup();
    exit(-1);
  }
  direction[ret] = '\0';
  if(strcmp(direction, DIR_IN) == 0) {
    return IN;
  } else if(strcmp(direction, DIR_OUT) == 0) {
    return OUT;
  } else {
    perror("Unknown direction\n");
    cleanup();
    exit(-1);
  }
}

void GPIO::SetValue(LogicLevel val) {
  if(GetDirection() == IN) {
    perror("Can't set value of an input pin\n");
    cleanup();
    exit(-1);
  }
  switch(val) {
  case HIGH:
    write(value_fd_, LOGIC_HIGH, sizeof(LOGIC_HIGH));
    break;
  case LOW:
    write(value_fd_, LOGIC_LOW, sizeof(LOGIC_LOW));
    break;
  default:
    // Invalid logic level. Ignore for now
    break;
  }
}

GPIO::LogicLevel GPIO::GetValue() {
  char value[10];
  int ret;

  ret = lseek(value_fd_, 0, SEEK_SET);
  if(ret < 0) {
    perror("Failed to lseek() value\n");
    cleanup();
    exit(-1);
  }

  ret = read(value_fd_, value, sizeof(value));
  if(ret < 0) {
    perror("Failed to read value\n");
    cleanup();
    exit(-1);
  }

  value[ret] = '\0';
  if(strncmp(value, LOGIC_HIGH, 1) == 0) {
    return HIGH;
  } else if(strncmp(value, LOGIC_LOW, 1) == 0) {
    return LOW;
  } else {
    fprintf(stderr, "Unknown value read: %s\n", value);
    cleanup();
    exit(-1);
  }
}
