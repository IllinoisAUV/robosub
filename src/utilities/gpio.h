#ifndef GPIO_H
#define GPIO_H

class GPIO {
 public:
  typedef enum {
    NONE,
    RISING,
    FALLING,
  } Edge;

  typedef enum {
    IN,
    OUT,
  } Direction;

  typedef enum {
    ACTIVE_HIGH = 0,
    ACTIVE_LOW = 1,
  } ActiveState;

  typedef enum {
    HIGH,
    LOW,
  } LogicLevel;

  GPIO(unsigned int pin);
  ~GPIO();

  void SetDirection(Direction dir);
  Direction GetDirection();

  void SetActiveState(ActiveState state);

  void SetValue(LogicLevel val);
  LogicLevel GetValue();

  void WaitOn(Edge edge, int timeout_ms);
  void WaitOn(Edge edge);

 private:
  void openFiles();
  void closePinFiles();
  void closeExportFiles();

  void cleanup();

  void unexport();

  unsigned int pin_;

  int export_fd_;
  int unexport_fd_;
  int edge_fd_;
  int direction_fd_;
  int value_fd_;
  int active_low_fd_;
  bool owner_;
};
#endif
