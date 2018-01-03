#ifndef GPIO_H
#define GPIO_H

// Represents a single sysfs gpio pin. See
// https://www.kernel.org/doc/Documentation/gpio/sysfs.txt for more details
// about how sysfs works. Basically, their are some files living in
// /sys/class/gpio that let you control gpio pins by writing and reading them.
// All writes are strings that are suffixed by \n. First, you must export the
// pin by writing the pin number to /sys/class/gpio/export. Then a new folder
// will show up: /sys/class/gpio/gpioN where N is the exported pin number. Note
// that these pin numbers do not necessarily line up with the pin names on the
// board. The gpioN folder contains a few files that can then be used to control
// the pin. Read the kernel docs for more info about their function
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

  // Pin direction getters and setters
  void SetDirection(Direction dir);
  Direction GetDirection();

  // Set the pin to be active high or active low
  void SetActiveState(ActiveState state);

  // Pin value setter and getter
  void SetValue(LogicLevel val);
  LogicLevel GetValue();

  // Wait for an edge with a timeout. If the correct edge is detected, returns
  // true. Otherwise if it times out, returns false
  bool WaitOn(Edge edge, int timeout_ms);
  bool WaitOn(Edge edge);

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
