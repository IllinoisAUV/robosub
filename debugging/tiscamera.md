# TISCamera Debugging tips


* Use the latest gscam for support of gst-launch-1.0. This should be installed by default in the installer.

* Make sure you use a USB 3.0 cable to get decent FPS

* List plugged in cameras using `tcam-ctrl -l`

* View the available resolutions using `tcam-ctrl -c <serial number>`. These are the only configurations available.
