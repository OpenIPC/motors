# Motors

Various code to manage motor hardware

## Common motor service prototype

[`motorsd`](motorsd/) is a host-tested prototype for shared motor control.
It contains the service, the public socket protocol, `libmotors`, command-line
access, separate driver programs, and architecture tests.

Read the [`motorsd` documentation](motorsd/README.md) for the current scope and
usage examples.

## Pelco-D configuration TUI

[`pelcodtui`](pelcodtui/) is an ncurses interface for Pelco-D PTZ controller
settings and manual movement. Camera profiles describe controller-specific
commands without hard-coding them in the application.

## Some theory behind

[Basic autofocus algorithms](https://www.csie.ntu.edu.tw/~fuh/personal/Images&Recognition.Vol.9,No.4.Autofocus.pdf)

[Learning approach for autofocus](https://openaccess.thecvf.com/content_CVPR_2020/papers/Herrmann_Learning_to_Autofocus_CVPR_2020_paper.pdf)

[Improving the accuracy and low-light performance of contrast-based autofocus using
supervised machine learning](https://cs.uwaterloo.ca/~vanbeek/Publications/prl-2015.pdf)

-----

## Various links

* XM AF Module description [Function Command List on p.6-10](doc/IVG-N83020S-T.pdf)

* Driver Manuals on stepper motor drivers - http://www.relmon.com/en/index.php/list/13/33.html

* Some code for ms41929 motor driver - https://download.csdn.net/download/u011212383/10668636

* Driver for ms32006 - https://github.com/song7788/driver_for_ms32006

-----

## Similar projects on Github

* https://github.com/Lynch234ok/git_ipc_hi3516C_V3/tree/master/git_ipc
* https://github.com/stonetan/AF-algorithm
* https://github.com/wztforest/test/tree/236bd04cad7aff12e56b7d26cf785b61a23904c9/ipc/hisi-vsrd/ipcam.hi3518A/src/ptz
