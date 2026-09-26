# Motors

Various code to manage motor hardware

## Anjoy motor control

[`anjoy-motor`](anjoy-motor/) drives the zoom/focus/iris lens motors and the
pan/tilt head of Anjoy AF camera modules over their motor-MCU UART
(Pelco-D-style frames), and decodes the MCU's live position reports.
Protocol recovered from a live MTF45-4G_AF.

## Zenointel SD-2N-4G pan/tilt

[`zenointel-sd2n4g`](zenointel-sd2n4g/) drives the pan/tilt stepper head of the
Zenointel SD-2N-4G (Goke GK7205V510) directly over memory-mapped PL061 GPIO — no
vendor kernel module and no vendor app, so it also runs under OpenIPC. The GPIO
map, half-step phase table and per-axis config were reverse-engineered from the
stock firmware (`motor.ko`/`gpioStep.ko`). The lens is fixed — no motorized
zoom/focus on this model.

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
