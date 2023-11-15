# Unofficial Python module for DJI Tello drone

This fork is modified/extended from [hanyazou/TelloPy](https://github.com/hanyazou/TelloPy)
as a Python module for controlling and communicating with the
[Tello drone](https://store.dji.com/product/tello) from DJI/Ryze.
This fork serves mainly as a base for the [ROS](http://www.ros.org/about-ros/)
module [tello_driver](https://github.com/anqixu/tello_driver).

Unlike the base TelloPy, you must install this fork from source.
It is recommended that you do the following (assuming Ubuntu/debian distro),
to make an editable installation:

* `$ cd <PATH_TO_SOURCE>`
* `$ git clone https://github.com/anqixu/TelloPy.git`
* `$ cd TelloPy`
* `$ sudo -H pip install -e .`

## Sources of knowledge

There are a number of other codebases for communicating with the Tello, each
with a partially-overlaping set of features, in no particular order:

* [TelloPy by hanyazou](https://github.com/hanyazou/TelloPy)
* [pytello by PingguSoft](https://bitbucket.org/PingguSoft/pytello)
* [TelloLib by Kragrathea](https://github.com/Kragrathea/TelloLib)
* [gobot/tello by deadprogram](https://github.com/hybridgroup/gobot/tree/master/platforms/dji/tello)

The underlying UDP protocols for communicating with the drone and receiving
its video feed were reverse-engineered by the community, notably those listed above.
Valuable tidbits include, in no particular order:

* [Tello general dev thread](https://tellopilots.com/threads/tello-whats-possible.88/)
* [Log packet decoding thread](https://tellopilots.com/threads/has-anyone-decoded-the-log-headers-messages-from-the-tello.511/)
* [Gobot Tello blog post](https://gobot.io/blog/2018/04/20/hello-tello-hacking-drones-with-go/)
* [TelloPilots Dev Forum](https://tellopilots.com/forums/tello-development.8/)
* [TelloPilots Wiki](https://tellopilots.com/wiki/index/)