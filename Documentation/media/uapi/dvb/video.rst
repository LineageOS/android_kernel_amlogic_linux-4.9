.. -*- coding: utf-8; mode: rst -*-

.. _dvb_video:

################
DVB Video Device
################
The DVB video device controls the MPEG2 video decoder of the DVB
hardware. It can be accessed through **/dev/dvb0.video0**. Data
types and and ioctl definitions can be accessed by including
**linux/dvb/video.h** in your application.

Note that the DVB video device only controls decoding of the MPEG video
stream, not its presentation on the TV or computer screen. On PCs this
is typically handled by an associated video4linux device, e.g.
**/dev/video**, which allows scaling and defining output windows.

Some DVB cards don’t have their own MPEG decoder, which results in the
omission of the audio and video device as well as the video4linux
device.

The ioctls that deal with SPUs (sub picture units) and navigation
packets are only supported on some MPEG decoders made for DVD playback.

These ioctls were also used by V4L2 to control MPEG decoders implemented
in V4L2. The use of these ioctls for that purpose has been made obsolete
and proper V4L2 ioctls or controls have been created to replace that
functionality.


.. toctree::
    :maxdepth: 1

    video_types
    video_function_calls
