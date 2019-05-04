^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sicktoolbox_wrapper
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.5.4 (2019-05-04)
------------------
* Merge pull request `#6 <https://github.com/ros-drivers/sicktoolbox_wrapper/issues/6>`_ from SantoshBanisetty/kinetic-devel
  Kinetic-devel and readme
* updated Readme
* Kinetic-devel and readme
* Contributors: b2256, santosh banisetty

2.5.3 (2013-08-21)
------------------
* No more Willow Garage email.
* Merge pull request `#2 <https://github.com/ros-drivers/sicktoolbox_wrapper/issues/2>`_ from stwirth/hydro-devel
  added new parameter time_offset
* added new parameter time_offset
* Contributors: Chad Rockey, Stephan Wirth

2.5.2 (2013-04-25)
------------------
* Removed REP 117 Tick/Tock for Hydro.
* Contributors: chadrockey

2.5.1 (2013-03-28)
------------------
* Finished catkinizing with install rules.
* Everything builds with catkin. A bit more cleanup is needed.
* Workaround for Groovy build.
* Tock on REP 117.  Will now default to True.  Warns if not set or false.
* Added a diagnostic publisher.
* Added support for changing measurement units reported by the laser.  Resolves ticket 4931.
* Resolving tickets 4138, 4797, 5358.  The driver will now wait for lasers according to the connect_delay parameter.
* Updated sicklms to add the use_rep_117 parameter and output the appropriate special values.
* Updated pthread linking for to fix Oneric build issues.
* adding sickld patch
* Minor fixes on previous change to sick_lms from patch in `#4290 <https://github.com/ros-drivers/sicktoolbox_wrapper/issues/4290>`_.
* Applied patch from `#4290 <https://github.com/ros-drivers/sicktoolbox_wrapper/issues/4290>`_ with a small tweak. Still needs testing before release.
* Added Ubuntu platform tags to manifest
* Modified the recent patch that allows setting of angle and resolution to default to whatever the laser defaults to.
* Applied patch from jksrecko that adds an angle and resolution parameter, and uses them to call set sick variant.
* adding a little standalone ascii logger
* preparing laser_drivers 1.0.0. Updated manifest descriptions slightly
* sicktoolbox_wrapper: doc reviewed
* Added a doxygen main page.
* staging laser_drivers into tick-tock
* Contributors: Brian Gerkey, Chad Rockey, Gassend Blaise, Ken Conley, Melonee Wise, Morgan Quigley, chadrockey
