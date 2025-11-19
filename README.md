



<img width="496" alt="RRT* Simulation" src="https://github.com/kedar2017/dynamic-RRT-star/blob/main/GIF_Video_Image/GIFs_Videos_Images_RRT__Full_Video.gif"> 



Dynamic RRT*
============

For detailed description go here
https://github.com/kedar2017/dynamic-RRT-star/wiki

Dynamic RRT* implementation in C++ with a 2D OpenCV visualizer and support for
dynamic obstacles and online replanning.

The project simulates a robot in a 2D world with static and moving obstacles and
continuously repairs its RRT* tree to keep a collision-free path to the goal.


Features
--------

- RRT* tree growth with cost-based rewiring
- Dynamic obstacles (moving shapes) and online replanning
- Simple robot + world model driven by a time/update loop
- OpenCV-based visualization of robot, tree, and obstacles
- Tunable parameters (step size, neighborhood radius, thresholds)

Dependencies
------------

- CMake >= 3.0
- C++11 compiler
- OpenCV (4.x recommended)

Build and Run
-------------

From the project root:

.. code-block:: bash

   mkdir build
   cd build
   cmake ..
   make

The main executable will be placed in ``bin/``:

.. code-block:: bash

   ./bin/main

Notes
-----

If you hit OpenCV include errors, update the include path in
``include/Render.h`` or adjust your CMake/OpenCV installation to match your
system.
