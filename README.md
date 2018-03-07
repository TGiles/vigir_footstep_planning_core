# Reference for this readme [http://wiki.ros.org/vigir_footstep_planning]
# 1. Getting Started
* Make sure you have run the install_vigir_footstep script from the vigir_footstep_install repository on gitlab under the vigir_footstep_devel_one group.
* Make sure you have built the workspace after wstool does its merge magic.

# 2. Running the ViGIR footstep planner
* The default demo can be launched by:
<pre>
roslaunch vigir_footstep_planner footstep_planner_test.launch
</pre>
* The NAO demo can be launched by:
<pre>
roslaunch vigir_footstep_planner footstep_planner_test_nao.launch
</pre>
## All generated step plans are published on vigir_footstep_planning/step_plan

# 3. Visualizing the footstep planning
* A fully pre-configured RViz environment is started by:
<pre>
roslaunch vigir_footstep_planning rviz_footstep_planning.launch
</pre>
* You can place a start pose and goal pose to trigger planning.