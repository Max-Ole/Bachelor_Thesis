# rescue_jackal

Depends on the following package: https://github.com/l-schilling/jackal_simulation_rob

And uses elevation mapping
Following error can occure during catkin_make, change the line in the corresponding file to EIGEN_MAKE_ALIGNED_OPERATOR_NEW:
"/home/rescue/catkin_ws/src/3_jackal/elevation_mapping/elevation_mapping/include/elevation_mapping/PointXYZRGBConfidenceRatio.hpp:31:3: error: ‘PCL_MAKE_ALIGNED_OPERATOR_NEW’ does not name a type; did you mean ‘EIGEN_MAKE_ALIGNED_OPERATOR_NEW’?
   PCL_MAKE_ALIGNED_OPERATOR_NEW
   ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
"


Or this error can occure during catkin_make, change "filter_chain.hpp" to "filter_chain.h" in the corresponding file:
"/home/rescue/catkin_ws/src/3_jackal/elevation_mapping/elevation_mapping/include/elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp:12:10: fatal error: filters/filter_chain.hpp: No such file or directory
 #include <filters/filter_chain.hpp>
          ^~~~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated."


And hector_nist_arenas_gazebo: https://github.com/tu-darmstadt-ros-pkg/hector_nist_arenas_gazebo
