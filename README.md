# usv_estiplan
linking libraries ROS https://answers.ros.org/question/239690/how-to-include-a-library-in-cmakelists/

* Done : Re-existing modes should not be re-initialized with bad phase
* Done : Frequencies with very low amplitude need to be discarded to prevent noise
* Done : Low amplitude can be gated using a ratio of the highest amplitude
* Solved : Data becomes NaN after running for a while
* A low-pass filter could be applied for removing high frequency noise
* Prediction function has to be added
* TESTING : Has to be attempted on actual IMU data
* High DC offset breaks the Kalman Filter