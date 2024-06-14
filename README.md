# zongmu radar scripts

scripts to parse zongmu radars data

Fig 1 in setup.jpg shows the extrinsic parameters of the radars.
Note that zongmu specifies that right is positive, left is negative for yaw angles.

radar_para_sda.ini is the radar extrinsic parameters relative to the vehicle frame (see [setup](setup.jpg)).

纵目ZM SDR2没有对点云进行畸变校正。
