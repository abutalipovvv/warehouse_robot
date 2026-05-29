include "include/pioneer2dx.inc"

define hokuyolaser ranger
(
  sensor(       
    # laser-specific properties
    # factory settings for LMS200  
    pose [ 0 0 0 0 ]
    size [ 0.07 0.07 0.05 ]
    range [ 0.0  5.0 ]
    fov 270.0
    samples 270
    color_rgba [ 0 0 1 0.15 ]
  )
  model
  (
    # generic model properties
    pose [ 0.0 0.0 -0.2 0.0 ]
    size [ 0.07 0.07 0.05 ]  
    color "blue"
  )
)

define camera_realsens_d435 camera
(
	range [ 0.3 3.0 ]
	resolution [ 160 90 ]  # 1280 × 720 / 8
 	fov [ 87 58 ]
	pantilt [ 0 0 ]
	alwayson 1
  size [ 0.025 0.09 0.025 ]
  color "gray"
)

define qrcodereader fiducial
(
  fiducial_key 1 # fiducial_key values of model and fiducial detector must match

  # fiducial properties
  range_min 0.0
  range_max 8.0
  range_max_id 5.0
  fov 270.0
  ignore_zloc 1  #  When set to 1, the fiducial finder ignores the z component when checking a fiducial

  model
  (
    # model properties
    size [ 0.1 0.1 0.1 ]
    color "green"
  )
)

define pioneer2dx_with_laser pioneer2dx_no_sonar 
(
  # can refer to the robot by this name
  hokuyolaser(pose [ 0.15 0 0 0 ])
  model
  (
    # generic model properties
    pose [ -0.1 0.0 0 0.0 ]
    size [ 0.2 0.2 0.05 ]  
    color "DimGray"
  )
)

define pioneer2dx_with_laser_and_camera pioneer2dx_with_laser 
(
  # can refer to the robot by this name
  camera_realsens_d435(pose [ 0 0 0.5 0 ])
  mount( pose [ 0 -0.15 0 0.000 ] color "gray")
  mount( pose [ 0  0.15 0 0.000 ] color "gray")
)


define pioneer2dx_with_two_laser pioneer2dx_no_sonar 
(
  # can refer to the robot by this name
  hokuyolaser(pose [ 0.15 0.15 0 45 ])
  model
  (
    # generic model properties
    pose [ -0.1 0.0 0 0 ]
    size [ 0.2 0.2 0.05 ]  
    color "DimGray"
  )  
  hokuyolaser(pose [ -0.20 -0.15 0 -135 ])
  model
  (
    # generic model properties
    pose [ -0.1 0.0 0 0.0 ]
    size [ 0.2 0.2 0.05 ]  
    color "DimGray"
  )
)
define trp1_base position
(
  color "red"
  drive "diff"
  gui_nose 1

  obstacle_return 1
  ranger_return 0.5
  blob_return 1
  fiducial_return 1

  localization "odom"
  odom_error [ 0.05 0.05 0.0 0.1 ]

  velocity_bounds [-1.0 1.0 0 0 0 0 -45.0 45.0]
  acceleration_bounds [-0.5 0.5 0 0 0 0 -45.0 45.0]
)

define trp1_no_sonar trp1_base
(
  size [0.60 0.50 0.22]
  origin [0.0 0.0 0.0 0.0]
  gui_nose 1
  mass 20.0

  block(
    points 8
    point[0] [-0.30  0.12]
    point[1] [-0.30 -0.12]
    point[2] [-0.20 -0.25]
    point[3] [ 0.20 -0.25]
    point[4] [ 0.30 -0.12]
    point[5] [ 0.30  0.12]
    point[6] [ 0.20  0.25]
    point[7] [-0.20  0.25]
    z [0 0.22]
  )
)

define trp1_with_laser trp1_no_sonar
(
  hokuyolaser(pose [ 0.28 0.0 0 0 ])

  model
  (
    pose [ 0.0 0.0 0.0 0.0 ]
    size [ 0.10 0.10 0.05 ]
    color "DimGray"
  )
)