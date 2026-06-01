define hokuyolaser ranger
(
  sensor(       
    # laser-specific properties
    # factory settings for LMS200  
    pose [ 0 0 0 0 ]
    size [ 0.07 0.07 0.05 ]
    range [ 0.0  5.0 ]
    fov 270.0
    samples 271
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
  odom_error [ 0.03 0.03 0.0 0.03 ]

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
