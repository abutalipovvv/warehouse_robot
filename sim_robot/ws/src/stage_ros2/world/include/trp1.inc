define hokuyolaser ranger
(
  sensor(       
    # laser-specific properties
    # wide-area simulator LiDAR: full 360 deg coverage, 40 m range
    pose [ 0 0 0 0 ]
    size [ 0.07 0.07 0.05 ]
    range [ 0.0  40.0 ]
    fov 360.0
    samples 361
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
  color "LightSteelBlue"
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
  # Ecom mesh bounds: x [-0.523, 0.477], y [-0.3532, 0.3468].
  # Keep the historical Stage model name so topics and namespaces do not
  # change, but use the physical Ecom collision footprint.
  # The lower shell is 0.17 m high. The separate upper deck overlaps it by
  # 1 cm and brings the simplified visual to 0.205 m overall.
  size [1.00 0.70 0.17]
  # The block points already contain the measured CAD offset from base_link.
  origin [0.0 0.0 0.0 0.0]
  gui_nose 1
  mass 100.0

  # Rounded/chamfered top projection measured from base_link.STL.  The fleet
  # planner deliberately keeps the conservative rectangular footprint, while
  # Stage can render and collide with the closer CAD silhouette.
  block
  (
    points 18
    point[0]  [-0.5230 -0.1840]
    point[1]  [-0.5000 -0.2350]
    point[2]  [-0.4520 -0.3163]
    point[3]  [-0.4339 -0.3280]
    point[4]  [-0.0504 -0.3532]
    point[5]  [ 0.3507 -0.3282]
    point[6]  [ 0.3677 -0.3183]
    point[7]  [ 0.4337 -0.2110]
    point[8]  [ 0.4770 -0.1000]
    point[9]  [ 0.4770  0.1000]
    point[10] [ 0.4337  0.2047]
    point[11] [ 0.3677  0.3119]
    point[12] [ 0.3507  0.3219]
    point[13] [-0.0479  0.3468]
    point[14] [-0.4339  0.3217]
    point[15] [-0.4520  0.3100]
    point[16] [-0.5000  0.2300]
    point[17] [-0.5230  0.1840]
    z [0 0.17]
  )

  # Upper deck and recessed drive wheels make the Stage view recognizable
  # without loading the high-poly mesh.
  model
  (
    # Stage stacks children on the parent's top surface. -0.01 puts this deck
    # at z=0.16 instead of adding the parent height a second time.
    pose [0 0 -0.01 0]
    size [0.76 0.50 0.045]
    color "SlateGray3"
    obstacle_return 0
    ranger_return 0
    gui_nose 0
    block
    (
      points 10
      point[0] [-0.420 -0.220]
      point[1] [-0.360 -0.265]
      point[2] [ 0.275 -0.265]
      point[3] [ 0.355 -0.205]
      point[4] [ 0.380 -0.090]
      point[5] [ 0.380  0.090]
      point[6] [ 0.355  0.205]
      point[7] [ 0.275  0.265]
      point[8] [-0.360  0.265]
      point[9] [-0.420  0.220]
      z [0 0.045]
    )
  )

  model
  (
    pose [-0.043 0.300 -0.165 0]
    size [0.18 0.035 0.12]
    color "DimGray"
    obstacle_return 0
    ranger_return 0
    gui_nose 0
  )
  model
  (
    pose [-0.043 -0.300 -0.165 0]
    size [0.18 0.035 0.12]
    color "DimGray"
    obstacle_return 0
    ranger_return 0
    gui_nose 0
  )
)

define trp1_with_laser trp1_no_sonar
(
  # Preserve the existing frame name "laser", but put its simulated scan at
  # the physical front Ecom lidar. Parent height 0.17 + 0.048 = 0.218 m.
  hokuyolaser(pose [ 0.32487 0.24906 0.048 0 ])

  model
  (
    pose [ 0.32487 0.24906 0.048 0.0 ]
    size [ 0.0674 0.0674 0.042 ]
    color "DimGray"
  )
)
