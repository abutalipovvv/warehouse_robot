define laser ranger
(
  sensor
  (
    range [0.0 20.0]
    fov 180
    samples 300
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define trp1 position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  size [ 0.6 0.5 0.15 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "red"
  
  laser(pose [ -0.1 0.0 -0.11 0.0 ])

  localization "odom"
  odom_error [ 0.05 0.05 0.0 0.1 ]
)
