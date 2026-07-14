define laser ranger
(
  sensor
  (
    range [0.0 40.0]
    fov 360
    samples 361
  )
  # generic model properties
  color "DimGray"
  size [ 0.0674 0.0674 0.042 ]
)

define trp1 position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  size [ 1.0 0.70 0.22 ]
  origin [ -0.023 -0.0032 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "red"
  mass 100.0
  
  laser(pose [ 0.32487 0.24906 -0.002 0.0 ])

  localization "odom"
  odom_error [ 0.05 0.05 0.0 0.1 ]
)
