#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__PlanRoute_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__PlanRoute_Request__init(msg: *mut PlanRoute_Request) -> bool;
    fn robot_msgs__srv__PlanRoute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlanRoute_Request>, size: usize) -> bool;
    fn robot_msgs__srv__PlanRoute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlanRoute_Request>);
    fn robot_msgs__srv__PlanRoute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlanRoute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<PlanRoute_Request>) -> bool;
}

// Corresponds to robot_msgs__srv__PlanRoute_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlanRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_lm: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_lm: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub use_start_pose: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_x: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_y: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_yaw: f64,

}



impl Default for PlanRoute_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__PlanRoute_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__PlanRoute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlanRoute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__PlanRoute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__PlanRoute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__PlanRoute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlanRoute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlanRoute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/PlanRoute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__PlanRoute_Request() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__PlanRoute_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__PlanRoute_Response__init(msg: *mut PlanRoute_Response) -> bool;
    fn robot_msgs__srv__PlanRoute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<PlanRoute_Response>, size: usize) -> bool;
    fn robot_msgs__srv__PlanRoute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<PlanRoute_Response>);
    fn robot_msgs__srv__PlanRoute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<PlanRoute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<PlanRoute_Response>) -> bool;
}

// Corresponds to robot_msgs__srv__PlanRoute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlanRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route_json: rosidl_runtime_rs::String,

}



impl Default for PlanRoute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__PlanRoute_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__PlanRoute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for PlanRoute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__PlanRoute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__PlanRoute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__PlanRoute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for PlanRoute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for PlanRoute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/PlanRoute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__PlanRoute_Response() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__ExecuteRoute_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__ExecuteRoute_Request__init(msg: *mut ExecuteRoute_Request) -> bool;
    fn robot_msgs__srv__ExecuteRoute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteRoute_Request>, size: usize) -> bool;
    fn robot_msgs__srv__ExecuteRoute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteRoute_Request>);
    fn robot_msgs__srv__ExecuteRoute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteRoute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteRoute_Request>) -> bool;
}

// Corresponds to robot_msgs__srv__ExecuteRoute_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub route_json: rosidl_runtime_rs::String,

}



impl Default for ExecuteRoute_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__ExecuteRoute_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__ExecuteRoute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteRoute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ExecuteRoute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ExecuteRoute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ExecuteRoute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteRoute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteRoute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/ExecuteRoute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__ExecuteRoute_Request() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__ExecuteRoute_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__ExecuteRoute_Response__init(msg: *mut ExecuteRoute_Response) -> bool;
    fn robot_msgs__srv__ExecuteRoute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ExecuteRoute_Response>, size: usize) -> bool;
    fn robot_msgs__srv__ExecuteRoute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ExecuteRoute_Response>);
    fn robot_msgs__srv__ExecuteRoute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ExecuteRoute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ExecuteRoute_Response>) -> bool;
}

// Corresponds to robot_msgs__srv__ExecuteRoute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: rosidl_runtime_rs::String,

}



impl Default for ExecuteRoute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__ExecuteRoute_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__ExecuteRoute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ExecuteRoute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ExecuteRoute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ExecuteRoute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ExecuteRoute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ExecuteRoute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ExecuteRoute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/ExecuteRoute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__ExecuteRoute_Response() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__CancelRoute_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__CancelRoute_Request__init(msg: *mut CancelRoute_Request) -> bool;
    fn robot_msgs__srv__CancelRoute_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CancelRoute_Request>, size: usize) -> bool;
    fn robot_msgs__srv__CancelRoute_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CancelRoute_Request>);
    fn robot_msgs__srv__CancelRoute_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CancelRoute_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<CancelRoute_Request>) -> bool;
}

// Corresponds to robot_msgs__srv__CancelRoute_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CancelRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for CancelRoute_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__CancelRoute_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__CancelRoute_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CancelRoute_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__CancelRoute_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__CancelRoute_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__CancelRoute_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CancelRoute_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CancelRoute_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/CancelRoute_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__CancelRoute_Request() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__CancelRoute_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__CancelRoute_Response__init(msg: *mut CancelRoute_Response) -> bool;
    fn robot_msgs__srv__CancelRoute_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<CancelRoute_Response>, size: usize) -> bool;
    fn robot_msgs__srv__CancelRoute_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<CancelRoute_Response>);
    fn robot_msgs__srv__CancelRoute_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<CancelRoute_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<CancelRoute_Response>) -> bool;
}

// Corresponds to robot_msgs__srv__CancelRoute_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CancelRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: rosidl_runtime_rs::String,

}



impl Default for CancelRoute_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__CancelRoute_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__CancelRoute_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for CancelRoute_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__CancelRoute_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__CancelRoute_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__CancelRoute_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for CancelRoute_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for CancelRoute_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/CancelRoute_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__CancelRoute_Response() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__LoadRobotMap_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__LoadRobotMap_Request__init(msg: *mut LoadRobotMap_Request) -> bool;
    fn robot_msgs__srv__LoadRobotMap_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadRobotMap_Request>, size: usize) -> bool;
    fn robot_msgs__srv__LoadRobotMap_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadRobotMap_Request>);
    fn robot_msgs__srv__LoadRobotMap_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadRobotMap_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadRobotMap_Request>) -> bool;
}

// Corresponds to robot_msgs__srv__LoadRobotMap_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadRobotMap_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub map_name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_dir: rosidl_runtime_rs::String,

}



impl Default for LoadRobotMap_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__LoadRobotMap_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__LoadRobotMap_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadRobotMap_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__LoadRobotMap_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__LoadRobotMap_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__LoadRobotMap_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadRobotMap_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadRobotMap_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/LoadRobotMap_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__LoadRobotMap_Request() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__LoadRobotMap_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__LoadRobotMap_Response__init(msg: *mut LoadRobotMap_Response) -> bool;
    fn robot_msgs__srv__LoadRobotMap_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<LoadRobotMap_Response>, size: usize) -> bool;
    fn robot_msgs__srv__LoadRobotMap_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<LoadRobotMap_Response>);
    fn robot_msgs__srv__LoadRobotMap_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<LoadRobotMap_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<LoadRobotMap_Response>) -> bool;
}

// Corresponds to robot_msgs__srv__LoadRobotMap_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct LoadRobotMap_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_dir: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_id: rosidl_runtime_rs::String,

}



impl Default for LoadRobotMap_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__LoadRobotMap_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__LoadRobotMap_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for LoadRobotMap_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__LoadRobotMap_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__LoadRobotMap_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__LoadRobotMap_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for LoadRobotMap_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for LoadRobotMap_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/LoadRobotMap_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__LoadRobotMap_Response() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__GetRobotMapState_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__GetRobotMapState_Request__init(msg: *mut GetRobotMapState_Request) -> bool;
    fn robot_msgs__srv__GetRobotMapState_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetRobotMapState_Request>, size: usize) -> bool;
    fn robot_msgs__srv__GetRobotMapState_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetRobotMapState_Request>);
    fn robot_msgs__srv__GetRobotMapState_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetRobotMapState_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<GetRobotMapState_Request>) -> bool;
}

// Corresponds to robot_msgs__srv__GetRobotMapState_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetRobotMapState_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for GetRobotMapState_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__GetRobotMapState_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__GetRobotMapState_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetRobotMapState_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__GetRobotMapState_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__GetRobotMapState_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__GetRobotMapState_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetRobotMapState_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetRobotMapState_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/GetRobotMapState_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__GetRobotMapState_Request() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__GetRobotMapState_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__GetRobotMapState_Response__init(msg: *mut GetRobotMapState_Response) -> bool;
    fn robot_msgs__srv__GetRobotMapState_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<GetRobotMapState_Response>, size: usize) -> bool;
    fn robot_msgs__srv__GetRobotMapState_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<GetRobotMapState_Response>);
    fn robot_msgs__srv__GetRobotMapState_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<GetRobotMapState_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<GetRobotMapState_Response>) -> bool;
}

// Corresponds to robot_msgs__srv__GetRobotMapState_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct GetRobotMapState_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_name: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_dir: rosidl_runtime_rs::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_id: rosidl_runtime_rs::String,

}



impl Default for GetRobotMapState_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__GetRobotMapState_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__GetRobotMapState_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for GetRobotMapState_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__GetRobotMapState_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__GetRobotMapState_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__GetRobotMapState_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for GetRobotMapState_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for GetRobotMapState_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/GetRobotMapState_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__GetRobotMapState_Response() }
  }
}






#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__PlanRoute() -> *const std::ffi::c_void;
}

// Corresponds to robot_msgs__srv__PlanRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct PlanRoute;

impl rosidl_runtime_rs::Service for PlanRoute {
    type Request = PlanRoute_Request;
    type Response = PlanRoute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__PlanRoute() }
    }
}




#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__ExecuteRoute() -> *const std::ffi::c_void;
}

// Corresponds to robot_msgs__srv__ExecuteRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct ExecuteRoute;

impl rosidl_runtime_rs::Service for ExecuteRoute {
    type Request = ExecuteRoute_Request;
    type Response = ExecuteRoute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__ExecuteRoute() }
    }
}




#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__CancelRoute() -> *const std::ffi::c_void;
}

// Corresponds to robot_msgs__srv__CancelRoute
#[allow(missing_docs, non_camel_case_types)]
pub struct CancelRoute;

impl rosidl_runtime_rs::Service for CancelRoute {
    type Request = CancelRoute_Request;
    type Response = CancelRoute_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__CancelRoute() }
    }
}




#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__LoadRobotMap() -> *const std::ffi::c_void;
}

// Corresponds to robot_msgs__srv__LoadRobotMap
#[allow(missing_docs, non_camel_case_types)]
pub struct LoadRobotMap;

impl rosidl_runtime_rs::Service for LoadRobotMap {
    type Request = LoadRobotMap_Request;
    type Response = LoadRobotMap_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__LoadRobotMap() }
    }
}




#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__GetRobotMapState() -> *const std::ffi::c_void;
}

// Corresponds to robot_msgs__srv__GetRobotMapState
#[allow(missing_docs, non_camel_case_types)]
pub struct GetRobotMapState;

impl rosidl_runtime_rs::Service for GetRobotMapState {
    type Request = GetRobotMapState_Request;
    type Response = GetRobotMapState_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__GetRobotMapState() }
    }
}


