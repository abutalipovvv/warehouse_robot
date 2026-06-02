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


