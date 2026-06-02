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
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__SetTeleop_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__SetTeleop_Request__init(msg: *mut SetTeleop_Request) -> bool;
    fn robot_msgs__srv__SetTeleop_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetTeleop_Request>, size: usize) -> bool;
    fn robot_msgs__srv__SetTeleop_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetTeleop_Request>);
    fn robot_msgs__srv__SetTeleop_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetTeleop_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<SetTeleop_Request>) -> bool;
}

// Corresponds to robot_msgs__srv__SetTeleop_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetTeleop_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub linear: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angular: f64,


    // This member is not documented.
    #[allow(missing_docs)]
    pub timeout_ms: u32,

}



impl Default for SetTeleop_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__SetTeleop_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__SetTeleop_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetTeleop_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__SetTeleop_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__SetTeleop_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__SetTeleop_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetTeleop_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetTeleop_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/SetTeleop_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__SetTeleop_Request() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__SetTeleop_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__SetTeleop_Response__init(msg: *mut SetTeleop_Response) -> bool;
    fn robot_msgs__srv__SetTeleop_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<SetTeleop_Response>, size: usize) -> bool;
    fn robot_msgs__srv__SetTeleop_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<SetTeleop_Response>);
    fn robot_msgs__srv__SetTeleop_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<SetTeleop_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<SetTeleop_Response>) -> bool;
}

// Corresponds to robot_msgs__srv__SetTeleop_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetTeleop_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: rosidl_runtime_rs::String,

}



impl Default for SetTeleop_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__SetTeleop_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__SetTeleop_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for SetTeleop_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__SetTeleop_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__SetTeleop_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__SetTeleop_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for SetTeleop_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for SetTeleop_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/SetTeleop_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__SetTeleop_Response() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__ReleaseManual_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__ReleaseManual_Request__init(msg: *mut ReleaseManual_Request) -> bool;
    fn robot_msgs__srv__ReleaseManual_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ReleaseManual_Request>, size: usize) -> bool;
    fn robot_msgs__srv__ReleaseManual_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ReleaseManual_Request>);
    fn robot_msgs__srv__ReleaseManual_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ReleaseManual_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<ReleaseManual_Request>) -> bool;
}

// Corresponds to robot_msgs__srv__ReleaseManual_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReleaseManual_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ReleaseManual_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__ReleaseManual_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__ReleaseManual_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ReleaseManual_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ReleaseManual_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ReleaseManual_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ReleaseManual_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ReleaseManual_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ReleaseManual_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/ReleaseManual_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__ReleaseManual_Request() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__ReleaseManual_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__ReleaseManual_Response__init(msg: *mut ReleaseManual_Response) -> bool;
    fn robot_msgs__srv__ReleaseManual_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<ReleaseManual_Response>, size: usize) -> bool;
    fn robot_msgs__srv__ReleaseManual_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<ReleaseManual_Response>);
    fn robot_msgs__srv__ReleaseManual_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<ReleaseManual_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<ReleaseManual_Response>) -> bool;
}

// Corresponds to robot_msgs__srv__ReleaseManual_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReleaseManual_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: rosidl_runtime_rs::String,

}



impl Default for ReleaseManual_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__ReleaseManual_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__ReleaseManual_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for ReleaseManual_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ReleaseManual_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ReleaseManual_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__ReleaseManual_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for ReleaseManual_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for ReleaseManual_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/ReleaseManual_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__ReleaseManual_Response() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__StopRobot_Request() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__StopRobot_Request__init(msg: *mut StopRobot_Request) -> bool;
    fn robot_msgs__srv__StopRobot_Request__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StopRobot_Request>, size: usize) -> bool;
    fn robot_msgs__srv__StopRobot_Request__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StopRobot_Request>);
    fn robot_msgs__srv__StopRobot_Request__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StopRobot_Request>, out_seq: *mut rosidl_runtime_rs::Sequence<StopRobot_Request>) -> bool;
}

// Corresponds to robot_msgs__srv__StopRobot_Request
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StopRobot_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub message: rosidl_runtime_rs::String,

}



impl Default for StopRobot_Request {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__StopRobot_Request__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__StopRobot_Request__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StopRobot_Request {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__StopRobot_Request__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__StopRobot_Request__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__StopRobot_Request__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StopRobot_Request {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StopRobot_Request where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/StopRobot_Request";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__StopRobot_Request() }
  }
}


#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__StopRobot_Response() -> *const std::ffi::c_void;
}

#[link(name = "robot_msgs__rosidl_generator_c")]
extern "C" {
    fn robot_msgs__srv__StopRobot_Response__init(msg: *mut StopRobot_Response) -> bool;
    fn robot_msgs__srv__StopRobot_Response__Sequence__init(seq: *mut rosidl_runtime_rs::Sequence<StopRobot_Response>, size: usize) -> bool;
    fn robot_msgs__srv__StopRobot_Response__Sequence__fini(seq: *mut rosidl_runtime_rs::Sequence<StopRobot_Response>);
    fn robot_msgs__srv__StopRobot_Response__Sequence__copy(in_seq: &rosidl_runtime_rs::Sequence<StopRobot_Response>, out_seq: *mut rosidl_runtime_rs::Sequence<StopRobot_Response>) -> bool;
}

// Corresponds to robot_msgs__srv__StopRobot_Response
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]


// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[repr(C)]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StopRobot_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: rosidl_runtime_rs::String,

}



impl Default for StopRobot_Response {
  fn default() -> Self {
    unsafe {
      let mut msg = std::mem::zeroed();
      if !robot_msgs__srv__StopRobot_Response__init(&mut msg as *mut _) {
        panic!("Call to robot_msgs__srv__StopRobot_Response__init() failed");
      }
      msg
    }
  }
}

impl rosidl_runtime_rs::SequenceAlloc for StopRobot_Response {
  fn sequence_init(seq: &mut rosidl_runtime_rs::Sequence<Self>, size: usize) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__StopRobot_Response__Sequence__init(seq as *mut _, size) }
  }
  fn sequence_fini(seq: &mut rosidl_runtime_rs::Sequence<Self>) {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__StopRobot_Response__Sequence__fini(seq as *mut _) }
  }
  fn sequence_copy(in_seq: &rosidl_runtime_rs::Sequence<Self>, out_seq: &mut rosidl_runtime_rs::Sequence<Self>) -> bool {
    // SAFETY: This is safe since the pointer is guaranteed to be valid/initialized.
    unsafe { robot_msgs__srv__StopRobot_Response__Sequence__copy(in_seq, out_seq as *mut _) }
  }
}

impl rosidl_runtime_rs::Message for StopRobot_Response {
  type RmwMsg = Self;
  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> { msg_cow }
  fn from_rmw_message(msg: Self::RmwMsg) -> Self { msg }
}

impl rosidl_runtime_rs::RmwMessage for StopRobot_Response where Self: Sized {
  const TYPE_NAME: &'static str = "robot_msgs/srv/StopRobot_Response";
  fn get_type_support() -> *const std::ffi::c_void {
    // SAFETY: No preconditions for this function.
    unsafe { rosidl_typesupport_c__get_message_type_support_handle__robot_msgs__srv__StopRobot_Response() }
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
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__SetTeleop() -> *const std::ffi::c_void;
}

// Corresponds to robot_msgs__srv__SetTeleop
#[allow(missing_docs, non_camel_case_types)]
pub struct SetTeleop;

impl rosidl_runtime_rs::Service for SetTeleop {
    type Request = SetTeleop_Request;
    type Response = SetTeleop_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__SetTeleop() }
    }
}




#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__ReleaseManual() -> *const std::ffi::c_void;
}

// Corresponds to robot_msgs__srv__ReleaseManual
#[allow(missing_docs, non_camel_case_types)]
pub struct ReleaseManual;

impl rosidl_runtime_rs::Service for ReleaseManual {
    type Request = ReleaseManual_Request;
    type Response = ReleaseManual_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__ReleaseManual() }
    }
}




#[link(name = "robot_msgs__rosidl_typesupport_c")]
extern "C" {
    fn rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__StopRobot() -> *const std::ffi::c_void;
}

// Corresponds to robot_msgs__srv__StopRobot
#[allow(missing_docs, non_camel_case_types)]
pub struct StopRobot;

impl rosidl_runtime_rs::Service for StopRobot {
    type Request = StopRobot_Request;
    type Response = StopRobot_Response;

    fn get_type_support() -> *const std::ffi::c_void {
        // SAFETY: No preconditions for this function.
        unsafe { rosidl_typesupport_c__get_service_type_support_handle__robot_msgs__srv__StopRobot() }
    }
}


