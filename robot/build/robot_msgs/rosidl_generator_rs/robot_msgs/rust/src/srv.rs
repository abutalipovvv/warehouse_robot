#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};




// Corresponds to robot_msgs__srv__PlanRoute_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlanRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub goal_lm: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub start_lm: std::string::String,


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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::PlanRoute_Request::default())
  }
}

impl rosidl_runtime_rs::Message for PlanRoute_Request {
  type RmwMsg = super::srv::rmw::PlanRoute_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_lm: msg.goal_lm.as_str().into(),
        start_lm: msg.start_lm.as_str().into(),
        use_start_pose: msg.use_start_pose,
        start_x: msg.start_x,
        start_y: msg.start_y,
        start_yaw: msg.start_yaw,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        goal_lm: msg.goal_lm.as_str().into(),
        start_lm: msg.start_lm.as_str().into(),
      use_start_pose: msg.use_start_pose,
      start_x: msg.start_x,
      start_y: msg.start_y,
      start_yaw: msg.start_yaw,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      goal_lm: msg.goal_lm.to_string(),
      start_lm: msg.start_lm.to_string(),
      use_start_pose: msg.use_start_pose,
      start_x: msg.start_x,
      start_y: msg.start_y,
      start_yaw: msg.start_yaw,
    }
  }
}


// Corresponds to robot_msgs__srv__PlanRoute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct PlanRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route_json: std::string::String,

}



impl Default for PlanRoute_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::PlanRoute_Response::default())
  }
}

impl rosidl_runtime_rs::Message for PlanRoute_Response {
  type RmwMsg = super::srv::rmw::PlanRoute_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ok: msg.ok,
        error: msg.error.as_str().into(),
        route_json: msg.route_json.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ok: msg.ok,
        error: msg.error.as_str().into(),
        route_json: msg.route_json.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ok: msg.ok,
      error: msg.error.to_string(),
      route_json: msg.route_json.to_string(),
    }
  }
}


// Corresponds to robot_msgs__srv__ExecuteRoute_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub route_json: std::string::String,

}



impl Default for ExecuteRoute_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ExecuteRoute_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteRoute_Request {
  type RmwMsg = super::srv::rmw::ExecuteRoute_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        route_json: msg.route_json.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        route_json: msg.route_json.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      route_json: msg.route_json.to_string(),
    }
  }
}


// Corresponds to robot_msgs__srv__ExecuteRoute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecuteRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: std::string::String,

}



impl Default for ExecuteRoute_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ExecuteRoute_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ExecuteRoute_Response {
  type RmwMsg = super::srv::rmw::ExecuteRoute_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ok: msg.ok,
        error: msg.error.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ok: msg.ok,
        error: msg.error.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ok: msg.ok,
      error: msg.error.to_string(),
    }
  }
}


// Corresponds to robot_msgs__srv__CancelRoute_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CancelRoute_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for CancelRoute_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::CancelRoute_Request::default())
  }
}

impl rosidl_runtime_rs::Message for CancelRoute_Request {
  type RmwMsg = super::srv::rmw::CancelRoute_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to robot_msgs__srv__CancelRoute_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct CancelRoute_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: std::string::String,

}



impl Default for CancelRoute_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::CancelRoute_Response::default())
  }
}

impl rosidl_runtime_rs::Message for CancelRoute_Response {
  type RmwMsg = super::srv::rmw::CancelRoute_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ok: msg.ok,
        error: msg.error.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ok: msg.ok,
        error: msg.error.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ok: msg.ok,
      error: msg.error.to_string(),
    }
  }
}


// Corresponds to robot_msgs__srv__SetTeleop_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
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
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetTeleop_Request::default())
  }
}

impl rosidl_runtime_rs::Message for SetTeleop_Request {
  type RmwMsg = super::srv::rmw::SetTeleop_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        linear: msg.linear,
        angular: msg.angular,
        timeout_ms: msg.timeout_ms,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      linear: msg.linear,
      angular: msg.angular,
      timeout_ms: msg.timeout_ms,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      linear: msg.linear,
      angular: msg.angular,
      timeout_ms: msg.timeout_ms,
    }
  }
}


// Corresponds to robot_msgs__srv__SetTeleop_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct SetTeleop_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: std::string::String,

}



impl Default for SetTeleop_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::SetTeleop_Response::default())
  }
}

impl rosidl_runtime_rs::Message for SetTeleop_Response {
  type RmwMsg = super::srv::rmw::SetTeleop_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ok: msg.ok,
        error: msg.error.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ok: msg.ok,
        error: msg.error.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ok: msg.ok,
      error: msg.error.to_string(),
    }
  }
}


// Corresponds to robot_msgs__srv__ReleaseManual_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReleaseManual_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub structure_needs_at_least_one_member: u8,

}



impl Default for ReleaseManual_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ReleaseManual_Request::default())
  }
}

impl rosidl_runtime_rs::Message for ReleaseManual_Request {
  type RmwMsg = super::srv::rmw::ReleaseManual_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      structure_needs_at_least_one_member: msg.structure_needs_at_least_one_member,
    }
  }
}


// Corresponds to robot_msgs__srv__ReleaseManual_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ReleaseManual_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: std::string::String,

}



impl Default for ReleaseManual_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::ReleaseManual_Response::default())
  }
}

impl rosidl_runtime_rs::Message for ReleaseManual_Response {
  type RmwMsg = super::srv::rmw::ReleaseManual_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ok: msg.ok,
        error: msg.error.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ok: msg.ok,
        error: msg.error.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ok: msg.ok,
      error: msg.error.to_string(),
    }
  }
}


// Corresponds to robot_msgs__srv__StopRobot_Request

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StopRobot_Request {

    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,

}



impl Default for StopRobot_Request {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::StopRobot_Request::default())
  }
}

impl rosidl_runtime_rs::Message for StopRobot_Request {
  type RmwMsg = super::srv::rmw::StopRobot_Request;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        message: msg.message.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        message: msg.message.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      message: msg.message.to_string(),
    }
  }
}


// Corresponds to robot_msgs__srv__StopRobot_Response

// This struct is not documented.
#[allow(missing_docs)]

#[allow(non_camel_case_types)]
#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct StopRobot_Response {

    // This member is not documented.
    #[allow(missing_docs)]
    pub ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub error: std::string::String,

}



impl Default for StopRobot_Response {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::srv::rmw::StopRobot_Response::default())
  }
}

impl rosidl_runtime_rs::Message for StopRobot_Response {
  type RmwMsg = super::srv::rmw::StopRobot_Response;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        ok: msg.ok,
        error: msg.error.as_str().into(),
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
      ok: msg.ok,
        error: msg.error.as_str().into(),
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      ok: msg.ok,
      error: msg.error.to_string(),
    }
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


