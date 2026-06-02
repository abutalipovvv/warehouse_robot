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


