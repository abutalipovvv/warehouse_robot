#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};



// Corresponds to robot_msgs__msg__ExecutorState

// This struct is not documented.
#[allow(missing_docs)]

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct ExecutorState {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route_active: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_lm: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_edge_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route_progress: f32,

}



impl Default for ExecutorState {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::ExecutorState::default())
  }
}

impl rosidl_runtime_rs::Message for ExecutorState {
  type RmwMsg = super::msg::rmw::ExecutorState;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
        map_id: msg.map_id.as_str().into(),
        route_active: msg.route_active,
        state: msg.state.as_str().into(),
        message: msg.message.as_str().into(),
        target_lm: msg.target_lm.as_str().into(),
        current_edge_id: msg.current_edge_id.as_str().into(),
        route_id: msg.route_id.as_str().into(),
        route_progress: msg.route_progress,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
        map_id: msg.map_id.as_str().into(),
      route_active: msg.route_active,
        state: msg.state.as_str().into(),
        message: msg.message.as_str().into(),
        target_lm: msg.target_lm.as_str().into(),
        current_edge_id: msg.current_edge_id.as_str().into(),
        route_id: msg.route_id.as_str().into(),
      route_progress: msg.route_progress,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      robot_id: msg.robot_id.to_string(),
      map_id: msg.map_id.to_string(),
      route_active: msg.route_active,
      state: msg.state.to_string(),
      message: msg.message.to_string(),
      target_lm: msg.target_lm.to_string(),
      current_edge_id: msg.current_edge_id.to_string(),
      route_id: msg.route_id.to_string(),
      route_progress: msg.route_progress,
    }
  }
}


// Corresponds to robot_msgs__msg__RobotStatus
/// Robot state values are published as strings:
/// DISCONNECTED, LOCALIZING, IDLE, MANUAL, EXECUTING_ROUTE, ARRIVED, ERROR

#[cfg_attr(feature = "serde", derive(Deserialize, Serialize))]
#[derive(Clone, Debug, PartialEq, PartialOrd)]
pub struct RobotStatus {

    // This member is not documented.
    #[allow(missing_docs)]
    pub stamp: builtin_interfaces::msg::Time,


    // This member is not documented.
    #[allow(missing_docs)]
    pub robot_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub map_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub connected: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub localization_ok: bool,


    // This member is not documented.
    #[allow(missing_docs)]
    pub localization_age_sec: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub state: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub message: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub target_lm: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub nearest_lm: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub current_edge_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route_id: std::string::String,


    // This member is not documented.
    #[allow(missing_docs)]
    pub route_progress: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose_x: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose_y: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub pose_yaw: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub linear_velocity: f32,


    // This member is not documented.
    #[allow(missing_docs)]
    pub angular_velocity: f32,

}



impl Default for RobotStatus {
  fn default() -> Self {
    <Self as rosidl_runtime_rs::Message>::from_rmw_message(super::msg::rmw::RobotStatus::default())
  }
}

impl rosidl_runtime_rs::Message for RobotStatus {
  type RmwMsg = super::msg::rmw::RobotStatus;

  fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
    match msg_cow {
      std::borrow::Cow::Owned(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Owned(msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
        map_id: msg.map_id.as_str().into(),
        connected: msg.connected,
        localization_ok: msg.localization_ok,
        localization_age_sec: msg.localization_age_sec,
        state: msg.state.as_str().into(),
        message: msg.message.as_str().into(),
        target_lm: msg.target_lm.as_str().into(),
        nearest_lm: msg.nearest_lm.as_str().into(),
        current_edge_id: msg.current_edge_id.as_str().into(),
        route_id: msg.route_id.as_str().into(),
        route_progress: msg.route_progress,
        pose_x: msg.pose_x,
        pose_y: msg.pose_y,
        pose_yaw: msg.pose_yaw,
        linear_velocity: msg.linear_velocity,
        angular_velocity: msg.angular_velocity,
      }),
      std::borrow::Cow::Borrowed(msg) => std::borrow::Cow::Owned(Self::RmwMsg {
        stamp: builtin_interfaces::msg::Time::into_rmw_message(std::borrow::Cow::Borrowed(&msg.stamp)).into_owned(),
        robot_id: msg.robot_id.as_str().into(),
        map_id: msg.map_id.as_str().into(),
      connected: msg.connected,
      localization_ok: msg.localization_ok,
      localization_age_sec: msg.localization_age_sec,
        state: msg.state.as_str().into(),
        message: msg.message.as_str().into(),
        target_lm: msg.target_lm.as_str().into(),
        nearest_lm: msg.nearest_lm.as_str().into(),
        current_edge_id: msg.current_edge_id.as_str().into(),
        route_id: msg.route_id.as_str().into(),
      route_progress: msg.route_progress,
      pose_x: msg.pose_x,
      pose_y: msg.pose_y,
      pose_yaw: msg.pose_yaw,
      linear_velocity: msg.linear_velocity,
      angular_velocity: msg.angular_velocity,
      })
    }
  }

  fn from_rmw_message(msg: Self::RmwMsg) -> Self {
    Self {
      stamp: builtin_interfaces::msg::Time::from_rmw_message(msg.stamp),
      robot_id: msg.robot_id.to_string(),
      map_id: msg.map_id.to_string(),
      connected: msg.connected,
      localization_ok: msg.localization_ok,
      localization_age_sec: msg.localization_age_sec,
      state: msg.state.to_string(),
      message: msg.message.to_string(),
      target_lm: msg.target_lm.to_string(),
      nearest_lm: msg.nearest_lm.to_string(),
      current_edge_id: msg.current_edge_id.to_string(),
      route_id: msg.route_id.to_string(),
      route_progress: msg.route_progress,
      pose_x: msg.pose_x,
      pose_y: msg.pose_y,
      pose_yaw: msg.pose_yaw,
      linear_velocity: msg.linear_velocity,
      angular_velocity: msg.angular_velocity,
    }
  }
}


