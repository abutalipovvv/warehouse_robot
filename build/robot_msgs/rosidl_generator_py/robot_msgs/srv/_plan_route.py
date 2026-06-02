# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_msgs:srv/PlanRoute.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_PlanRoute_Request(type):
    """Metaclass of message 'PlanRoute_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_msgs.srv.PlanRoute_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__plan_route__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__plan_route__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__plan_route__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__plan_route__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__plan_route__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlanRoute_Request(metaclass=Metaclass_PlanRoute_Request):
    """Message class 'PlanRoute_Request'."""

    __slots__ = [
        '_goal_lm',
        '_start_lm',
        '_use_start_pose',
        '_start_x',
        '_start_y',
        '_start_yaw',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'goal_lm': 'string',
        'start_lm': 'string',
        'use_start_pose': 'boolean',
        'start_x': 'double',
        'start_y': 'double',
        'start_yaw': 'double',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.goal_lm = kwargs.get('goal_lm', str())
        self.start_lm = kwargs.get('start_lm', str())
        self.use_start_pose = kwargs.get('use_start_pose', bool())
        self.start_x = kwargs.get('start_x', float())
        self.start_y = kwargs.get('start_y', float())
        self.start_yaw = kwargs.get('start_yaw', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.goal_lm != other.goal_lm:
            return False
        if self.start_lm != other.start_lm:
            return False
        if self.use_start_pose != other.use_start_pose:
            return False
        if self.start_x != other.start_x:
            return False
        if self.start_y != other.start_y:
            return False
        if self.start_yaw != other.start_yaw:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def goal_lm(self):
        """Message field 'goal_lm'."""
        return self._goal_lm

    @goal_lm.setter
    def goal_lm(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'goal_lm' field must be of type 'str'"
        self._goal_lm = value

    @builtins.property
    def start_lm(self):
        """Message field 'start_lm'."""
        return self._start_lm

    @start_lm.setter
    def start_lm(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'start_lm' field must be of type 'str'"
        self._start_lm = value

    @builtins.property
    def use_start_pose(self):
        """Message field 'use_start_pose'."""
        return self._use_start_pose

    @use_start_pose.setter
    def use_start_pose(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'use_start_pose' field must be of type 'bool'"
        self._use_start_pose = value

    @builtins.property
    def start_x(self):
        """Message field 'start_x'."""
        return self._start_x

    @start_x.setter
    def start_x(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'start_x' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'start_x' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._start_x = value

    @builtins.property
    def start_y(self):
        """Message field 'start_y'."""
        return self._start_y

    @start_y.setter
    def start_y(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'start_y' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'start_y' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._start_y = value

    @builtins.property
    def start_yaw(self):
        """Message field 'start_yaw'."""
        return self._start_yaw

    @start_yaw.setter
    def start_yaw(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'start_yaw' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'start_yaw' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._start_yaw = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PlanRoute_Response(type):
    """Metaclass of message 'PlanRoute_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_msgs.srv.PlanRoute_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__plan_route__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__plan_route__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__plan_route__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__plan_route__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__plan_route__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlanRoute_Response(metaclass=Metaclass_PlanRoute_Response):
    """Message class 'PlanRoute_Response'."""

    __slots__ = [
        '_ok',
        '_error',
        '_route_json',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'ok': 'boolean',
        'error': 'string',
        'route_json': 'string',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ok = kwargs.get('ok', bool())
        self.error = kwargs.get('error', str())
        self.route_json = kwargs.get('route_json', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.ok != other.ok:
            return False
        if self.error != other.error:
            return False
        if self.route_json != other.route_json:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ok(self):
        """Message field 'ok'."""
        return self._ok

    @ok.setter
    def ok(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'ok' field must be of type 'bool'"
        self._ok = value

    @builtins.property
    def error(self):
        """Message field 'error'."""
        return self._error

    @error.setter
    def error(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'error' field must be of type 'str'"
        self._error = value

    @builtins.property
    def route_json(self):
        """Message field 'route_json'."""
        return self._route_json

    @route_json.setter
    def route_json(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'route_json' field must be of type 'str'"
        self._route_json = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_PlanRoute_Event(type):
    """Metaclass of message 'PlanRoute_Event'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_msgs.srv.PlanRoute_Event')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__plan_route__event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__plan_route__event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__plan_route__event
            cls._TYPE_SUPPORT = module.type_support_msg__srv__plan_route__event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__plan_route__event

            from service_msgs.msg import ServiceEventInfo
            if ServiceEventInfo.__class__._TYPE_SUPPORT is None:
                ServiceEventInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class PlanRoute_Event(metaclass=Metaclass_PlanRoute_Event):
    """Message class 'PlanRoute_Event'."""

    __slots__ = [
        '_info',
        '_request',
        '_response',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'info': 'service_msgs/ServiceEventInfo',
        'request': 'sequence<robot_msgs/PlanRoute_Request, 1>',
        'response': 'sequence<robot_msgs/PlanRoute_Response, 1>',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['service_msgs', 'msg'], 'ServiceEventInfo'),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['robot_msgs', 'srv'], 'PlanRoute_Request'), 1),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.NamespacedType(['robot_msgs', 'srv'], 'PlanRoute_Response'), 1),  # noqa: E501
    )

    def __init__(self, **kwargs):
        if 'check_fields' in kwargs:
            self._check_fields = kwargs['check_fields']
        else:
            self._check_fields = ros_python_check_fields == '1'
        if self._check_fields:
            assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
                'Invalid arguments passed to constructor: %s' % \
                ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from service_msgs.msg import ServiceEventInfo
        self.info = kwargs.get('info', ServiceEventInfo())
        self.request = kwargs.get('request', [])
        self.response = kwargs.get('response', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.get_fields_and_field_types().keys(), self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    if self._check_fields:
                        assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.info != other.info:
            return False
        if self.request != other.request:
            return False
        if self.response != other.response:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def info(self):
        """Message field 'info'."""
        return self._info

    @info.setter
    def info(self, value):
        if self._check_fields:
            from service_msgs.msg import ServiceEventInfo
            assert \
                isinstance(value, ServiceEventInfo), \
                "The 'info' field must be a sub message of type 'ServiceEventInfo'"
        self._info = value

    @builtins.property
    def request(self):
        """Message field 'request'."""
        return self._request

    @request.setter
    def request(self, value):
        if self._check_fields:
            from robot_msgs.srv import PlanRoute_Request
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, PlanRoute_Request) for v in value) and
                 True), \
                "The 'request' field must be a set or sequence with length <= 1 and each value of type 'PlanRoute_Request'"
        self._request = value

    @builtins.property
    def response(self):
        """Message field 'response'."""
        return self._response

    @response.setter
    def response(self, value):
        if self._check_fields:
            from robot_msgs.srv import PlanRoute_Response
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1 and
                 all(isinstance(v, PlanRoute_Response) for v in value) and
                 True), \
                "The 'response' field must be a set or sequence with length <= 1 and each value of type 'PlanRoute_Response'"
        self._response = value


class Metaclass_PlanRoute(type):
    """Metaclass of service 'PlanRoute'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_msgs.srv.PlanRoute')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__plan_route

            from robot_msgs.srv import _plan_route
            if _plan_route.Metaclass_PlanRoute_Request._TYPE_SUPPORT is None:
                _plan_route.Metaclass_PlanRoute_Request.__import_type_support__()
            if _plan_route.Metaclass_PlanRoute_Response._TYPE_SUPPORT is None:
                _plan_route.Metaclass_PlanRoute_Response.__import_type_support__()
            if _plan_route.Metaclass_PlanRoute_Event._TYPE_SUPPORT is None:
                _plan_route.Metaclass_PlanRoute_Event.__import_type_support__()


class PlanRoute(metaclass=Metaclass_PlanRoute):
    from robot_msgs.srv._plan_route import PlanRoute_Request as Request
    from robot_msgs.srv._plan_route import PlanRoute_Response as Response
    from robot_msgs.srv._plan_route import PlanRoute_Event as Event

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
