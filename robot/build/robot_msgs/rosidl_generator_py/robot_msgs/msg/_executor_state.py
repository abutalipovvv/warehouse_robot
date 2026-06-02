# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_msgs:msg/ExecutorState.idl
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


class Metaclass_ExecutorState(type):
    """Metaclass of message 'ExecutorState'."""

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
                'robot_msgs.msg.ExecutorState')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__executor_state
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__executor_state
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__executor_state
            cls._TYPE_SUPPORT = module.type_support_msg__msg__executor_state
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__executor_state

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ExecutorState(metaclass=Metaclass_ExecutorState):
    """Message class 'ExecutorState'."""

    __slots__ = [
        '_stamp',
        '_robot_id',
        '_map_id',
        '_route_active',
        '_state',
        '_message',
        '_target_lm',
        '_current_edge_id',
        '_route_id',
        '_route_progress',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'stamp': 'builtin_interfaces/Time',
        'robot_id': 'string',
        'map_id': 'string',
        'route_active': 'boolean',
        'state': 'string',
        'message': 'string',
        'target_lm': 'string',
        'current_edge_id': 'string',
        'route_id': 'string',
        'route_progress': 'float',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
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
        from builtin_interfaces.msg import Time
        self.stamp = kwargs.get('stamp', Time())
        self.robot_id = kwargs.get('robot_id', str())
        self.map_id = kwargs.get('map_id', str())
        self.route_active = kwargs.get('route_active', bool())
        self.state = kwargs.get('state', str())
        self.message = kwargs.get('message', str())
        self.target_lm = kwargs.get('target_lm', str())
        self.current_edge_id = kwargs.get('current_edge_id', str())
        self.route_id = kwargs.get('route_id', str())
        self.route_progress = kwargs.get('route_progress', float())

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
        if self.stamp != other.stamp:
            return False
        if self.robot_id != other.robot_id:
            return False
        if self.map_id != other.map_id:
            return False
        if self.route_active != other.route_active:
            return False
        if self.state != other.state:
            return False
        if self.message != other.message:
            return False
        if self.target_lm != other.target_lm:
            return False
        if self.current_edge_id != other.current_edge_id:
            return False
        if self.route_id != other.route_id:
            return False
        if self.route_progress != other.route_progress:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def stamp(self):
        """Message field 'stamp'."""
        return self._stamp

    @stamp.setter
    def stamp(self, value):
        if self._check_fields:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'stamp' field must be a sub message of type 'Time'"
        self._stamp = value

    @builtins.property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'robot_id' field must be of type 'str'"
        self._robot_id = value

    @builtins.property
    def map_id(self):
        """Message field 'map_id'."""
        return self._map_id

    @map_id.setter
    def map_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'map_id' field must be of type 'str'"
        self._map_id = value

    @builtins.property
    def route_active(self):
        """Message field 'route_active'."""
        return self._route_active

    @route_active.setter
    def route_active(self, value):
        if self._check_fields:
            assert \
                isinstance(value, bool), \
                "The 'route_active' field must be of type 'bool'"
        self._route_active = value

    @builtins.property
    def state(self):
        """Message field 'state'."""
        return self._state

    @state.setter
    def state(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'state' field must be of type 'str'"
        self._state = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value

    @builtins.property
    def target_lm(self):
        """Message field 'target_lm'."""
        return self._target_lm

    @target_lm.setter
    def target_lm(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'target_lm' field must be of type 'str'"
        self._target_lm = value

    @builtins.property
    def current_edge_id(self):
        """Message field 'current_edge_id'."""
        return self._current_edge_id

    @current_edge_id.setter
    def current_edge_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'current_edge_id' field must be of type 'str'"
        self._current_edge_id = value

    @builtins.property
    def route_id(self):
        """Message field 'route_id'."""
        return self._route_id

    @route_id.setter
    def route_id(self, value):
        if self._check_fields:
            assert \
                isinstance(value, str), \
                "The 'route_id' field must be of type 'str'"
        self._route_id = value

    @builtins.property
    def route_progress(self):
        """Message field 'route_progress'."""
        return self._route_progress

    @route_progress.setter
    def route_progress(self, value):
        if self._check_fields:
            assert \
                isinstance(value, float), \
                "The 'route_progress' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'route_progress' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._route_progress = value
