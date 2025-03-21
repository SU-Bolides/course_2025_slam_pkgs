# generated from rosidl_generator_py/resource/_idl.py.em
# with input from bolide_interfaces:msg/MultipleRange.idl
# generated code does not contain a copyright notice

# This is being done at the module level and not on the instance level to avoid looking
# for the same variable multiple times on each instance. This variable is not supposed to
# change during runtime so it makes sense to only look for it once.
from os import getenv

ros_python_check_fields = getenv('ROS_PYTHON_CHECK_FIELDS', default='')


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MultipleRange(type):
    """Metaclass of message 'MultipleRange'."""

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
            module = import_type_support('bolide_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'bolide_interfaces.msg.MultipleRange')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__multiple_range
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__multiple_range
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__multiple_range
            cls._TYPE_SUPPORT = module.type_support_msg__msg__multiple_range
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__multiple_range

            from sensor_msgs.msg import Range
            if Range.__class__._TYPE_SUPPORT is None:
                Range.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class MultipleRange(metaclass=Metaclass_MultipleRange):
    """Message class 'MultipleRange'."""

    __slots__ = [
        '_ir_rear_left',
        '_ir_rear_right',
        '_sonar_rear',
        '_check_fields',
    ]

    _fields_and_field_types = {
        'ir_rear_left': 'sensor_msgs/Range',
        'ir_rear_right': 'sensor_msgs/Range',
        'sonar_rear': 'sensor_msgs/Range',
    }

    # This attribute is used to store an rosidl_parser.definition variable
    # related to the data type of each of the components the message.
    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Range'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Range'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Range'),  # noqa: E501
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
        from sensor_msgs.msg import Range
        self.ir_rear_left = kwargs.get('ir_rear_left', Range())
        from sensor_msgs.msg import Range
        self.ir_rear_right = kwargs.get('ir_rear_right', Range())
        from sensor_msgs.msg import Range
        self.sonar_rear = kwargs.get('sonar_rear', Range())

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
        if self.ir_rear_left != other.ir_rear_left:
            return False
        if self.ir_rear_right != other.ir_rear_right:
            return False
        if self.sonar_rear != other.sonar_rear:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ir_rear_left(self):
        """Message field 'ir_rear_left'."""
        return self._ir_rear_left

    @ir_rear_left.setter
    def ir_rear_left(self, value):
        if self._check_fields:
            from sensor_msgs.msg import Range
            assert \
                isinstance(value, Range), \
                "The 'ir_rear_left' field must be a sub message of type 'Range'"
        self._ir_rear_left = value

    @builtins.property
    def ir_rear_right(self):
        """Message field 'ir_rear_right'."""
        return self._ir_rear_right

    @ir_rear_right.setter
    def ir_rear_right(self, value):
        if self._check_fields:
            from sensor_msgs.msg import Range
            assert \
                isinstance(value, Range), \
                "The 'ir_rear_right' field must be a sub message of type 'Range'"
        self._ir_rear_right = value

    @builtins.property
    def sonar_rear(self):
        """Message field 'sonar_rear'."""
        return self._sonar_rear

    @sonar_rear.setter
    def sonar_rear(self, value):
        if self._check_fields:
            from sensor_msgs.msg import Range
            assert \
                isinstance(value, Range), \
                "The 'sonar_rear' field must be a sub message of type 'Range'"
        self._sonar_rear = value
