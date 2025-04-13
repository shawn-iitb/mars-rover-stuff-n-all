# generated from rosidl_generator_py/resource/_idl.py.em
# with input from msg_interfaces:msg/EncoderArm.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'arm_node0'
# Member 'arm_node1'
# Member 'arm_node2'
# Member 'arm_node3'
# Member 'arm_node4'
# Member 'arm_node5'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_EncoderArm(type):
    """Metaclass of message 'EncoderArm'."""

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
            module = import_type_support('msg_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'msg_interfaces.msg.EncoderArm')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__encoder_arm
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__encoder_arm
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__encoder_arm
            cls._TYPE_SUPPORT = module.type_support_msg__msg__encoder_arm
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__encoder_arm

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class EncoderArm(metaclass=Metaclass_EncoderArm):
    """Message class 'EncoderArm'."""

    __slots__ = [
        '_arm_node0',
        '_arm_node1',
        '_arm_node2',
        '_arm_node3',
        '_arm_node4',
        '_arm_node5',
    ]

    _fields_and_field_types = {
        'arm_node0': 'sequence<int64>',
        'arm_node1': 'sequence<int64>',
        'arm_node2': 'sequence<int64>',
        'arm_node3': 'sequence<int64>',
        'arm_node4': 'sequence<int64>',
        'arm_node5': 'sequence<int64>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int64')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int64')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int64')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int64')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int64')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int64')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.arm_node0 = array.array('q', kwargs.get('arm_node0', []))
        self.arm_node1 = array.array('q', kwargs.get('arm_node1', []))
        self.arm_node2 = array.array('q', kwargs.get('arm_node2', []))
        self.arm_node3 = array.array('q', kwargs.get('arm_node3', []))
        self.arm_node4 = array.array('q', kwargs.get('arm_node4', []))
        self.arm_node5 = array.array('q', kwargs.get('arm_node5', []))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
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
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.arm_node0 != other.arm_node0:
            return False
        if self.arm_node1 != other.arm_node1:
            return False
        if self.arm_node2 != other.arm_node2:
            return False
        if self.arm_node3 != other.arm_node3:
            return False
        if self.arm_node4 != other.arm_node4:
            return False
        if self.arm_node5 != other.arm_node5:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def arm_node0(self):
        """Message field 'arm_node0'."""
        return self._arm_node0

    @arm_node0.setter
    def arm_node0(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'q', \
                "The 'arm_node0' array.array() must have the type code of 'q'"
            self._arm_node0 = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -9223372036854775808 and val < 9223372036854775808 for val in value)), \
                "The 'arm_node0' field must be a set or sequence and each value of type 'int' and each integer in [-9223372036854775808, 9223372036854775807]"
        self._arm_node0 = array.array('q', value)

    @builtins.property
    def arm_node1(self):
        """Message field 'arm_node1'."""
        return self._arm_node1

    @arm_node1.setter
    def arm_node1(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'q', \
                "The 'arm_node1' array.array() must have the type code of 'q'"
            self._arm_node1 = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -9223372036854775808 and val < 9223372036854775808 for val in value)), \
                "The 'arm_node1' field must be a set or sequence and each value of type 'int' and each integer in [-9223372036854775808, 9223372036854775807]"
        self._arm_node1 = array.array('q', value)

    @builtins.property
    def arm_node2(self):
        """Message field 'arm_node2'."""
        return self._arm_node2

    @arm_node2.setter
    def arm_node2(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'q', \
                "The 'arm_node2' array.array() must have the type code of 'q'"
            self._arm_node2 = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -9223372036854775808 and val < 9223372036854775808 for val in value)), \
                "The 'arm_node2' field must be a set or sequence and each value of type 'int' and each integer in [-9223372036854775808, 9223372036854775807]"
        self._arm_node2 = array.array('q', value)

    @builtins.property
    def arm_node3(self):
        """Message field 'arm_node3'."""
        return self._arm_node3

    @arm_node3.setter
    def arm_node3(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'q', \
                "The 'arm_node3' array.array() must have the type code of 'q'"
            self._arm_node3 = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -9223372036854775808 and val < 9223372036854775808 for val in value)), \
                "The 'arm_node3' field must be a set or sequence and each value of type 'int' and each integer in [-9223372036854775808, 9223372036854775807]"
        self._arm_node3 = array.array('q', value)

    @builtins.property
    def arm_node4(self):
        """Message field 'arm_node4'."""
        return self._arm_node4

    @arm_node4.setter
    def arm_node4(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'q', \
                "The 'arm_node4' array.array() must have the type code of 'q'"
            self._arm_node4 = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -9223372036854775808 and val < 9223372036854775808 for val in value)), \
                "The 'arm_node4' field must be a set or sequence and each value of type 'int' and each integer in [-9223372036854775808, 9223372036854775807]"
        self._arm_node4 = array.array('q', value)

    @builtins.property
    def arm_node5(self):
        """Message field 'arm_node5'."""
        return self._arm_node5

    @arm_node5.setter
    def arm_node5(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'q', \
                "The 'arm_node5' array.array() must have the type code of 'q'"
            self._arm_node5 = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -9223372036854775808 and val < 9223372036854775808 for val in value)), \
                "The 'arm_node5' field must be a set or sequence and each value of type 'int' and each integer in [-9223372036854775808, 9223372036854775807]"
        self._arm_node5 = array.array('q', value)
