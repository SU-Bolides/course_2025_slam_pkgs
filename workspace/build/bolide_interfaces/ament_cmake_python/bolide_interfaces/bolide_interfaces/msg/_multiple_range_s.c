// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from bolide_interfaces:msg/MultipleRange.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "bolide_interfaces/msg/detail/multiple_range__struct.h"
#include "bolide_interfaces/msg/detail/multiple_range__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__range__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__range__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__range__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__range__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool sensor_msgs__msg__range__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * sensor_msgs__msg__range__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool bolide_interfaces__msg__multiple_range__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("bolide_interfaces.msg._multiple_range.MultipleRange", full_classname_dest, 51) == 0);
  }
  bolide_interfaces__msg__MultipleRange * ros_message = _ros_message;
  {  // ir_rear_left
    PyObject * field = PyObject_GetAttrString(_pymsg, "ir_rear_left");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__range__convert_from_py(field, &ros_message->ir_rear_left)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // ir_rear_right
    PyObject * field = PyObject_GetAttrString(_pymsg, "ir_rear_right");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__range__convert_from_py(field, &ros_message->ir_rear_right)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // sonar_rear
    PyObject * field = PyObject_GetAttrString(_pymsg, "sonar_rear");
    if (!field) {
      return false;
    }
    if (!sensor_msgs__msg__range__convert_from_py(field, &ros_message->sonar_rear)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * bolide_interfaces__msg__multiple_range__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MultipleRange */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("bolide_interfaces.msg._multiple_range");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MultipleRange");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  bolide_interfaces__msg__MultipleRange * ros_message = (bolide_interfaces__msg__MultipleRange *)raw_ros_message;
  {  // ir_rear_left
    PyObject * field = NULL;
    field = sensor_msgs__msg__range__convert_to_py(&ros_message->ir_rear_left);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "ir_rear_left", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ir_rear_right
    PyObject * field = NULL;
    field = sensor_msgs__msg__range__convert_to_py(&ros_message->ir_rear_right);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "ir_rear_right", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sonar_rear
    PyObject * field = NULL;
    field = sensor_msgs__msg__range__convert_to_py(&ros_message->sonar_rear);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "sonar_rear", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
