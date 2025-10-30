# aylp_tl_piezo
An AnyLoop plugin for the Thorlabs MDT693 3-Axis Piezo Controller.

aylp_tl_piezo.so
--------------

Types and units: `[T_VECTOR, U_V] -> [T_UNCHANGED, U_UNCHANGED]`.

This device interprets the state vector as an array of three voltages, writing them to
each axis of the Thorlabs piezo controller.

### Parameters

- `dev` (string) (required)
  - The name of the controller (e.g., `/dev/ttyACM0`)
- `map` (array) (required)
  - Map each voltage in the state vector to each axis of the piezo controller with repetition. 
- `mask` (array) (required)
  - Enable or disable each axis of the piezo controller.

```json
{
  "pipeline": [
    {
      "uri": "file:/opt/anyloop/aylp_tl_piezo.so",
      "params": {
        "dev": "/dev/ttyACM0",
        "map": [0, 1, 2],
        "mask": [1, 1, 1]
      }
    }
  ]
}

```

Linux dependency
--------------------

This plugin is only supported on Linux distributions.


libserialport dependency
--------------------

This plugin depends on libserialport, which can be installed from the package manager.


Building
--------

Use meson:

```sh
meson setup build
meson compile -C build
```
