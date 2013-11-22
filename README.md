Objective-OVR
=============

An Objective-C framework for the Oculus Rift. This is a wrapper and partial re-design for the Oculus Rift SDK.

As the Oculus Rift SDK is currently still in early stages of development (features can be added or removed), this framework does not try to offer any kind of compatibility between these current versions, it will just be updated to work with the SDK provided in the repository. So the framework may change drastically to adopt the later SDK versions.

The current design for the framework is to associated all behaviour with the OVRDevice (an HMD device), as it assumes functionality is only available when an HMD device is available.


Usage
-----

Download the realease build, or download the same version of the Oculus Rift SDK used but the framework and add the libovr.a to the project.



TO-DO
-----

 * Set up notifications for different events triggered by the Oculus Rift.
 * Expose the rest of the profile functionality (saving, loading, creationg).
 * Allow callback for sensor data, with optional use of Sensor Fusion (currently only uses Sensor Fusion).