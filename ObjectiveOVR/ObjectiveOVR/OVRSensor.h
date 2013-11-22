/*
 *  Copyright (c) 2013, Stefan Johnson
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list
 *     of conditions and the following disclaimer.
 *  2. Redistributions in binary form must reproduce the above copyright notice, this
 *     list of conditions and the following disclaimer in the documentation and/or other
 *     materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#import <Foundation/Foundation.h>
#import <ObjectiveOVR/OVRGenericDevice.h>
#import <GLKit/GLKit.h>


typedef int OVRCoordinateFrame;
extern const OVRCoordinateFrame OVRCoordSensor;
extern const OVRCoordinateFrame OVRCoordHMD;


@interface OVRSensor : OVRGenericDevice

@property OVRCoordinateFrame coordinateFrame;
@property unsigned int reportRate;
@property float limitAcceleration;
@property float limitRotationRate;
@property float limitMagneticField;

-(_Bool) setRangeAcceleration: (float)acceleration RotationRate: (float)rotationRate MagneticField: (float)magneticField ShouldWait: (_Bool)wait; //an alternative to using the individual limit properties

@end

@interface OVRSensor (SensorInfo)

@property (readonly) uint16_t vendorID;
@property (readonly) uint16_t productID;
@property (readonly) float maxAcceleration;
@property (readonly) float maxRotationRate;
@property (readonly) float maxMagneticField;
@property (readonly) NSString *serialNumber;

@end

@interface OVRSensor (SensorFusion)

//State Query
@property (readonly) GLKQuaternion orientation;
@property (readonly) GLKQuaternion predictedOrientation; //uses predictionDelta
@property (readonly) GLKVector3 acceleration;
@property (readonly) GLKVector3 angularVelocity;
@property (readonly) GLKVector3 magnetometer;
@property (readonly) GLKVector3 calibratedMagnetometer;
//Configuration
@property _Bool useMotionTracking;
//Prediction Control
@property float predictionDelta;
@property _Bool usePrediction;
//Accelerometer/Gravity Correction Control
@property _Bool useGravity;
@property float accelerometerGain;
//Magnetometer and Yaw Drift Correction Control
@property _Bool useYawCorrection;
@property GLKMatrix4 magnetometerCalibration;
@property (readonly) time_t magnetometerCalibrationTime;
@property (readonly) _Bool hasMagnetometerCalibration;
//Euler Angles
@property (readonly) float yaw;
@property (readonly) float pitch;
@property (readonly) float roll;

//State Query
-(GLKQuaternion) predictOrientationAtInterval: (float)interval;
-(void) resetOrientation;
//Magnetometer and Yaw Drift Correction Control
-(_Bool) saveMagnetometerCalibrationWithName: (NSString*)name;
-(_Bool) loadMagnetometerCalibrationWithName: (NSString*)name;
-(void) clearMagnetometerCalibration;
-(void) clearMagnetometerReferences;
-(GLKVector3) calibratedMagnetometerValue: (GLKVector3)rawMag;
//Euler Angles
-(void) getYaw: (float*)yaw Pitch: (float*)pitch Roll: (float*)roll; //alternative to individual

-(NSString*) descriptionOfSensorData; //Convenience method to get string of current sensor fusion data

@end
