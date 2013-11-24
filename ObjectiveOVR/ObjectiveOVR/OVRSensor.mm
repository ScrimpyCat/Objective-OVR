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

#import "OVRSensor_Private.h"
#import "OVRGenericDevice_Private.h"

using namespace OVR;


const OVRCoordinateFrame OVRCoordSensor = SensorDevice::Coord_Sensor;
const OVRCoordinateFrame OVRCoordHMD = SensorDevice::Coord_HMD;



@implementation OVRSensor
{
    SensorFusion sensorFusion;
}
@synthesize sensor;

-(instancetype) initWithSensorDevice: (Ptr<SensorDevice>)device
{
    if (device == nullptr)
    {
        [self release];
        return nil;
    }
    
    if ((self = [super init]))
    {
        sensor = device;
        sensorFusion.AttachToSensor(device);
        sensorFusion.SetPredictionEnabled();
    }
    
    return self;
}

-(NSString*) description
{
    NSMutableString *Description = [NSMutableString stringWithFormat: @"<%@: %p>", [self class], self];
    
    [Description appendString: @"\n{"];
    
    [Description appendFormat: @"\n\tManufacturer: \"%@\"  Product: \"%@\"  Version: %u", self.manufacturer, self.productName, self.version];
    [Description appendFormat: @"\n\tVendor ID: %u  Product ID: %u  Serial: \"%@\"", self.vendorID, self.productID, self.serialNumber];
    [Description appendFormat: @"\n\tCoordinate Frame: %@", self.coordinateFrame == OVRCoordSensor? @"Sensor coordinate frame" : @"HMD coordinate frame"];
    [Description appendFormat: @"\n\tReport Rate: %u", self.reportRate];
    [Description appendFormat: @"\n\tAcceleration Range: %@  Acceleration Max: %@", @(self.limitAcceleration), @(self.maxAcceleration)];
    [Description appendFormat: @"\n\tRotation Rate Range: %@  Rotation Rate Max: %@", @(self.limitRotationRate), @(self.maxRotationRate)];
    [Description appendFormat: @"\n\tMagnetic Field Range: %@  Magnetic Field Max: %@", @(self.limitMagneticField), @(self.maxMagneticField)];
    
    [Description appendFormat: @"\n\n\tSensor Fusion Configuration"];
    [Description appendString: @"\n\t{"];
    
    NSString *BoolString[2] = { @"No", @"Yes" };
    [Description appendFormat: @"\n\t\tUse Motion Tracking: %@", BoolString[self.useMotionTracking]];
    [Description appendFormat: @"\n\t\tUse Prediction: %@", BoolString[self.usePrediction]];
    [Description appendFormat: @"\n\t\tPrediction Delta: %@", @(self.predictionDelta)];
    [Description appendFormat: @"\n\t\tUse Gravity: %@", BoolString[self.useGravity]];
    [Description appendFormat: @"\n\t\tAccelerometer Gain: %@", @(self.accelerometerGain)];
    [Description appendFormat: @"\n\t\tUse Yaw Correction: %@", BoolString[self.useYawCorrection]];
    [Description appendFormat: @"\n\t\tMagnetometer Calibration: %@", NSStringFromGLKMatrix4(self.magnetometerCalibration)];
    [Description appendFormat: @"\n\t\tMagnetometer Calibration Time: %@", [NSDate dateWithTimeIntervalSince1970: self.magnetometerCalibrationTime]];
    [Description appendFormat: @"\n\t\tHas Magnetometer Calibration: %@", BoolString[self.hasMagnetometerCalibration]];
    
    [Description appendString: @"\n\t}"];
    
    [Description appendString: @"\n}"];
    
    return Description;
}

-(Ptr<DeviceBase>) device
{
    return sensor;
}

-(void) dealloc
{
    if (sensor != nullptr) sensor = nullptr;
    
    [super dealloc];
}

#pragma mark - Sensor Functionality
-(OVRCoordinateFrame) coordinateFrame
{
    return sensor->GetCoordinateFrame();
}

-(void) setCoordinateFrame: (OVRCoordinateFrame)coordinateFrame
{
    sensor->SetCoordinateFrame((SensorDevice::CoordinateFrame)coordinateFrame);
}

-(unsigned int) reportRate
{
    return sensor->GetReportRate();
}

-(void) setReportRate: (unsigned int)reportRate
{
    sensor->SetReportRate(reportRate);
}

-(float) limitAcceleration
{
    SensorRange Range;
    sensor->GetRange(&Range);
    
    return Range.MaxAcceleration;
}

-(void) setLimitAcceleration: (float)acceleration
{
    SensorRange Range = SensorRange(acceleration, self.limitRotationRate, self.limitMagneticField);
    sensor->SetRange(Range, true);
}

-(float) limitRotationRate
{
    SensorRange Range;
    sensor->GetRange(&Range);
    
    return Range.MaxRotationRate;
}

-(void) setLimitRotationRate: (float)rotationRate
{
    SensorRange Range = SensorRange(self.limitAcceleration, rotationRate, self.limitMagneticField);
    sensor->SetRange(Range, true);
}

-(float) limitMagneticField
{
    SensorRange Range;
    sensor->GetRange(&Range);
    
    return Range.MaxMagneticField;
}

-(void) setLimitMagneticField: (float)magneticField
{
    SensorRange Range = SensorRange(self.limitAcceleration, self.limitRotationRate, magneticField);
    sensor->SetRange(Range, true);
}

-(_Bool) setRangeAcceleration: (float)acceleration RotationRate: (float)rotationRate MagneticField: (float)magneticField ShouldWait: (_Bool)wait
{
    SensorRange Range = SensorRange(acceleration, rotationRate, magneticField);
    return sensor->SetRange(Range, wait);
}

@end

#pragma mark - SensorInfo
@implementation OVRSensor (SensorInfo)

-(uint16_t) vendorID
{
    SensorInfo Info;
    sensor->GetDeviceInfo(&Info);
    
    return Info.VendorId;
}

-(uint16_t) productID
{
    SensorInfo Info;
    sensor->GetDeviceInfo(&Info);
    
    return Info.ProductId;
}

-(float) maxAcceleration
{
    SensorInfo Info;
    sensor->GetDeviceInfo(&Info);
    
    return Info.MaxRanges.MaxAcceleration;
}

-(float) maxRotationRate
{
    SensorInfo Info;
    sensor->GetDeviceInfo(&Info);
    
    return Info.MaxRanges.MaxRotationRate;
}

-(float) maxMagneticField
{
    SensorInfo Info;
    sensor->GetDeviceInfo(&Info);
    
    return Info.MaxRanges.MaxMagneticField;
}

-(NSString*) serialNumber
{
    SensorInfo Info;
    sensor->GetDeviceInfo(&Info);
    
    return [NSString stringWithUTF8String: Info.SerialNumber];
}

@end

#pragma mark - SensorFusion
@implementation OVRSensor (SensorFusion)

#pragma mark State Query
-(GLKQuaternion) orientation
{
    const Quatf Quat = sensorFusion.GetOrientation();
    return GLKQuaternionMake(Quat.x, Quat.y, Quat.z, Quat.w);
}

-(GLKQuaternion) predictedOrientation
{
    const Quatf Quat = sensorFusion.GetPredictedOrientation();
    return GLKQuaternionMake(Quat.x, Quat.y, Quat.z, Quat.w);
}

-(GLKVector3) acceleration
{
    const Vector3f Accel = sensorFusion.GetAcceleration();
    return GLKVector3Make(Accel.x, Accel.y, Accel.z);
}

-(GLKVector3) angularVelocity
{
    const Vector3f AngVel = sensorFusion.GetAngularVelocity();
    return GLKVector3Make(AngVel.x, AngVel.y, AngVel.z);
}

-(GLKVector3) magnetometer
{
    const Vector3f Mag = sensorFusion.GetMagnetometer();
    return GLKVector3Make(Mag.x, Mag.y, Mag.z);
}

-(GLKVector3) calibratedMagnetometer
{
    const Vector3f CalMag = sensorFusion.GetCalibratedMagnetometer();
    return GLKVector3Make(CalMag.x, CalMag.y, CalMag.z);
}

-(GLKQuaternion) predictOrientationAtInterval: (float)interval
{
    const Quatf Quat = sensorFusion.GetPredictedOrientation(interval);
    return GLKQuaternionMake(Quat.x, Quat.y, Quat.z, Quat.w);
}

-(void) resetOrientation
{
    sensorFusion.Reset();
}

#pragma mark Configuration
-(bool) useMotionTracking
{
    return sensorFusion.IsMotionTrackingEnabled();
}

-(void) setUseMotionTracking: (bool)useMotionTracking
{
    sensorFusion.EnableMotionTracking(useMotionTracking);
}

#pragma mark Prediction Control
-(float) predictionDelta
{
    return sensorFusion.GetPredictionDelta();
}

-(void) setPredictionDelta: (float)predictionDelta
{
    sensorFusion.SetPrediction(predictionDelta);
}

-(bool) usePrediction
{
    return sensorFusion.IsPredictionEnabled();
}

-(void) setUsePrediction: (bool)usePrediction
{
    sensorFusion.SetPredictionEnabled(usePrediction);
}

#pragma mark Accelerometer/Gravity Correction Control
-(bool) useGravity
{
    return sensorFusion.IsGravityEnabled();
}

-(void) setUseGravity: (bool)useGravity
{
    sensorFusion.SetGravityEnabled(useGravity);
}

-(float) accelerometerGain
{
    return sensorFusion.GetAccelGain();
}

-(void) setAccelerometerGain: (float)accelerometerGain
{
    sensorFusion.SetAccelGain(accelerometerGain);
}

#pragma mark Magnetometer and Yaw Drift Correction Control
-(bool) useYawCorrection
{
    return sensorFusion.IsYawCorrectionEnabled();
}

-(void) setUseYawCorrection: (bool)useYawCorrection
{
    sensorFusion.SetYawCorrectionEnabled(useYawCorrection);
}

-(GLKMatrix4) magnetometerCalibration
{
    const Matrix4f MagCalMat = sensorFusion.GetMagCalibration();
    return GLKMatrix4MakeWithArray((float*)MagCalMat.M);
}

-(void) setMagnetometerCalibration: (GLKMatrix4)magnetometerCalibration
{
    const Matrix4f MagCalMat = Matrix4f(magnetometerCalibration.m00, magnetometerCalibration.m01, magnetometerCalibration.m02, magnetometerCalibration.m03,
                                        magnetometerCalibration.m10, magnetometerCalibration.m11, magnetometerCalibration.m12, magnetometerCalibration.m13,
                                        magnetometerCalibration.m20, magnetometerCalibration.m21, magnetometerCalibration.m22, magnetometerCalibration.m23,
                                        magnetometerCalibration.m30, magnetometerCalibration.m31, magnetometerCalibration.m32, magnetometerCalibration.m33);
    sensorFusion.SetMagCalibration(MagCalMat);
}

-(time_t) magnetometerCalibrationTime
{
    return sensorFusion.GetMagCalibrationTime();
}

-(bool) hasMagnetometerCalibration
{
    return sensorFusion.HasMagCalibration();
}

-(_Bool) saveMagnetometerCalibrationWithName: (NSString*)name
{
    return sensorFusion.SaveMagCalibration([name UTF8String]);
}

-(_Bool) loadMagnetometerCalibrationWithName: (NSString*)name
{
    return sensorFusion.LoadMagCalibration([name UTF8String]);
}

-(void) clearMagnetometerCalibration
{
    sensorFusion.ClearMagCalibration();
}

-(void) clearMagnetometerReferences
{
    sensorFusion.ClearMagReferences();
}

-(GLKVector3) calibratedMagnetometerValue: (GLKVector3)rawMag
{
    const Vector3f RawMag = Vector3f(rawMag.x, rawMag.y, rawMag.z);
    const Vector3f CalMagVal = sensorFusion.GetCalibratedMagValue(RawMag);
    
    return GLKVector3Make(CalMagVal.x, CalMagVal.y, CalMagVal.z);
}

#pragma mark - Euler Angles
-(float) yaw
{
    float Yaw, Pitch, Roll;
    Quatf Quat = sensorFusion.GetPredictedOrientation();
    
    Quat.GetEulerAngles<Axis_Y, Axis_X, Axis_Z>(&Yaw, &Pitch, &Roll);
    
    return Yaw;
}

-(float) pitch
{
    float Yaw, Pitch, Roll;
    Quatf Quat = sensorFusion.GetPredictedOrientation();
    
    Quat.GetEulerAngles<Axis_Y, Axis_X, Axis_Z>(&Yaw, &Pitch, &Roll);
    
    return Pitch;
}

-(float) roll
{
    float Yaw, Pitch, Roll;
    Quatf Quat = sensorFusion.GetPredictedOrientation();
    
    Quat.GetEulerAngles<Axis_Y, Axis_X, Axis_Z>(&Yaw, &Pitch, &Roll);
    
    return Roll;
}

-(void) getYaw: (float*)yaw Pitch: (float*)pitch Roll: (float*)roll
{
    float Yaw, Pitch, Roll;
    Quatf Quat = sensorFusion.GetPredictedOrientation();
    
    Quat.GetEulerAngles<Axis_Y, Axis_X, Axis_Z>(&Yaw, &Pitch, &Roll);
    
    if (yaw) *yaw = Yaw;
    if (pitch) *pitch = Pitch;
    if (roll) *roll = Roll;
}

-(NSString*) descriptionOfSensorData
{
    NSMutableString *Description = [NSMutableString stringWithFormat: @"<%@: %p>", [self class], self];
    
    [Description appendString: @"\n{"];
    
    float EulerAngle[3];
    [self getYaw: EulerAngle Pitch: &EulerAngle[1] Roll: &EulerAngle[2]];
    [Description appendFormat: @"\n\tYaw: %@  Pitch: %@  Roll: %@", @(EulerAngle[0]), @(EulerAngle[1]), @(EulerAngle[2])];
    [Description appendFormat: @"\n\tOrientation: %@", NSStringFromGLKQuaternion(self.orientation)];
    [Description appendFormat: @"\n\tPredicted Orientation: %@", NSStringFromGLKQuaternion(self.predictedOrientation)];
    [Description appendFormat: @"\n\tAcceleration: %@", NSStringFromGLKVector3(self.acceleration)];
    [Description appendFormat: @"\n\tAngular Velocity: %@", NSStringFromGLKVector3(self.angularVelocity)];
    [Description appendFormat: @"\n\tMagnetometer: %@", NSStringFromGLKVector3(self.magnetometer)];
    [Description appendFormat: @"\n\tCalibrated Magnetometer: %@", NSStringFromGLKVector3(self.calibratedMagnetometer)];
    
    [Description appendString: @"\n}"];
    
    return Description;
}

@end
