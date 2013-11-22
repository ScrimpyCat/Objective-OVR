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

@class OVRSensor, OVRProfile;
@interface OVRDevice : OVRGenericDevice

@property (readonly) OVRSensor *sensor;
@property (readonly) OVRProfile *profile;
@property (copy) NSString *profileName;

+(NSNotificationCenter*) notificationCenter;
+(NSArray*) devices;

@end

@interface OVRDevice (HMDInfo)

@property (readonly) CGSize resolution;
@property (readonly) CGSize screenSize;
@property (readonly) float verticalScreenCenter;
@property (readonly) float eyeToScreenDistance;
@property (readonly) float lensSeparationDistance;
@property (readonly) float interpupillaryDistance;
@property (readonly) GLKVector4 distortionK;
@property (readonly) GLKVector4 chromaAbCorrection;
@property (readonly) CGPoint screenPosition;
@property (readonly) NSString *displayDeviceName;
@property (readonly) CGDirectDisplayID displayID;

@end

@interface NSView (OVRDevice)

-(BOOL) enterFullScreenModeInOVRDevice: (OVRDevice*)device withOptions: (NSDictionary*)options;

@end


extern NSString * const OVRDeviceManagerAddedNotification;
extern NSString * const OVRDeviceHMDAddedNotification;
extern NSString * const OVRDeviceSensorAddedNotification;
extern NSString * const OVRDeviceLatencyTesterAddedNotification;
extern NSString * const OVRDeviceManagerRemovedNotification;
extern NSString * const OVRDeviceHMDRemovedNotification;
extern NSString * const OVRDeviceSensorRemovedNotification;
extern NSString * const OVRDeviceLatencyTesterRemovedNotification;
