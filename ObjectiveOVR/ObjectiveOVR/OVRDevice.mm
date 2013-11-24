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

#import "OVRDevice.h"
#import <dispatch/dispatch.h>
#import "LibOVR/Include/OVRVersion.h"
#import "LibOVR/Include/OVR.h"
#import "OVRGenericDevice_Private.h"
#import "OVRSensor_Private.h"
#import "OVRProfile_Private.h"


NSString * const OVRDeviceManagerAddedNotification = @"OVRDeviceManagerAdded";
NSString * const OVRDeviceHMDAddedNotification = @"OVRDeviceHMDAdded";
NSString * const OVRDeviceSensorAddedNotification = @"OVRDeviceSensorAdded";
NSString * const OVRDeviceLatencyTesterAddedNotification = @"OVRDeviceLatencyTesterAdded";
NSString * const OVRDeviceManagerRemovedNotification = @"OVRDeviceManagerRemoved";
NSString * const OVRDeviceHMDRemovedNotification = @"OVRDeviceHMDRemoved";
NSString * const OVRDeviceSensorRemovedNotification = @"OVRDeviceSensorRemoved";
NSString * const OVRDeviceLatencyTesterRemovedNotification = @"OVRDeviceLatencyTesterRemoved";


using namespace OVR;

@interface OVRDevice ()

@property (readonly) Ptr<HMDDevice> hmd;

+(void) destroy;
+(Ptr<DeviceManager>) deviceManager;
+(instancetype) device: (Ptr<HMDDevice>)hmd;

-(instancetype) initWithHMDDevice: (Ptr<HMDDevice>)device;

@end


@implementation OVRDevice
@synthesize hmd, sensor;

static NSMutableArray *Devices = nil;
+(instancetype) device: (Ptr<HMDDevice>)hmd
{
    if (!Devices) Devices = [NSMutableArray new];
    
    for (OVRDevice *device in Devices)
    {
        if (device.hmd == hmd) return [[device retain] autorelease];
    }
    
    
    OVRDevice *Device = [[[OVRDevice alloc] initWithHMDDevice: hmd] autorelease];
    if (Device) [Devices addObject: Device];
    
    
    return Device;
}

static Ptr<DeviceManager> Manager = nullptr;
class Notifier : MessageHandler
{
public:
    void OnMessage(const Message &Msg) {
        @autoreleasepool {
            const DeviceHandle *Handle = &((MessageDeviceStatus&)Msg).Handle;
            const DeviceType HandleType = Handle->GetType();
            switch (Msg.Type)
            {
                case Message_DeviceAdded:
                    if (HandleType == Device_HMD)
                    {
                        
                    }
                    
                    else if (HandleType == Device_Sensor)
                    {
                        
                    }
                    
                    else if (HandleType == Device_LatencyTester)
                    {
                        
                    }
                    
                    else if (HandleType == Device_Manager)
                    {
                        
                    }
                    break;
                    
                case Message_DeviceRemoved:
                    if (HandleType == Device_HMD)
                    {
                        
                    }
                    
                    else if (HandleType == Device_Sensor)
                    {
                        
                    }
                    
                    else if (HandleType == Device_LatencyTester)
                    {
                        
                    }
                    
                    else if (HandleType == Device_Manager)
                    {
                        
                    }
                    break;
                    
                default:
                    break;
            }
        }
    }
};

static void DestroySystem(void)
{
    [OVRDevice destroy];
}

static Notifier MessageNotifier;
static _Bool Initialized = NO;
+(void) initialize
{
    if (!Initialized)
    {
        static dispatch_once_t onceToken;
        dispatch_once(&onceToken, ^{
            Initialized = YES;
            System::Init(Log::ConfigureDefaultLog(LogMask_All));
            
            Manager = *DeviceManager::Create();
            
            DeviceEnumerator<HMDDevice> DeviceList = Manager->EnumerateDevices<HMDDevice>();
            do
            {
                Ptr<HMDDevice> Device = DeviceList.CreateDevice();
                [OVRDevice device: Device];
                if (Device != nullptr) Device->Release();
            } while (DeviceList.Next());
            
            
            Manager->SetMessageHandler((MessageHandler*)&MessageNotifier);
            atexit(DestroySystem);
        });
    }
}

+(void) destroy
{
    if (Initialized)
    {
        static dispatch_once_t onceToken;
        dispatch_once(&onceToken, ^{
            Initialized = NO;
            [Devices release]; Devices = nil;
            
            Manager = nullptr;
            System::Destroy();
        });
    }
}

+(NSNotificationCenter*) notificationCenter
{
    return [NSNotificationCenter defaultCenter];
}

+(Ptr<DeviceManager>) deviceManager
{
    return Manager;
}

+(NSArray*) devices
{
    return [[Devices copy] autorelease];
}

-(instancetype) initWithHMDDevice: (Ptr<HMDDevice>)device
{
    if (device == nullptr)
    {
        [self release];
        return nil;
    }
    
    if ((self = [super init]))
    {
        hmd = device;
        
        Ptr<SensorDevice> Sensor = device->GetSensor();
        if (Sensor != nullptr)
        {
            sensor = [[OVRSensor alloc] initWithSensorDevice: Sensor];
            Sensor->Release();
        }
    }
    
    return self;
}

-(NSString*) description
{
    NSMutableString *Description = [NSMutableString stringWithFormat: @"<%@: %p>", [self class], self];
    
    [Description appendString: @"\n{"];
    
    [Description appendFormat: @"\n\tManufacturer: \"%@\"  Product: \"%@\"  Version: %u", self.manufacturer, self.productName, self.version];
    [Description appendFormat: @"\n\tDisplay Name: \"%@\"  ID: %u", self.displayDeviceName, self.displayID];
    [Description appendFormat: @"\n\tResolution: %@  Screen Position: %@ Screen Size: %@", NSStringFromSize(self.resolution), NSStringFromPoint(self.screenPosition), NSStringFromSize(self.screenSize)];
    [Description appendFormat: @"\n\tVertical Screen Center: %@", @(self.verticalScreenCenter)];
    [Description appendFormat: @"\n\tEye To Screen Distance: %@  Lens Separation Distance: %@  Interpupillary Distance: %@", @(self.eyeToScreenDistance), @(self.lensSeparationDistance), @(self.interpupillaryDistance)];
    [Description appendFormat: @"\n\tDistortion K: %@", NSStringFromGLKVector4(self.distortionK)];
    [Description appendFormat: @"\n\tChromatic Aberration Correction: %@", NSStringFromGLKVector4(self.chromaAbCorrection)];
    [Description appendFormat: @"\n\tProfile: %@", [[self.profile description] stringByReplacingOccurrencesOfString: @"\n" withString: @"\n\t"]];
    [Description appendFormat: @"\n\tSensor: %@", [[sensor description] stringByReplacingOccurrencesOfString: @"\n" withString: @"\n\t"]];
    
    [Description appendString: @"\n}"];
    
    return Description;
}

-(Ptr<DeviceBase>) device
{
    return hmd;
}

-(OVRProfile*) profile
{
    return [[[OVRProfile alloc] initWithProfile: hmd->GetProfile()] autorelease];
}

-(NSString*) profileName
{
    return [NSString stringWithUTF8String: hmd->GetProfileName()];
}

-(void) setProfileName: (NSString*)profileName
{
    hmd->SetProfileName([profileName UTF8String]);
}

-(_Bool) isDisconnected
{
    return hmd->IsDisconnected();
}

-(void) dealloc
{
    [sensor release]; sensor = nil;
    
    if (hmd != nullptr) hmd = nullptr;
    
    [super dealloc];
}

@end

#pragma mark - HMDInfo
@implementation OVRDevice (HMDInfo)

-(CGSize) resolution
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return CGSizeMake(Info.HResolution, Info.VResolution);
}

-(CGSize) screenSize
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return CGSizeMake(Info.HScreenSize, Info.VScreenSize);
}

-(float) verticalScreenCenter
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return Info.VScreenCenter;
}

-(float)eyeToScreenDistance
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return Info.EyeToScreenDistance;
}

-(float) lensSeparationDistance
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return Info.LensSeparationDistance;
}

-(float) interpupillaryDistance
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return Info.InterpupillaryDistance;
}

-(GLKVector4) distortionK
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return GLKVector4Make(Info.DistortionK[0], Info.DistortionK[1], Info.DistortionK[2], Info.DistortionK[3]);
}

-(GLKVector4) chromaAbCorrection
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return GLKVector4Make(Info.ChromaAbCorrection[0], Info.ChromaAbCorrection[1], Info.ChromaAbCorrection[2], Info.ChromaAbCorrection[3]);
}

-(CGPoint) screenPosition
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return CGPointMake(Info.DesktopX, Info.DesktopY);
}

-(NSString*) displayDeviceName
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return [NSString stringWithUTF8String: Info.DisplayDeviceName];
}

-(CGDirectDisplayID) displayID
{
    HMDInfo Info;
    hmd->GetDeviceInfo(&Info);
    
    return (CGDirectDisplayID)Info.DisplayId;
}

@end


#pragma mark - NSView
@implementation NSView (OVRDevice)

-(BOOL) enterFullScreenModeInOVRDevice: (OVRDevice*)device withOptions: (NSDictionary*)options;
{
    for (NSScreen *Screen in [NSScreen screens])
    {
        CGDirectDisplayID DisplayID = [[[Screen deviceDescription] objectForKey: @"NSScreenNumber"] unsignedIntValue];
        if (DisplayID == device.displayID)
        {
            return [self enterFullScreenMode: Screen withOptions: options];
        }
    }
    
    return NO;
}

@end
