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

#import "OVRProfile_Private.h"
#import "OVRProfileRiftDK1.h"
#import "OVRProfileRiftDKHD.h"

using namespace OVR;

const OVRGender OVRGenderUnspecified = Profile::Gender_Unspecified;
const OVRGender OVRGenderMale = Profile::Gender_Male;
const OVRGender OVRGenderFemale = Profile::Gender_Female;

const OVREyeCupType OVREyeCupA = EyeCup_A;
const OVREyeCupType OVREyeCupB = EyeCup_B;
const OVREyeCupType OVREyeCupC = EyeCup_C;


@implementation OVRProfile
@synthesize profile;

-(instancetype) initWithProfile: (Ptr<Profile>)prof
{
    if (prof == nullptr)
    {
        [self release];
        return nil;
    }
    
    Class ShouldBeInstanceOfClass = Nil;
    switch (prof->Type)
    {
        case Profile_Unknown:
            ShouldBeInstanceOfClass = [OVRProfile class];
            break;
            
        case Profile_GenericHMD:
            ShouldBeInstanceOfClass = [OVRProfileHMD class];
            break;
            
        case Profile_RiftDK1:
            ShouldBeInstanceOfClass = [OVRProfileRiftDK1 class];
            break;
            
        case Profile_RiftDKHD:
            ShouldBeInstanceOfClass = [OVRProfileRiftDKHD class];
            break;
    }
    
    if ([self class] != ShouldBeInstanceOfClass)
    {
        [self release];
        return [[ShouldBeInstanceOfClass alloc] initWithProfile: prof];
    }
    
    if ((self = [super init]))
    {
        profile = prof;
    }
    
    return self;
}

-(instancetype) initWithName: (NSString*)profName
{
    if ((self = [super init]))
    {
        
    }
    
    return self;
}

-(NSString*) description
{
    NSMutableString *Description = [NSMutableString stringWithFormat: @"<%@: %p>", [self class], self];
    
    [Description appendString: @"\n{"];
    
    [Description appendFormat: @"\n\tName: \"%@\"", self.name];
    [Description appendFormat: @"\n\tGender: %@  Height: %@", (NSString*[]){
        @"Unspecified",
        @"Male",
        @"Female"
    }[self.userGender], @(self.userHeight)];
    [Description appendFormat: @"\n\tInterpupillary Distance: %@", @(self.interpupillaryDistance)];
    
    [Description appendString: @"\n}"];
    
    return Description;
}

-(void) dealloc
{
    if (profile != nullptr) profile = nullptr;
    
    [super dealloc];
}

#pragma mark - Profile Functionality
-(NSString*) name
{
    return [NSString stringWithUTF8String: profile->Name];
}

-(OVRGender) userGender
{
    return profile->GetGender();
}

-(void) setUserGender: (OVRGender)userGender
{
    profile->SetGender((Profile::GenderType)userGender);
}

-(float) userHeight
{
    return profile->GetPlayerHeight();
}

-(void) setUserHeight: (float)userHeight
{
    profile->SetPlayerHeight(userHeight);
}

-(float) interpupillaryDistance
{
    return profile->GetIPD();
}

-(void) setInterpupillaryDistance: (float)interpupillaryDistance
{
    profile->SetIPD(interpupillaryDistance);
}

@end

#pragma mark - HMDProfile
@implementation OVRProfileHMD

-(NSString*) description
{
    NSMutableString *Description = (NSMutableString*)[super description];
    
    [Description deleteCharactersInRange: NSMakeRange([Description length] - 2, 2)];
    
    [Description appendFormat: @"\n\tLeft Eye Outer Extent: %@  Inner Extent: %@", @(self.leftEyeOuterExtent), @(self.leftEyeInnerExtent)];
    [Description appendFormat: @"\n\tRight Eye Inner Extent: %@  Outer Extent: %@", @(self.rightEyeInnerExtent), @(self.rightEyeOuterExtent)];
    
    [Description appendString: @"\n}"];
    
    return Description;
}

-(int) leftEyeOuterExtent
{
    return ((HMDProfile*)(profile.GetPtr()))->GetLL();
}

-(void) setLeftEyeOuterExtent: (int)leftEyeOuterExtent
{
    ((HMDProfile*)(profile.GetPtr()))->SetLL(leftEyeOuterExtent);
}

-(int) leftEyeInnerExtent
{
    return ((HMDProfile*)(profile.GetPtr()))->GetLR();
}

-(void) setLeftEyeInnerExtent: (int)leftEyeInnerExtent
{
    ((HMDProfile*)(profile.GetPtr()))->SetLR(leftEyeInnerExtent);
}

-(int) rightEyeOuterExtent
{
    return ((HMDProfile*)(profile.GetPtr()))->GetRR();
}

-(void) setRightEyeOuterExtent: (int)rightEyeOuterExtent
{
    ((HMDProfile*)(profile.GetPtr()))->SetRR(rightEyeOuterExtent);
}

-(int) rightEyeInnerExtent
{
    return ((HMDProfile*)(profile.GetPtr()))->GetRL();
}

-(void) setRightEyeInnerExtent: (int)rightEyeInnerExtent
{
    ((HMDProfile*)(profile.GetPtr()))->SetRL(rightEyeInnerExtent);
}

@end