// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
#pragma once
// UE
#include "ChaosVehicleWheel.h"
#include "CoreMinimal.h"
#include "Misc/Paths.h"
#include "Templates/SharedPointer.h"

// RapyutaSimulationPlugins
#include "Core/RRCoreUtils.h"
#include "Core/RRMathUtils.h"
#include "Core/RRTypeUtils.h"

#include "RRRobotStructs.generated.h"
// (NOTE) To avoid cyclic inclusion, except for utils, please DO NOT include any other component header files here!

class ARRSkeletalRobot;

UENUM(meta = (Bitflags))
enum class ERRUEComponentType : uint8
{
    NONE = 0x00,
    DIFF_DRIVE = 0x01,
    WHEEL_DRIVE = 0x02,
    ARTICULATION_DRIVE = 0x04
};
ENUM_CLASS_FLAGS(ERRUEComponentType);

UENUM()
enum class ERRRobotDescriptionType : uint8
{
    NONE,
    URDF,
    SDF
};

/**
 * @brief
 *
 */
UENUM()
enum class ERRRobotStatus : uint8
{
    INVALID,             //! Invalid type, or not loaded successfully
    CREATING,            //! Being built up (loading the meshses, building the structure, etc.)
    IDLE,                //! Idling, ready for command order
    ORDER_PROCESSING,    //! Received command and in middle of processing
    MOVING,              //! On the move
    ERROR,               //! Having some internal errors
    TOTAL
};

UENUM(meta = (Bitflags))
enum class ERRRobotGeometryType : uint8
{
    NONE = 0x00,
    VISUAL = 0x01,
    COLLISION = 0x02,
    INERTIA = 0x04
};
ENUM_CLASS_FLAGS(ERRRobotGeometryType);

UENUM()
enum class ERRSensorType : uint8
{
    NONE,
    CAMERA,
    LIDAR,
    DEPTH,
    IMU,
    TOTAL
};

UENUM()
enum class ERRLidarSensorType : uint8
{
    NONE,
    TWO_D,
    THREE_D,
    TOTAL
};

UENUM()
enum class ERRRobotJointType : uint8
{
    INVALID,
    FIXED,
    FLOATING,
    PRISMATIC,
    REVOLUTE,
    BALL,
    PLANAR,
    CONTINUOUS,
    TOTAL
};

UENUM()
enum class ERRRobotJointStatus : uint8
{
    // Invalid -> Idle: After joint drive params are fully configured
    INVALID,
    IDLE,
    MOVING,
    TOTAL
};

UENUM()
enum class ERRRobotMeshComponentType : uint8
{
    INVALID,
    STATIC_MESH,
    SKELETAL_MESH,
    PROCEDURAL_MESH,
    RUNTIME_MESH,
    TOTAL
};

UENUM()
enum class ERRJointAxisRotation : uint8
{
    NONE = 0x00,
    X = 0x01,
    Y = 0x02,
    Z = 0x04
};

// -----------------------------------------------------------------------------------------------
// [FRRRobotJointDynamicProperties] --
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotJointDynamicProperties
{
    GENERATED_BODY()
    FRRRobotJointDynamicProperties()
    {
    }
    FRRRobotJointDynamicProperties(const FString& InJointName) : JointName(InJointName)
    {
    }
    FRRRobotJointDynamicProperties(float InStiffness, float InDamping, float InP, float InI, float InD)
    {
        SpringStiff = InStiffness;
        Damping = InDamping;
        P = InP;
        I = InI;
        D = InD;
    }
    FRRRobotJointDynamicProperties(const FString& InJointName, float InStiffness, float InDamping)
        : JointName(InJointName), SpringStiff(InStiffness), Damping(InDamping)
    {
    }
    UPROPERTY(VisibleAnywhere)
    FString JointName;
    // For Spring-Damper Control Mode --
    // Stiffness
    UPROPERTY(EditAnywhere,
              Category = "Spring-Damper",
              meta = (ClampMin = "0.0", ClampMax = "1000000000.0", UIMin = "0.0", UIMax = "1000000000.0"))
    float SpringStiff = 0.f;
    // https://docs.unrealengine.com/en-us/Engine/Physics/FrictionRestitutionAndDamping
    UPROPERTY(EditAnywhere,
              Category = "Spring-Damper",
              meta = (ClampMin = "0.0", ClampMax = "1000000000.0", UIMin = "0.0", UIMax = "1000000000.0"))
    float LimitSpringStiff = 0.f;
    // Damping
    UPROPERTY(EditAnywhere,
              Category = "Spring-Damper",
              meta = (ClampMin = "0.0", ClampMax = "500.0", UIMin = "0.0", UIMax = "500.0"))
    float Damping = 0.f;
    UPROPERTY(EditAnywhere,
              Category = "Spring-Damper",
              meta = (ClampMin = "0.0", ClampMax = "500.0", UIMin = "0.0", UIMax = "500.0"))
    float LimitDamping = 0.f;
    UPROPERTY(EditAnywhere,
              Category = "Spring-Damper",
              meta = (ClampMin = "0.0", ClampMax = "1000000000.0", UIMin = "0.0", UIMax = "1000000000.0"))
    float Friction = 0.f;
    UPROPERTY(EditAnywhere,
              Category = "Spring-Damper",
              meta = (ClampMin = "0.0", ClampMax = "1000000000.0", UIMin = "0.0", UIMax = "1000000000.0"))
    float SpringRef = 0.f;
    UPROPERTY(EditAnywhere,
              Category = "Spring-Damper",
              meta = (ClampMin = "0.0", ClampMax = "10000000.0", UIMin = "0.0", UIMax = "10000000.0"))
    float MaxForceLimit = 100000000.f;

    //! Rad(m)/s
    UPROPERTY(EditAnywhere)
    float MaxVelocity = 1000000.f;

    //! Safety Controller
    UPROPERTY(EditAnywhere)
    float KVelocity = 0.f;

    //! PID Controller P Gain
    UPROPERTY(EditAnywhere)
    float P = 0.1f;

    //! PID Controller I Gain
    UPROPERTY(EditAnywhere)
    float I = 0.5f;

    //! PID Controller D Gain
    UPROPERTY(EditAnywhere)
    float D = 0.01f;

    void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- SpringStiff %f LimitSpringStiff %f"), SpringStiff, LimitSpringStiff);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Damping %f LimitDamping %f"), Damping, LimitDamping);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Friction %f SpringRef %f"), Friction, SpringRef);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- MaxForceLimit %f MaxVelocity %f"), MaxForceLimit, MaxVelocity);
    }
};
// [ROBOT JOINT] --
//
/**
 * @brief The FRRRobotJointProperty struct
 *  [rad] for REVOLUTE joint, [m] for Prismatic Joint. (Both URDF and SDF)
 * @sa http://sdformat.org/spec?ver=1.6&elem=joint
 * @sa http://wiki.ros.org/urdf/XML/joint
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotJointProperty
{
    GENERATED_BODY()
public:
    UPROPERTY(VisibleAnywhere)
    FString Name;

    UPROPERTY(VisibleAnywhere)
    ERRRobotJointType Type = ERRRobotJointType::INVALID;

    FORCEINLINE static ERRRobotJointType GetERRRobotJointTypeValueFromString(const FString& InEnumStringValue)
    {
        return static_cast<ERRRobotJointType>(URRTypeUtils::GetEnumValueFromString(TEXT("ERRRobotJointType"), InEnumStringValue));
    }

    FORCEINLINE static FString GetJointTypeName(const ERRRobotJointType InJointType)
    {
        return URRTypeUtils::GetEnumValueAsString(TEXT("ERRRobotJointType"), InJointType);
    }

    UPROPERTY(VisibleAnywhere)
    ERRRobotJointStatus Status = ERRRobotJointStatus::INVALID;

    //! In URDF/SDF: Child Link's relative Location to its Parent
    UPROPERTY(VisibleAnywhere)
    FVector Location = FVector::ZeroVector;

    //! In URDF/SDF: Child Link's relative Rotation to its Parent
    UPROPERTY(VisibleAnywhere)
    FQuat Rotation = FQuat::Identity;

    //! Whether Rotation & Location are absolute or relative
    UPROPERTY(VisibleAnywhere)
    bool bIsTransformRelative = true;

    FTransform GetTransform() const
    {
        return FTransform(Rotation, Location);
    }
    UPROPERTY(VisibleAnywhere)
    FString ParentLinkName;
    UPROPERTY(VisibleAnywhere)
    FString ChildLinkName;
    UPROPERTY(VisibleAnywhere)
    FString MimicJointName;
    UPROPERTY(VisibleAnywhere)
    float MimicMultiplier = 1.0f;
    UPROPERTY(VisibleAnywhere)
    float MimicOffset = 0.0f;
    UPROPERTY(VisibleAnywhere)
    FVector Axis = FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere)
    FVector AxisInParentFrame = FVector::ZeroVector;
    const FVector& GetAxis(bool bIsLocal = false) const
    {
        return bIsLocal ? Axis : AxisInParentFrame;
    }

    //! [rad] for REVOLUTE joint, [m] for Prismatic Joint. (Both URDF and SDF)
    UPROPERTY(VisibleAnywhere)
    float LowerLimit = 0.f;

    //! [rad] for REVOLUTE joint, [m] for Prismatic Joint. (Both URDF and SDF)
    UPROPERTY(VisibleAnywhere)
    float UpperLimit = 0.f;

    UPROPERTY(EditAnywhere)
    FRRRobotJointDynamicProperties DynamicParams;
    bool operator==(const FRRRobotJointProperty& JointProp)
    {
        return Name.Equals(JointProp.Name);
    }
    bool operator==(const FString& String)
    {
        return Name.Equals(String);
    }
    bool IsValid() const
    {
        return (false == Name.IsEmpty()) && (ERRRobotJointType::INVALID != Type);
    }
    ELinearConstraintMotion GetLinearConstraintMotion() const
    {
        switch (Type)
        {
            case ERRRobotJointType::INVALID:
            case ERRRobotJointType::FIXED:
            case ERRRobotJointType::REVOLUTE:
            case ERRRobotJointType::CONTINUOUS:
            case ERRRobotJointType::BALL:
                return LCM_Locked;

            case ERRRobotJointType::PRISMATIC:
                return LCM_Limited;

            case ERRRobotJointType::PLANAR:
            case ERRRobotJointType::FLOATING:
                return LCM_Free;

            default:
                return LCM_Locked;
        }
    }
    EAngularConstraintMotion GetAngularConstraintMotion() const
    {
        switch (Type)
        {
            case ERRRobotJointType::INVALID:
            case ERRRobotJointType::FIXED:
            case ERRRobotJointType::PRISMATIC:
                return ACM_Locked;

            case ERRRobotJointType::REVOLUTE:
            {
                return (UpperLimit > UE_TWO_PI) ? ACM_Free : ACM_Limited;
            }

            case ERRRobotJointType::FLOATING:
            case ERRRobotJointType::BALL:
            case ERRRobotJointType::PLANAR:
            case ERRRobotJointType::CONTINUOUS:
                return ACM_Free;
            default:
                return ACM_Locked;
        }
    }

    /**
     * @brief Get the Joint Axis object
     * @param bIsGlobal
     * @param InLinkQuat
     * @return FVector
     */
    FVector GetJointAxis(bool bIsGlobal = false, const FQuat& InLinkQuat = FQuat::Identity) const
    {
        // https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/robot/robot_joint.cpp#L425
        return bIsGlobal ? InLinkQuat.RotateVector(Axis) : Axis;
    }

    /**
     * @brief
     * Twist(X), Swing1(Z), Swing2(Y)
     * @sa https://forums.unrealengine.com/t/can-someone-explain-swing-1-2-and-twist/103357/3
     * @sa https://documentation.help/NVIDIA-PhysX-SDK-Guide/Joints.html
     * @sa https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Joints.html
     * FPhysicsInterface_PhysX::GetCurrentSwing2
     * @param InAxis
     * @return true
     * @return false
     */
    bool IsTwisting(const FVector& InAxis) const
    {
        return FMath::IsNearlyEqual(FMath::Abs(InAxis.X), 1.f, 1.e-3f);
    }

    /**
     * @brief
     * Twist(X), Swing1(Z), Swing2(Y)
     * @sa https://forums.unrealengine.com/t/can-someone-explain-swing-1-2-and-twist/103357/3
     * @sa https://documentation.help/NVIDIA-PhysX-SDK-Guide/Joints.html
     * @sa https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Joints.html
     * FPhysicsInterface_PhysX::GetCurrentSwing2
     * @param InAxis
     * @return true
     * @return false
     */
    bool IsSwing1(const FVector& InAxis) const
    {
        return FMath::IsNearlyEqual(FMath::Abs(InAxis.Z), 1.f, 1.e-3f);
    }

    /**
     * @brief
     * Twist(X), Swing1(Z), Swing2(Y)
     * @sa https://forums.unrealengine.com/t/can-someone-explain-swing-1-2-and-twist/103357/3
     * @sa https://documentation.help/NVIDIA-PhysX-SDK-Guide/Joints.html
     * @sa https://docs.nvidia.com/gameworks/content/gameworkslibrary/physx/guide/Manual/Joints.html
     * FPhysicsInterface_PhysX::GetCurrentSwing2
     * @param InAxis
     * @return true
     * @return false
     */
    bool IsSwing2(const FVector& InAxis) const
    {
        return FMath::IsNearlyEqual(FMath::Abs(InAxis.Y), 1.f, 1.e-3f);
    }
    void GetTwistSwing(bool& bOutTwist, bool& bOutSwing1, bool& bOutSwing2) const
    {
        bOutTwist = IsTwisting(AxisInParentFrame);
        bOutSwing1 = IsSwing1(AxisInParentFrame);
        bOutSwing2 = IsSwing2(AxisInParentFrame);
    }
    void GetLinearXYZ(bool& bOutLinearX, bool& bOutLinearY, bool& bOutLinearZ) const
    {
        bOutLinearX = FMath::IsNearlyEqual(FMath::Abs(AxisInParentFrame.X), 1.f, 1.e-3f);
        bOutLinearY = FMath::IsNearlyEqual(FMath::Abs(AxisInParentFrame.Y), 1.f, 1.e-3f);
        bOutLinearZ = FMath::IsNearlyEqual(FMath::Abs(AxisInParentFrame.Z), 1.f, 1.e-3f);
    }

    bool IsValidJointPosition(float InJointPos) const
    {
        return (LowerLimit <= UpperLimit) ? (InJointPos >= LowerLimit) && (InJointPos <= UpperLimit) : false;
    }
    bool IsValidJointVelocity(float InJointVelocity) const
    {
        return (DynamicParams.MaxVelocity > 0.f) ? (FMath::Abs(InJointVelocity) <= DynamicParams.MaxVelocity)
                                                 : (InJointVelocity != 0.f);
    }
    void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp,
                         Warning,
                         TEXT("Joint Name: %s - Type %s"),
                         *Name,
                         *URRTypeUtils::GetEnumValueAsString(TEXT("ERRRobotJointType"), Type));
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Location: %s"), *Location.ToString());
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Rotation: %s - %s"), *Rotation.ToString(), *FRotator(Rotation).ToString());
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- ParentLinkName: %s"), *ParentLinkName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- ChildLinkName: %s"), *ChildLinkName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Axis: %s"), *Axis.ToString());
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- MimicJointName: %s"), *MimicJointName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- MimicMultiplier: %f"), MimicMultiplier);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- MimicOffset: %f"), MimicOffset);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- LowerLimit: %f"), LowerLimit);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- UpperLimit: %f"), UpperLimit);
        DynamicParams.PrintSelf();
    }
};

USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotJointValue
{
    GENERATED_BODY()
    //! [rad] for REVOLUTE joint, [m] for Prismatic Joint
    UPROPERTY(EditAnywhere, meta = (ToopTip = "rad or m", ClampMin = "-100.0", ClampMax = "100.0", UIMin = "-100", UIMax = "100.0"))
    double Position = 0;
    UPROPERTY(EditAnywhere, meta = (ToopTip = "m/s", ClampMin = "-100.0", ClampMax = "100.0", UIMin = "-100.0", UIMax = "100.0"))
    double LinearVel = 0;
    UPROPERTY(EditAnywhere, meta = (ToopTip = "rad/s", ClampMin = "-10.0", ClampMax = "10.0", UIMin = "-10.0", UIMax = "10.0"))
    double AngularVel = 0;

    FString ToString() const
    {
        return FString::Printf(TEXT("Pos: %lf LinearVel: %lf AngularVel: %lf"), Position, LinearVel, AngularVel);
    }

    bool operator==(const FRRRobotJointValue& Other) const
    {
        return FMath::IsNearlyEqual(Position, Other.Position, KINDA_SMALL_NUMBER) &&
               FMath::IsNearlyEqual(LinearVel, Other.LinearVel, KINDA_SMALL_NUMBER) &&
               FMath::IsNearlyEqual(AngularVel, Other.AngularVel, KINDA_SMALL_NUMBER);
    }

    bool operator!=(const FRRRobotJointValue& Other) const
    {
        return (false == (*this == Other));
    }
};

// [ROBOT LINK] --
//
// Note: Nested struct does not support USTRUCT(), and so forth also could not affort UPROPERTY members
// A structure representing a SDF/URDF Link
// http://sdformat.org/spec?ver=1.6&elem=link
// http://wiki.ros.org/urdf/XML/link

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotLinkInertia
{
    GENERATED_BODY()
    UPROPERTY(VisibleAnywhere)
    float Mass = 0.f;
    UPROPERTY(VisibleAnywhere)
    FVector Location = FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere)
    FQuat Rotation = FQuat::Identity;
    UPROPERTY(VisibleAnywhere)
    float Ixx = 0.f;
    UPROPERTY(VisibleAnywhere)
    float Ixy = 0.f;
    UPROPERTY(VisibleAnywhere)
    float Ixz = 0.f;
    UPROPERTY(VisibleAnywhere)
    float Iyy = 0.f;
    UPROPERTY(VisibleAnywhere)
    float Iyz = 0.f;
    UPROPERTY(VisibleAnywhere)
    float Izz = 0.f;
};

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotGeometryInfo
{
    GENERATED_BODY()
    UPROPERTY(VisibleAnywhere)
    FString Name;
    UPROPERTY(VisibleAnywhere)
    FString MeshName;

    //! This size is mostly used for Collision info in case of links being Runtime mesh components.
    //! For static mesh based linkes, the collision is already auto-generated by UE.
    UPROPERTY(VisibleAnywhere)
    FVector Size = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere)
    FVector WorldScale = FVector::OneVector;

    //! Owner Link info
    UPROPERTY(VisibleAnywhere)
    ERRShapeType LinkType = ERRShapeType::INVALID;

    UPROPERTY(VisibleAnywhere)
    FString LinkName;
    UPROPERTY(VisibleAnywhere)
    FVector Location = FVector::ZeroVector;
    UPROPERTY(VisibleAnywhere)
    FQuat Rotation = FQuat::Identity;
    FTransform GetTransformOffset() const
    {
        return FTransform(Rotation, Location);
    }
    UPROPERTY(VisibleAnywhere)
    FRRMaterialProperty MaterialInfo;
};

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRSensorLidarInfo
{
    GENERATED_BODY()
    UPROPERTY(VisibleAnywhere)
    ERRLidarSensorType LidarType = ERRLidarSensorType::NONE;

    //! Scan (angle in deg)
    UPROPERTY(VisibleAnywhere)
    int32 NHorSamplesPerScan = 360;

    //! Scan (angle in deg)
    UPROPERTY(VisibleAnywhere)
    int32 NVerSamplesPerScan = 360;

    //! Scan (angle in deg)
    UPROPERTY(VisibleAnywhere)
    float HMinAngle = 0.f;
    UPROPERTY(VisibleAnywhere)
    float HMaxAngle = 0.f;
    UPROPERTY(VisibleAnywhere)
    float HResolution = 0.f;
    UPROPERTY(VisibleAnywhere)
    float VMinAngle = 0.f;
    UPROPERTY(VisibleAnywhere)
    float VMaxAngle = 0.f;
    UPROPERTY(VisibleAnywhere)
    float VResolution = 0.f;

    // Range
    UPROPERTY(VisibleAnywhere)
    float MinRange = 0.f;
    UPROPERTY(VisibleAnywhere)
    float MaxRange = 0.f;
    UPROPERTY(VisibleAnywhere)
    float RangeResolution = 0.f;

    // Noise
    UPROPERTY(VisibleAnywhere)
    FString NoiseTypeName;
    UPROPERTY(VisibleAnywhere)
    double NoiseMean = 0;
    UPROPERTY(VisibleAnywhere)
    double NoiseStdDev = 0;
};

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRSensorProperty
{
    GENERATED_BODY()
    UPROPERTY(VisibleAnywhere)
    FString LinkName;
    UPROPERTY(VisibleAnywhere)
    FString SensorName;
    UPROPERTY(VisibleAnywhere)
    ERRSensorType SensorType = ERRSensorType::NONE;
    UPROPERTY(VisibleAnywhere)
    FRRSensorLidarInfo LidarInfo;
};

USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotLinkProperty
{
    GENERATED_BODY()
public:
    UPROPERTY(VisibleAnywhere)
    FString Name;

    //! Only available in SDF
    UPROPERTY(VisibleAnywhere)
    FString ParentFrameName;

    UPROPERTY(VisibleAnywhere)
    int8 LinkIndex = INDEX_NONE;
    UPROPERTY(VisibleAnywhere)
    int8 ParentLinkIndex = INDEX_NONE;

    //! Relative to the parent link
    //! In URDF: <joint>�s <origin xyz= "" rpy="">
    //! In SDF : <joint> -> <pose> 6 values (xyz + rpy)
    UPROPERTY(VisibleAnywhere)
    FVector Location = FVector::ZeroVector;

    UPROPERTY(VisibleAnywhere)
    FQuat Rotation = FQuat::Identity;
    FTransform GetRelativeTransformToParent() const
    {
        return FTransform(Rotation, Location);
    }
    UPROPERTY(VisibleAnywhere)
    FRRRobotLinkInertia Inertia;

    //! Relative visual offset to the link itself(URDF) or parent frame(SDF)
    //! In URDF: <visual>�s <origin xyz= "" rpy="">
    //! In SDF : <link> -> <pose> 6 values (xyz + rpy)
    UPROPERTY(VisibleAnywhere)
    TArray<FRRRobotGeometryInfo> VisualList;
    FTransform GetVisualOffset(int32 InVisualIndex = 0) const
    {
        return VisualList.IsValidIndex(InVisualIndex) ? VisualList[InVisualIndex].GetTransformOffset() : FTransform::Identity;
    }
    FTransform GetVisualRelativeTransformToParent(int32 InVisualIndex = 0) const
    {
        return GetRelativeTransformToParent() * GetVisualOffset(InVisualIndex);
    }
    UPROPERTY(VisibleAnywhere)
    TArray<FRRRobotGeometryInfo> CollisionList;
    bool operator==(const FRRRobotLinkProperty& LinkProp)
    {
        return Name.Equals(LinkProp.Name);
    }
    bool operator==(const FString& String)
    {
        return Name.Equals(String);
    }
    // Sensor
    UPROPERTY(VisibleAnywhere)
    TArray<FRRSensorProperty> SensorList;
    void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("Link Name: %s -> ParentFrameName: %s"), *Name, *ParentFrameName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Location: %s"), *Location.ToString());
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Rotation: %s - %s"), *Rotation.ToString(), *FRotator(Rotation).ToString());
        // Visuals
        for (const auto& visual : VisualList)
        {
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("Visual: %s"), *visual.Name);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Link: %s"), *visual.LinkName);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- MeshName: %s"), *visual.MeshName);
            UE_LOG_WITH_INFO(
                LogTemp, Display, TEXT("- Size: %s - WorldScale: %s"), *visual.Size.ToString(), *visual.WorldScale.ToString());
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Location: %s"), *visual.Location.ToString());
            UE_LOG_WITH_INFO(
                LogTemp, Display, TEXT("- Rotation: %s - %s"), *visual.Rotation.ToString(), *FRotator(visual.Rotation).ToString());
            visual.MaterialInfo.PrintSelf();
        }
        // Collisions
        for (const auto& collision : CollisionList)
        {
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("Collision: %s"), *collision.Name);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Link: %s"), *collision.LinkName);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- MeshName: %s"), *collision.MeshName);
            UE_LOG_WITH_INFO(LogTemp,
                             Display,
                             TEXT("- Size: %s - WorldScale: %s"),
                             *collision.Size.ToString(),
                             *collision.WorldScale.ToString());
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Location: %s"), *collision.Location.ToString());
            UE_LOG_WITH_INFO(LogTemp,
                             Display,
                             TEXT("- Rotation: %s - %s"),
                             *collision.Rotation.ToString(),
                             *FRotator(collision.Rotation).ToString());
        }
        // Sensors
        for (const auto& sensor : SensorList)
        {
            UE_LOG_WITH_INFO(LogTemp,
                             Warning,
                             TEXT("Sensor: %s %s"),
                             *URRTypeUtils::GetEnumValueAsString(TEXT("ERRSensorType"), sensor.SensorType),
                             *sensor.SensorName);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Link: %s"), *sensor.LinkName);

            const auto& lidarInfo = sensor.LidarInfo;
            UE_LOG_WITH_INFO(LogTemp,
                             Display,
                             TEXT("- Lidar: %s"),
                             *URRTypeUtils::GetEnumValueAsString(TEXT("ERRLidarSensorType"), lidarInfo.LidarType));
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Scan horizontal"));
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ NHorSamplesPerScan: %d"), lidarInfo.NHorSamplesPerScan);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ HMinAngle: %f"), lidarInfo.HMinAngle);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ HMaxAngle: %f"), lidarInfo.HMaxAngle);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ HResolution: %f"), lidarInfo.HResolution);

            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Scan vertical"));
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ NVerSamplesPerScan: %d"), lidarInfo.NVerSamplesPerScan);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ VMinAngle: %f"), lidarInfo.VMinAngle);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ VMaxAngle: %f"), lidarInfo.VMaxAngle);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ VResolution: %f"), lidarInfo.VResolution);

            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Range"));
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ MinRange: %f"), lidarInfo.MinRange);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ MinRange: %f"), lidarInfo.MaxRange);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ Resolution: %f"), lidarInfo.RangeResolution);

            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Noise"));
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ Type: %s"), *lidarInfo.NoiseTypeName);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ Mean: %f"), lidarInfo.NoiseMean);
            UE_LOG_WITH_INFO(LogTemp, Display, TEXT("+ StdDev: %f"), lidarInfo.NoiseStdDev);
        }
    }
};

// Ref: UChaosVehicleWheel

/**
 * @brief Wheel property for the robot, used to store configurations for runtime #UChaosVehicleWheel setup
 * @sa [UChaosVehicleWheel](https://docs.unrealengine.com/5.1/en-US/API/Plugins/ChaosVehicles/UChaosVehicleWheel/)
 */
USTRUCT()
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotWheelProperty
{
    GENERATED_BODY()
public:
    FRRRobotWheelProperty()
    {
    }
    FRRRobotWheelProperty(const FString& InWheelName) : WheelName(InWheelName)
    {
    }
    FRRRobotWheelProperty(FString&& InWheelName) : WheelName(MoveTemp(InWheelName))
    {
    }

    UPROPERTY()
    FString WheelName;

    /** If left undefined then the #bAffectedByEngine value is used, if defined then #bAffectedByEngine is ignored and the
     * differential setup on the vehicle defines which wheels get power from the engine */
    UPROPERTY()
    EAxleType AxleType = EAxleType::Undefined;

    /**
     * If BoneName is specified, offset the wheel from the bone's location.
     * Otherwise this offsets the wheel from the vehicle's origin.
     */
    UPROPERTY()
    FVector Offset = FVector::ZeroVector;

    UPROPERTY()
    float WheelRadius = 0.f;

    UPROPERTY()
    float WheelWidth = 0.f;

    UPROPERTY()
    float CorneringStiffness = 1000.f;

    UPROPERTY()
    float FrictionForceMultiplier = 2.f;

    //! Wheel Lateral Skid Grip Loss, lower number less grip on skid
    UPROPERTY()
    float SideSlipModifier = 1.f;

    //! Wheel Longitudinal Slip Threshold
    UPROPERTY()
    float SlipThreshold = 20.f;

    //! Wheel Lateral Skid Threshold
    UPROPERTY()
    float SkidThreshold = 20.f;

    UPROPERTY()
    bool bAffectedBySteering = false;

    UPROPERTY()
    bool bAffectedByBrake = true;

    UPROPERTY()
    bool bAffectedByHandbrake = false;

    //! Whether engine should power this wheel
    UPROPERTY()
    bool bAffectedByEngine = false;

    //! Advanced Braking System enabled
    UPROPERTY()
    bool bABSEnabled = false;

    //! Straight Line Traction Control Enabled
    UPROPERTY()
    bool bTractionControlEnabled = false;

    //! Determines how the SetDriveTorque/SetBrakeTorque inputs are combined with the internal torques
    UPROPERTY()
    ETorqueCombineMethod ExternalTorqueCombineMethod = ETorqueCombineMethod::None;

    UPROPERTY()
    FRuntimeFloatCurve LateralSlipGraph;

    //! Local body direction in which where suspension forces are applied (typically along -Z-axis)
    UPROPERTY()
    FVector SuspensionAxis = FVector(0.f, 0.f, -1.f);

    //! Vertical offset from where suspension forces are applied (along Z-axis)
    UPROPERTY()
    FVector SuspensionForceOffset = FVector::ZeroVector;

    //! How far the wheel can go above the resting position
    UPROPERTY()
    float SuspensionMaxRaise = 10.f;

    //! How far the wheel can drop below the resting position
    UPROPERTY()
    float SuspensionMaxDrop = 10.f;

    /** Suspension damping, larger value causes the suspension to come to rest faster [range 0 to 1] */
    UPROPERTY()
    float SuspensionDampingRatio = 0.5f;

    /** Smooth suspension [0-off, 10-max] - Warning might cause momentary visual inter-penetration of the wheel against
     * objects/terrain */
    UPROPERTY()
    int8 SuspensionSmoothing = 0;

    /**
     * Amount wheel load effects wheel friction.
     * At 0 wheel friction is completely independent of the loading on the wheel (This is artificial as it always assumes even
     * balance between all wheels) At 1 wheel friction is based on the force pressing wheel into the ground. This is more
     * realistic. Lower value cures lift off over-steer, generally makes vehicle easier to handle under extreme motions.
     */
    UPROPERTY()
    float WheelLoadRatio = 0.5f;

    //! Spring Force (N/m)
    UPROPERTY()
    float SpringRate = 250.0f;

    //! Spring Preload (N/m)
    UPROPERTY()
    float SpringPreload = 50.f;

    //! Anti-roll effect
    UPROPERTY()
    float RollbarScaling = 0.15f;

    //! Whether wheel suspension considers simple, complex, or both
    UPROPERTY()
    ESweepShape SweepShape = ESweepShape::Raycast;

    //! Whether wheel suspension considers simple, complex, or both
    UPROPERTY()
    ESweepType SweepType = ESweepType::SimpleSweep;

    UPROPERTY()
    float MaxSteerAngleDeg = 50.f;

    //! max brake torque for this wheel (Nm)
    UPROPERTY()
    float MaxBrakeTorque = 1500.f;

    /**
     * Max handbrake brake torque for this wheel (Nm). A handbrake should have a stronger brake torque
     * than the brake. This will be ignored for wheels that are not affected by the handbrake.
     */
    UPROPERTY()
    float MaxHandBrakeTorque = 3000.f;

    void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("WheelName: %s Radius %f Width %f"), *WheelName, WheelRadius, WheelWidth);
    }
};

// Node to represent the Robot virtually as a tree
UCLASS()
class RAPYUTASIMULATIONPLUGINS_API URRRobotNode : public UObject
{
    GENERATED_BODY()
public:
    UPROPERTY()
    URRRobotNode* Parent = nullptr;
    UPROPERTY()
    TArray<URRRobotNode*> Children;
    UPROPERTY()
    FRRRobotLinkProperty LinkProp;
    UPROPERTY()
    FRRRobotJointProperty JointProp;
};
// [ROBOT MODEL] --
//
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotModelInfo : public TSharedFromThis<FRRRobotModelInfo, ESPMode::ThreadSafe>
{
public:
    FRRRobotModelInfo(const TArray<FString>& InModelNameList) : ModelNameList(InModelNameList)
    {
    }
    FRRRobotModelInfo()
    {
    }
    // Robot Model Description Type (URDF/SDF)
    ERRRobotDescriptionType ModelDescType = ERRRobotDescriptionType::NONE;
    bool IsURDF() const
    {
        return (ERRRobotDescriptionType::URDF == ModelDescType);
    }
    bool IsSDF() const
    {
        return (ERRRobotDescriptionType::SDF == ModelDescType);
    }
    FString WorldName;
    bool bHasWorldJoint = false;
    // World model: [ModelNameList] >= 1
    // Pure model:  [ModelNameList] == 1 if Single else > 1
    TArray<FString> ModelNameList;
    FString GetModelName() const
    {
        return (ModelNameList.Num() > 0) ? ModelNameList[0] : EMPTY_STR;
    }
    // Meshes are async loaded, & so their sizes
    FVector TotalSize = FVector::ZeroVector;

    int32 GetVisualsNum() const
    {
        // Parent visuals
        int32 meshesNum = 0;
        for (const auto& linkProp : LinkPropList)
        {
            meshesNum += linkProp.VisualList.Num();
        }

        // Child visuals
        for (const auto& modelInfo : ChildModelsInfo)
        {
            meshesNum += modelInfo.GetVisualsNum();
        }
        return meshesNum;
    }

    //! Robot Model Description File (URDF/SDF)
    FString DescriptionFilePath;
    uint32 CreatedNum = 0;
    FString ParentFrameName;
    FTransform RelativeTransform = FTransform::Identity;

    //! UE Component types
    uint32 UEComponentTypeFlags = 0;
    bool IsUEComponentEnabled(const uint32 InTypeMask) const
    {
        return (UEComponentTypeFlags & InTypeMask);
    }
    bool IsUEComponentEnabled(const ERRUEComponentType InTypeMask) const
    {
        return IsUEComponentEnabled(static_cast<uint32>(InTypeMask));
    }
    void SetUEComponentEnabled(const uint32 InTypeMask)
    {
        UEComponentTypeFlags |= InTypeMask;
    }
    void SetUEComponentEnabled(const ERRUEComponentType InTypeMask)
    {
        SetUEComponentEnabled(static_cast<uint32>(InTypeMask));
    }
    bool IsPlainManipulatorModel() const
    {
        return (ERRUEComponentType::ARTICULATION_DRIVE == static_cast<ERRUEComponentType>(UEComponentTypeFlags));
    }
    bool IsPlainWheeledVehicleModel() const
    {
        return (ERRUEComponentType::WHEEL_DRIVE == static_cast<ERRUEComponentType>(UEComponentTypeFlags));
    }

    // Link/Joint list
    FString BaseLinkName;
    TArray<FRRRobotLinkProperty> LinkPropList;
    TArray<FRRRobotJointProperty> JointPropList;

    TArray<FRRRobotModelInfo> ChildModelsInfo;

    /**
     * @brief
    * http://wiki.ros.org/urdf/XML/joint
    * This is only applied to link/joint properties read from URDF, where the joint locates at the origin of the child
    * link. JointProp's loc info is a relative location of child link to its parent link. Since every Link other than the
    * base link has to be connected to another link, the number of joints is fixed.
    * JointInfo's Location --> Child Link Info
    *
    * http://sdformat.org/spec?elem=joint
    * http://sdformat.org/tutorials?tut=pose_frame_semantics
    * For SDF, typically link visual pose (as relative to child link) is defined independently of joint, which is also
    * attached to child link.
     */
    void UpdateLinksLocationFromJoints()
    {
        for (const auto& jointProp : JointPropList)
        {
            for (auto& linkProp : LinkPropList)
            {
                if (linkProp.Name == jointProp.ChildLinkName)
                {
                    linkProp.Location = jointProp.Location;
                    break;
                }
            }
        }
    }

    void RemoveLinkJointProp(const FString& InLinkName)
    {
        // 1- Rem joint prop having [ParentLinkName] as [InLinkName]
        JointPropList.RemoveAll(
            [this, InLinkName](const FRRRobotJointProperty& InJointProp)
            {
                if (InJointProp.ParentLinkName == InLinkName)
                {
#if RAPYUTA_SIM_DEBUG
                    UE_LOG_WITH_INFO(
                        LogTemp, Error, TEXT("Rem joint from parent [%s] %s"), *InLinkName, *InJointProp.GetTransform().ToString());
#endif
                    return true;
                }
                return false;
            });

        // 2- Rem link prop
        LinkPropList.RemoveAll([this, InLinkName](const FRRRobotLinkProperty& InLinkProp)
                               { return (InLinkProp.Name == InLinkName); });

        // 2.1 - BaseLinkName
        if (InLinkName == BaseLinkName)
        {
            BaseLinkName = TEXT("");
        }

        // 2.2 - ArticulatedLinks/EndEffectors
        ArticulatedLinksNames.RemoveSwap(InLinkName);
        EndEffectorNames.RemoveSwap(InLinkName);

        // 2.3 - Wheels
        WheelPropList.RemoveAll([this, InLinkName](const FRRRobotWheelProperty& InWheelProp)
                                { return (InWheelProp.WheelName == InLinkName); });
    }

    const FRRRobotLinkProperty FindLinkProp(int8 InLinkIndex) const
    {
        for (const auto& linkProp : LinkPropList)
        {
            if (linkProp.LinkIndex == InLinkIndex)
            {
                return linkProp;
            }
        }

        return FRRRobotLinkProperty();
    }

    const FRRRobotJointProperty FindJointPropByLinkIndex(int8 InLinkIndex) const
    {
        const FRRRobotLinkProperty linkProp = FindLinkProp(InLinkIndex);

        for (const auto& jointProp : JointPropList)
        {
            if (linkProp.Name == jointProp.ChildLinkName)
            {
                return jointProp;
            }
        }

        return FRRRobotJointProperty();
    }

    FTransform GetLinkVisualOffset(int8 InLinkIndex) const
    {
        const FRRRobotLinkProperty& linkProp = FindLinkProp(InLinkIndex);
        return linkProp.GetVisualOffset();
    }

    FTransform GetLinkAbsoluteTransform(int8 InLinkIndex) const
    {
        int8 linkIndexCurrent = InLinkIndex;
        TArray<FRRRobotJointProperty> parentLinks;

        while (linkIndexCurrent != INDEX_NONE)
        {
            const FRRRobotLinkProperty linkProp = FindLinkProp(linkIndexCurrent);
            const FRRRobotJointProperty jointProp = FindJointPropByLinkIndex(linkIndexCurrent);
            parentLinks.Insert(jointProp, 0);
            linkIndexCurrent = linkProp.ParentLinkIndex;
        }

        FTransform resultTransform = FTransform::Identity;

        for (const auto& jointProp : parentLinks)
        {
            FTransform jointTransform = jointProp.bIsTransformRelative ? jointProp.GetTransform() : FTransform::Identity;
            resultTransform.SetLocation(resultTransform.GetLocation() +
                                        resultTransform.GetRotation().RotateVector(jointTransform.GetLocation()));
            resultTransform.SetRotation(resultTransform.GetRotation() * jointTransform.GetRotation());
            resultTransform.SetScale3D(resultTransform.GetScale3D() * jointTransform.GetScale3D());
        }

        resultTransform *= GetLinkVisualOffset(InLinkIndex);

        return resultTransform;
    }

    FTransform GetLinkRelativeTransform(int8 InLinkIndex) const
    {
        const FRRRobotJointProperty jointProp = FindJointPropByLinkIndex(InLinkIndex);

        if (jointProp.bIsTransformRelative)
        {
            return jointProp.GetTransform();
        }
        else
        {
            const FRRRobotLinkProperty linkProp = FindLinkProp(InLinkIndex);
            FTransform resultTransform = linkProp.GetVisualOffset();

            if (linkProp.ParentLinkIndex != INDEX_NONE)
            {
                resultTransform = GetLinkAbsoluteTransform(linkProp.ParentLinkIndex).Inverse() * resultTransform;
            }

            return resultTransform;
        }
    }

    FTransform GetLinkRelTransformToBase(int8 InLinkIndex) const
    {
        if (InLinkIndex <= 0)
        {
            return FTransform::Identity;
        }

        for (const auto& linkProp : LinkPropList)
        {
            if (linkProp.LinkIndex == InLinkIndex)
            {
                return GetLinkRelTransformToBase(linkProp.ParentLinkIndex) * linkProp.GetRelativeTransformToParent();
            }
        }
        return FTransform::Identity;
    }

    FTransform GetLinkVisualOffsetToBase(int8 InLinkIndex) const
    {
        int8 linkIndexCurrent = InLinkIndex;
        FTransform resultTransform = GetLinkVisualOffset(InLinkIndex);

        while (linkIndexCurrent != INDEX_NONE)
        {
            for (const auto& linkProp : LinkPropList)
            {
                if (linkProp.LinkIndex == linkIndexCurrent)
                {
                    resultTransform = linkProp.GetRelativeTransformToParent() * resultTransform;
                    linkIndexCurrent = linkProp.ParentLinkIndex;

                    break;
                }
            }
        }

        return resultTransform;
    }

    FRRRobotGeometryInfo GetVisualInfo() const
    {
        return ((LinkPropList.Num() > 0) && (LinkPropList[0].VisualList.Num() > 0)) ? LinkPropList[0].VisualList[0]
                                                                                    : FRRRobotGeometryInfo();
    }

    // UE-StaticMesh
    FString WholeBodyStaticMeshName;

    // Material
    FRRMaterialProperty WholeBodyMaterialInfo;
    FRRMaterialProperty GetBodyMaterialInfo() const
    {
        return WholeBodyMaterialInfo;
    }

    FRRMaterialProperty GetVisualMaterialInfo() const
    {
        return GetVisualInfo().MaterialInfo;
    }

    // Articulated link names
    TArray<FString> ArticulatedLinksNames;

    // Endtip
    TArray<FString> EndEffectorNames;

    // Wheels properties
    TArray<FRRRobotWheelProperty> WheelPropList;
    bool HasWheel(const FString& InWheelName) const
    {
        for (const auto& wheelProp : WheelPropList)
        {
            if (wheelProp.WheelName == InWheelName)
            {
                return true;
            }
        }
        return false;
    }

    bool operator==(const FRRRobotModelInfo& OtherModelInfo)
    {
        return (ModelDescType == OtherModelInfo.ModelDescType) && (ModelNameList == OtherModelInfo.ModelNameList) &&
               DescriptionFilePath.Equals(OtherModelInfo.DescriptionFilePath, ESearchCase::CaseSensitive);
    }
    bool IsValid(bool bIsLogged = false) const
    {
        // NOTE: Joints num could be zero for a single-link object
        const ERRFileType fileType = URRCoreUtils::GetFileType(DescriptionFilePath);
        if (ERRRobotDescriptionType::NONE == ModelDescType)
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(
                    LogTemp, Log, TEXT("%s ModelDescType has not been set!"), *FString::Join(ModelNameList, TEXT(",")));
            }
            return false;
        }
        else if (0 == ModelNameList.Num())
        {
            // Either a world or entity model
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(LogTemp, Log, TEXT("%s ModelNameList EMPTY!"), *DescriptionFilePath);
            }
            return false;
        }
        else if (DescriptionFilePath.IsEmpty())
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(LogTemp, Log, TEXT("%s DescriptionFilePath EMPTY!"), *FString::Join(ModelNameList, TEXT(",")));
            }
            return false;
        }
        else if (WorldName.IsEmpty() && (0 == LinkPropList.Num()))
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(LogTemp, Log, TEXT("%s LinkPropList EMPTY!"), *FString::Join(ModelNameList, TEXT(",")));
            }
            return false;
        }
        else if ((0 == ChildModelsInfo.Num()) && (ModelNameList.Num() > 1))
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(
                    LogTemp, Log, TEXT("[%s] This should be a single model only!"), *FString::Join(ModelNameList, TEXT(",")));
            }
            return false;
        }
        else if (false == BaseLinkName.IsEmpty())
        {
            bool bBaseLinkFound = false;
            for (const auto& linkProp : LinkPropList)
            {
                if (BaseLinkName == linkProp.Name)
                {
                    bBaseLinkFound = true;
                    break;
                }
            }
            if (false == bBaseLinkFound)
            {
                if (bIsLogged)
                {
                    UE_LOG_WITH_INFO(LogTemp, Log, TEXT("BaseLinkName [%s] is not a link name!"), *BaseLinkName);
                }
                return false;
            }
        }
        else if (ArticulatedLinksNames.Num() > 0)
        {
            bool bAllArtLinksFound = true;
            for (const auto& artLinkName : ArticulatedLinksNames)
            {
                bool bLinkFound = false;
                for (const auto& linkProp : LinkPropList)
                {
                    if (artLinkName == linkProp.Name)
                    {
                        bLinkFound = true;
                        break;
                    }
                }
                if (false == bLinkFound)
                {
                    bAllArtLinksFound = false;
                    if (bIsLogged)
                    {
                        UE_LOG_WITH_INFO(LogTemp, Log, TEXT("Articulated link [%s] is not a link name!"), *artLinkName);
                    }
                    break;
                }
            }
            if (false == bAllArtLinksFound)
            {
                return false;
            }
        }
        else if (EndEffectorNames.Num() > 0)
        {
            bool bAllEELinksFound = true;
            for (const auto& eeName : EndEffectorNames)
            {
                bool bEELinkFound = false;
                for (const auto& linkProp : LinkPropList)
                {
                    if (eeName == linkProp.Name)
                    {
                        bEELinkFound = true;
                        break;
                    }
                }
                if (false == bEELinkFound)
                {
                    bAllEELinksFound = false;
                    if (bIsLogged)
                    {
                        UE_LOG_WITH_INFO(LogTemp, Log, TEXT("EndEffector [%s] is not a link name!"), *eeName);
                    }
                    break;
                }
            }
            if (false == bAllEELinksFound)
            {
                return false;
            }
        }
        else if (WheelPropList.Num() > 0)
        {
            bool bAllWheelsFound = true;
            for (const auto& wheelProp : WheelPropList)
            {
                bool bWheelFound = false;
                for (const auto& linkProp : LinkPropList)
                {
                    if (linkProp.Name == wheelProp.WheelName)
                    {
                        bWheelFound = true;
                        break;
                    }
                }
                if (false == bWheelFound)
                {
                    bAllWheelsFound = false;
                    if (bIsLogged)
                    {
                        UE_LOG_WITH_INFO(LogTemp, Log, TEXT("Wheel [%s] is not a link name!"), *wheelProp.WheelName);
                    }
                    break;
                }
            }
            if (false == bAllWheelsFound)
            {
                return false;
            }
        }
        return true;
    }
    void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("WorldName: %s"), *WorldName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("UEComponentTypeFlags: %d"), UEComponentTypeFlags);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("bHasWorldJoint: %d"), bHasWorldJoint);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("ModelNameList: %s"), *FString::Join(ModelNameList, TEXT(",")));
        UE_LOG_WITH_INFO(
            LogTemp, Display, TEXT("DescriptionFilePath: %s"), *FPaths::ConvertRelativePathToFull(DescriptionFilePath));
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("ParentFrameName: %s"), *ParentFrameName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("RelativeTransform: %s"), *RelativeTransform.ToString());
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("BaseLinkName: %s"), *BaseLinkName);
        for (const auto& linkProp : LinkPropList)
        {
            linkProp.PrintSelf();
        }
        for (const auto& jointProp : JointPropList)
        {
            jointProp.PrintSelf();
        }
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("ArticulatedLinksNames: %s"), *FString::Join(ArticulatedLinksNames, TEXT(",")));
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("EndEffectorNames: %s"), *FString::Join(EndEffectorNames, TEXT(",")));
        for (const auto& wheelProp : WheelPropList)
        {
            wheelProp.PrintSelf();
        }
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("WholeBodyStaticMeshName: %s"), *WholeBodyStaticMeshName);
        WholeBodyMaterialInfo.PrintSelf();

        for (const auto& modelInfo : ChildModelsInfo)
        {
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("SUBMODEL"));
            modelInfo.PrintSelf();
        }
    }
};

/**
 * @brief  This class will co-parent with [IFastXmlCallback], which is not a USTRUCT.
 *
 */
class RAPYUTASIMULATIONPLUGINS_API FRRRobotDescriptionParser
{
public:
    static constexpr const int8 ELEMENT_INDEX_NONE = -1;
    virtual ~FRRRobotDescriptionParser()
    {
    }
    static FString GetRealPathFromMeshName(const FString& InMeshName, const FString& InRobotModelsFolderFullPath)
    {
        // Example of Mesh name in:
        // - URDF: [package://pr2/meshes/shoulder_v0/shoulder_pan.dae]
        // - SDF: [model://pr2/meshes/shoulder_v0/shoulder_pan.dae]
        FString meshName(InMeshName);
        if (meshName.RemoveFromStart(TEXT("package://")) || meshName.RemoveFromStart(TEXT("model://")))
        {
            return FPaths::ConvertRelativePathToFull(InRobotModelsFolderFullPath, meshName);
        }
        else
        {
            return InMeshName;
        }
    }
    virtual bool LoadModelInfoFromXML(const FString& InXMLString, FRRRobotModelInfo& OutRobotModelInfo)
    {
        return true;
    }
    virtual FRRRobotModelInfo LoadModelInfoFromFile(const FString& Path)
    {
        return FRRRobotModelInfo();
    }
    virtual void ClearData()
    {
    }
};
