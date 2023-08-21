/**
 * @file RREntityStructs.h
 * @brief Contains various commonly used struct definitions for robot & object entities
 * @copyright Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
 */
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

#include "RREntityStructs.generated.h"
// (NOTE) To avoid cyclic inclusion, except for utils, please DO NOT include any other component header files here!

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
enum class ERREntityDescriptionType : uint8
{
    NONE,
    URDF,             //! [URDF](https://wiki.ros.org/urdf)
    SDF,              //! [SDF](http://sdformat.org)
    UE_DATA_TABLE,    //! [UDataTable](https://docs.unrealengine.com/5.2/en-US/API/Runtime/Engine/Engine/UDataTable)
    SINGLE_CAD,       //! Raw single CAD (FBX, COLLADA, etc.)
    TOTAL
};

/**
 * @brief
 *
 */
UENUM()
enum class ERRRobotStatus : uint8
{
    INVALID,             //! Invalid status, eg not loaded successfully
    CREATING,            //! Being built up (loading the meshses, building the structure, etc.)
    IDLE,                //! Idling, ready for command order
    ORDER_PROCESSING,    //! Received command and in middle of processing
    MOVING,              //! On the move
    ERROR,               //! Having some internal errors
    TOTAL
};

UENUM(meta = (Bitflags))
enum class ERREntityGeometryType : uint8
{
    NONE = 0x00,
    VISUAL = 0x01,
    COLLISION = 0x02,
    INERTIA = 0x04
};
ENUM_CLASS_FLAGS(ERREntityGeometryType);

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
    NONE,
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
    INVALID,    //! Invalid -> Idle: After joint drive params are fully configured
    IDLE,
    MOVING,
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

UENUM()
enum class ERREntityMeshComponentType : uint8
{
    NONE,
    STATIC_MESH,
    SKELETAL_MESH,
    PROCEDURAL_MESH,
    DYNAMIC_MESH,
    TOTAL
};

// -----------------------------------------------------------------------------------------------
// [FRRRobotJointDynamicProperties] --
USTRUCT(BlueprintType)
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
    UPROPERTY(EditAnywhere)
    FString JointName;
    // For Spring-Damper Control Mode --
    //! [Stiffness](https://docs.unrealengine.com/5.2/en-US/physics-damping-in-unreal-engine)
    UPROPERTY(EditAnywhere,
              Category = "Spring-Damper",
              meta = (ClampMin = "0.0", ClampMax = "1000000000.0", UIMin = "0.0", UIMax = "1000000000.0"))
    float SpringStiff = 0.f;
    UPROPERTY(EditAnywhere,
              Category = "Spring-Damper",
              meta = (ClampMin = "0.0", ClampMax = "1000000000.0", UIMin = "0.0", UIMax = "1000000000.0"))
    float LimitSpringStiff = 0.f;
    //! [Damping](https://docs.unrealengine.com/5.2/en-US/physics-damping-in-unreal-engine)
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

    //! [rad/s or m/s]
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
USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotJointProperty
{
    GENERATED_BODY()
public:
    FRRRobotJointProperty()
    {
    }
    FRRRobotJointProperty(FString InName) : Name(MoveTemp(InName))
    {
    }

    UPROPERTY(EditAnywhere)
    FString Name;

    UPROPERTY(EditAnywhere)
    ERRRobotJointType Type = ERRRobotJointType::NONE;

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

    //! [cm] In URDF/SDF: Child Link's relative Location to its Parent
    UPROPERTY(EditAnywhere)
    FVector Location = FVector::ZeroVector;

    //! [Quaternion] In URDF/SDF: Child Link's relative Rotation to its Parent
    UPROPERTY(EditAnywhere)
    FQuat Rotation = FQuat::Identity;

    //! Whether Rotation & Location are absolute or relative
    UPROPERTY(EditAnywhere)
    bool bIsTransformRelative = true;

    FTransform GetTransform() const
    {
        return FTransform(Rotation, Location);
    }
    UPROPERTY(EditAnywhere)
    FString ParentLinkName;
    UPROPERTY(EditAnywhere)
    FString ChildLinkName;
    UPROPERTY(EditAnywhere)
    FString MimicJointName;
    UPROPERTY(EditAnywhere)
    float MimicMultiplier = 1.0f;
    //! [cm]
    UPROPERTY(EditAnywhere)
    float MimicOffset = 0.0f;
    UPROPERTY(EditAnywhere)
    FVector Axis = FVector::ZeroVector;
    UPROPERTY(EditAnywhere)
    FVector AxisInParentFrame = FVector::ZeroVector;
    const FVector& GetAxis(bool bIsLocal = false) const
    {
        return bIsLocal ? Axis : AxisInParentFrame;
    }

    //! [rad] for REVOLUTE joint, [m] for Prismatic Joint. (Both URDF and SDF)
    UPROPERTY(EditAnywhere)
    float LowerLimit = 0.f;

    //! [rad] for REVOLUTE joint, [m] for Prismatic Joint. (Both URDF and SDF)
    UPROPERTY(EditAnywhere)
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
        return (false == Name.IsEmpty()) && (ERRRobotJointType::NONE != Type);
    }
    ELinearConstraintMotion GetLinearConstraintMotion() const
    {
        switch (Type)
        {
            case ERRRobotJointType::NONE:
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
            case ERRRobotJointType::NONE:
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

/**
 * @brief Joint value struct, mainly to directly store input values from ROS cmds, thus units are in SI to avoid conversion overheads
 */
USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotJointValue
{
    GENERATED_BODY()
    //! [rad] for REVOLUTE joint, [m] for Prismatic Joint
    UPROPERTY(EditAnywhere, meta = (ToopTip = "rad or m", ClampMin = "-100.0", ClampMax = "100.0", UIMin = "-100", UIMax = "100.0"))
    double Position = 0;
    //! [m/s]
    UPROPERTY(EditAnywhere, meta = (ToopTip = "m/s", ClampMin = "-100.0", ClampMax = "100.0", UIMin = "-100.0", UIMax = "100.0"))
    double LinearVel = 0;
    //! [rad/s]
    UPROPERTY(EditAnywhere, meta = (ToopTip = "rad/s", ClampMin = "-10.0", ClampMax = "10.0", UIMin = "-10.0", UIMax = "10.0"))
    double AngularVel = 0;

    FString ToString() const
    {
        return FString::Printf(
            TEXT("Pos: %lf[rad or m] LinearVel: %lf[m/s] AngularVel: %lf[rad/s]"), Position, LinearVel, AngularVel);
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

USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotLinkInertia
{
    GENERATED_BODY()
    //! [kg]
    UPROPERTY(EditAnywhere)
    float Mass = 0.f;
    //! [cm]
    UPROPERTY(EditAnywhere)
    FVector Location = FVector::ZeroVector;
    UPROPERTY(EditAnywhere)
    FQuat Rotation = FQuat::Identity;
    UPROPERTY(EditAnywhere)
    float Ixx = 0.f;
    UPROPERTY(EditAnywhere)
    float Ixy = 0.f;
    UPROPERTY(EditAnywhere)
    float Ixz = 0.f;
    UPROPERTY(EditAnywhere)
    float Iyy = 0.f;
    UPROPERTY(EditAnywhere)
    float Iyz = 0.f;
    UPROPERTY(EditAnywhere)
    float Izz = 0.f;
};

USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRREntityGeometryInfo
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere)
    FString Name;
    UPROPERTY(EditAnywhere)
    FString MeshName;

    //! This size is mostly used for Collision info in case of links being Runtime mesh components.
    //! For static mesh based linkes, the collision is already auto-generated by UE.
    UPROPERTY(EditAnywhere)
    FVector Size = FVector::ZeroVector;

    UPROPERTY(EditAnywhere)
    FVector WorldScale = FVector::OneVector;

    //! Owner Link info
    UPROPERTY(EditAnywhere)
    ERRShapeType LinkType = ERRShapeType::NONE;

    UPROPERTY(EditAnywhere)
    FString LinkName;
    UPROPERTY(EditAnywhere)
    FVector Location = FVector::ZeroVector;
    UPROPERTY(EditAnywhere)
    FQuat Rotation = FQuat::Identity;
    FTransform GetTransformOffset() const
    {
        return FTransform(Rotation, Location);
    }
    UPROPERTY(EditAnywhere)
    FRRMaterialProperty MaterialInfo;
};

/**
 * @brief Sensor base info which includes essential base attributes such as Topic name, frame id, publication rate.
 * Any custom sensor-type info struct (lidar, camera, imu, etc.) should inherit from this struct.
 * @sa [URDF-Sensor](http://wiki.ros.org/urdf/XML/sensor)
 * @sa [SDF-Sensor](http://sdformat.org/spec?elem=sensor)
 */
USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRSensorBaseInfo
{
    GENERATED_BODY()

    virtual ~FRRSensorBaseInfo()
    {
    }
    //! ROS Topic name to which sensor data is published to
    UPROPERTY(EditAnywhere)
    FString TopicName = TEXT("sensor_data");

    //! The coordinate frame in which sensor data is published under in the tf tree
    UPROPERTY(EditAnywhere)
    FString FrameId = TEXT("sensor_frame");

    //! The publishing rate
    UPROPERTY(EditAnywhere)
    float PublicationFrequencyHz = 0.f;

    virtual void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- TopicName: %s"), *TopicName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- FrameId: %s"), *FrameId);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- PublicationFrequencyHz: %f"), PublicationFrequencyHz);
    }
};

/**
 * @brief Sensor lidar info
 * @sa [URDF-Lidar](http://wiki.ros.org/urdf/XML/sensor#:~:text=to%20near%20clip.-,%3Cray%3E,-(optional))
 * @sa [SDF-Lidar](http://sdformat.org/spec?elem=sensor#sensor_lidar)
 */
USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRSensorLidarInfo : public FRRSensorBaseInfo
{
    GENERATED_BODY()
    UPROPERTY(EditAnywhere)
    ERRLidarSensorType LidarType = ERRLidarSensorType::NONE;

    //! The number of simulated lidar rays to generate per complete laser sweep cycle
    UPROPERTY(EditAnywhere)
    int32 NHorSamplesPerScan = 360;

    //! The number of simulated lidar rays to generate per complete laser sweep cycle
    UPROPERTY(EditAnywhere)
    int32 NVerSamplesPerScan = 360;

    //! [deg]
    UPROPERTY(EditAnywhere)
    float HMinAngle = 0.f;
    //! [deg]
    UPROPERTY(EditAnywhere)
    float HMaxAngle = 0.f;
    //! Factor to be multiplied by samples to determine the number of range data points returned
    UPROPERTY(EditAnywhere)
    float HResolution = 0.f;
    //! [deg]
    UPROPERTY(EditAnywhere)
    float VMinAngle = 0.f;
    //! [deg]
    UPROPERTY(EditAnywhere)
    float VMaxAngle = 0.f;
    //! Factor to be multiplied by samples to determine the number of range data points returned
    UPROPERTY(EditAnywhere)
    float VResolution = 0.f;

    // Range
    //! [cm] The minimum distance for each lidar ray
    UPROPERTY(EditAnywhere)
    float MinRange = 0.f;
    //! [cm] The maximum distance for each lidar ray
    UPROPERTY(EditAnywhere)
    float MaxRange = 0.f;
    //! Linear resolution of each lidar ray
    UPROPERTY(EditAnywhere)
    float RangeResolution = 0.f;

    // Noise
    UPROPERTY(EditAnywhere)
    FString NoiseTypeName;
    UPROPERTY(EditAnywhere)
    double NoiseMean = 0;
    UPROPERTY(EditAnywhere)
    double NoiseStdDev = 0;

    virtual void PrintSelf() const override
    {
        Super::PrintSelf();
        UE_LOG_WITH_INFO(
            LogTemp, Display, TEXT("- LidarType: %s"), *URRTypeUtils::GetEnumValueAsString(TEXT("ERRLidarSensorType"), LidarType));

        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Scan horizontal"));
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ NHorSamplesPerScan: %d"), NHorSamplesPerScan);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ HMinAngle: %f"), HMinAngle);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ HMaxAngle: %f"), HMaxAngle);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ HResolution: %f"), HResolution);

        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Scan vertical"));
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ NVerSamplesPerScan: %d"), NVerSamplesPerScan);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ VMinAngle: %f"), VMinAngle);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ VMaxAngle: %f"), VMaxAngle);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ VResolution: %f"), VResolution);

        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Range"));
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ MinRange: %f"), MinRange);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ MinRange: %f"), MaxRange);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ Resolution: %f"), RangeResolution);

        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Noise"));
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ Type: %s"), *NoiseTypeName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ Mean: %f"), NoiseMean);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("\t+ StdDev: %f"), NoiseStdDev);
    }
};

USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRSensorProperty
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere)
    FString LinkName;
    UPROPERTY(EditAnywhere)
    FString SensorName;
    UPROPERTY(EditAnywhere)
    ERRSensorType SensorType = ERRSensorType::NONE;

    //! TODO: To avoid extra memory cost, this should be actually a TUniquePtr, which is only instantiated upon SensorType as LIDAR.
    //! However, it will become not visible/editable in Blueprint like BP data table
    UPROPERTY(EditAnywhere)
    FRRSensorLidarInfo LidarInfo;

    void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp,
                         Warning,
                         TEXT("Sensor: %s %s"),
                         *URRTypeUtils::GetEnumValueAsString(TEXT("ERRSensorType"), SensorType),
                         *SensorName);
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("- Link: %s"), *LinkName);

        LidarInfo.PrintSelf();
    }
};

USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotLinkProperty
{
    GENERATED_BODY()
public:
    FRRRobotLinkProperty()
    {
    }
    FRRRobotLinkProperty(FString InName) : Name(MoveTemp(InName))
    {
    }
    UPROPERTY(EditAnywhere)
    FString Name;

    //! Only available in SDF
    UPROPERTY(EditAnywhere)
    FString ParentFrameName;

    UPROPERTY(EditAnywhere)
    int8 LinkIndex = INDEX_NONE;
    UPROPERTY(EditAnywhere)
    int8 ParentLinkIndex = INDEX_NONE;

    //! Relative to the parent link
    //! In URDF: <joint>�s <origin xyz= "" rpy="">
    //! In SDF : <joint> -> <pose> 6 values (xyz + rpy)
    UPROPERTY(EditAnywhere)
    FVector Location = FVector::ZeroVector;

    UPROPERTY(EditAnywhere)
    FQuat Rotation = FQuat::Identity;
    FTransform GetRelativeTransformToParent() const
    {
        return FTransform(Rotation, Location);
    }
    UPROPERTY(EditAnywhere)
    FRRRobotLinkInertia Inertia;

    //! Relative visual offset to the link itself(URDF) or parent frame(SDF)
    //! In URDF: <visual>�s <origin xyz= "" rpy="">
    //! In SDF : <link> -> <pose> 6 values (xyz + rpy)
    UPROPERTY(EditAnywhere)
    TArray<FRREntityGeometryInfo> VisualList;
    FTransform GetVisualOffset(int32 InVisualIndex = 0) const
    {
        return VisualList.IsValidIndex(InVisualIndex) ? VisualList[InVisualIndex].GetTransformOffset() : FTransform::Identity;
    }
    FTransform GetVisualRelativeTransformToParent(int32 InVisualIndex = 0) const
    {
        return GetRelativeTransformToParent() * GetVisualOffset(InVisualIndex);
    }
    UPROPERTY(EditAnywhere)
    TArray<FRREntityGeometryInfo> CollisionList;
    bool operator==(const FRRRobotLinkProperty& LinkProp)
    {
        return Name.Equals(LinkProp.Name);
    }
    bool operator==(const FString& String)
    {
        return Name.Equals(String);
    }
    // Sensor
    UPROPERTY(EditAnywhere)
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
            sensor.PrintSelf();
        }
    }
};

// Ref: UChaosVehicleWheel

/**
 * @brief Wheel property for the robot, used to store configurations for runtime #UChaosVehicleWheel setup
 * @sa [UChaosVehicleWheel](https://docs.unrealengine.com/5.1/en-US/API/Plugins/ChaosVehicles/UChaosVehicleWheel/)
 */
USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRRRobotWheelProperty
{
    GENERATED_BODY()
public:
    FRRRobotWheelProperty()
    {
    }
    FRRRobotWheelProperty(FString InWheelName) : WheelName(MoveTemp(InWheelName))
    {
    }

    UPROPERTY(EditAnywhere)
    FString WheelName;

    //! If left undefined then the #bAffectedByEngine value is used, if defined then #bAffectedByEngine is ignored and the
    //! differential setup on the vehicle defines which wheels get power from the engine
    UPROPERTY(EditAnywhere)
    EAxleType AxleType = EAxleType::Undefined;

    //! If BoneName is specified, offset the wheel from the bone's location.
    //! Otherwise this offsets the wheel from the vehicle's origin.
    //! [cm]
    UPROPERTY(EditAnywhere)
    FVector Offset = FVector::ZeroVector;

    //! [cm]
    UPROPERTY(EditAnywhere)
    float WheelRadius = 0.f;

    //! [cm]
    UPROPERTY(EditAnywhere)
    float WheelWidth = 0.f;

    UPROPERTY(EditAnywhere)
    float CorneringStiffness = 1000.f;

    UPROPERTY(EditAnywhere)
    float FrictionForceMultiplier = 2.f;

    //! Wheel Lateral Skid Grip Loss, lower number less grip on skid
    UPROPERTY(EditAnywhere)
    float SideSlipModifier = 1.f;

    //! Wheel Longitudinal Slip Threshold
    UPROPERTY()
    float SlipThreshold = 20.f;

    //! Wheel Lateral Skid Threshold
    UPROPERTY(EditAnywhere)
    float SkidThreshold = 20.f;

    UPROPERTY(EditAnywhere)
    bool bAffectedBySteering = false;

    UPROPERTY(EditAnywhere)
    bool bAffectedByBrake = true;

    UPROPERTY(EditAnywhere)
    bool bAffectedByHandbrake = false;

    //! Whether engine should power this wheel
    UPROPERTY(EditAnywhere)
    bool bAffectedByEngine = false;

    //! Advanced Braking System enabled
    UPROPERTY(EditAnywhere)
    bool bABSEnabled = false;

    //! Straight Line Traction Control Enabled
    UPROPERTY(EditAnywhere)
    bool bTractionControlEnabled = false;

    //! Determines how the SetDriveTorque/SetBrakeTorque inputs are combined with the internal torques
    UPROPERTY(EditAnywhere)
    ETorqueCombineMethod ExternalTorqueCombineMethod = ETorqueCombineMethod::None;

    UPROPERTY(EditAnywhere)
    FRuntimeFloatCurve LateralSlipGraph;

    //! Local body direction in which where suspension forces are applied (typically along -Z-axis)
    UPROPERTY(EditAnywhere)
    FVector SuspensionAxis = FVector(0.f, 0.f, -1.f);

    //! Vertical offset from where suspension forces are applied (along Z-axis)
    UPROPERTY(EditAnywhere)
    FVector SuspensionForceOffset = FVector::ZeroVector;

    //! How far the wheel can go above the resting position
    //! [cm]
    UPROPERTY(EditAnywhere)
    float SuspensionMaxRaise = 10.f;

    //! How far the wheel can drop below the resting position
    //! [cm]
    UPROPERTY(EditAnywhere)
    float SuspensionMaxDrop = 10.f;

    /** Suspension damping, larger value causes the suspension to come to rest faster [range 0 to 1] */
    UPROPERTY(EditAnywhere)
    float SuspensionDampingRatio = 0.5f;

    //! Smooth suspension [0-off, 10-max] - Warning might cause momentary visual inter-penetration of the wheel against
    //! objects/terrain
    UPROPERTY(EditAnywhere)
    int8 SuspensionSmoothing = 0;

    //! Amount wheel load effects wheel friction.
    //! At 0 wheel friction is completely independent of the loading on the wheel (This is artificial as it always assumes even
    //! balance between all wheels) At 1 wheel friction is based on the force pressing wheel into the ground. This is more
    //! realistic. Lower value cures lift off over-steer, generally makes vehicle easier to handle under extreme motions.
    UPROPERTY(EditAnywhere)
    float WheelLoadRatio = 0.5f;

    //! Spring Force (N/m)
    UPROPERTY(EditAnywhere)
    float SpringRate = 250.0f;

    //! Spring Preload (N/m)
    UPROPERTY(EditAnywhere)
    float SpringPreload = 50.f;

    //! Anti-roll effect
    UPROPERTY(EditAnywhere)
    float RollbarScaling = 0.15f;

    //! Whether wheel suspension considers simple, complex, or both
    UPROPERTY(EditAnywhere)
    ESweepShape SweepShape = ESweepShape::Raycast;

    //! Whether wheel suspension considers simple, complex, or both
    UPROPERTY(EditAnywhere)
    ESweepType SweepType = ESweepType::SimpleSweep;

    //! [deg]
    UPROPERTY(EditAnywhere)
    float MaxSteerAngleDeg = 50.f;

    //! [Nm]
    UPROPERTY(EditAnywhere)
    float MaxBrakeTorque = 1500.f;

    //! Max handbrake brake torque for this wheel (Nm). A handbrake should have a stronger brake torque
    //! than the brake. This will be ignored for wheels that are not affected by the handbrake.
    //! [Nm]
    UPROPERTY(EditAnywhere)
    float MaxHandBrakeTorque = 3000.f;

    void PrintSelf() const
    {
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("WheelName: %s Radius %f[cm] Width %f[cm]"), *WheelName, WheelRadius, WheelWidth);
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

// [ENTITY MODEL] --
/**
 * @brief Core UE struct housing entity (robot, object) model data, wrapped by #FRREntityModelInfo & #FRREntityModelTableRow
 */
USTRUCT(BlueprintType)
struct FRREntityModelData
{
    GENERATED_BODY()
public:
    FRREntityModelData(const TArray<FString>& InModelNameList) : ModelNameList(InModelNameList)
    {
    }
    FRREntityModelData(TArray<FString>&& InModelNameList) : ModelNameList(MoveTemp(InModelNameList))
    {
    }
    FRREntityModelData()
    {
    }

    //! Robot Model Description Type (URDF/SDF)
    UPROPERTY(EditAnywhere)
    ERREntityDescriptionType ModelDescType = ERREntityDescriptionType::NONE;
    bool IsURDF() const
    {
        return (ERREntityDescriptionType::URDF == ModelDescType);
    }
    bool IsSDF() const
    {
        return (ERREntityDescriptionType::SDF == ModelDescType);
    }
    bool IsUEDataTable() const
    {
        return (ERREntityDescriptionType::UE_DATA_TABLE == ModelDescType);
    }
    bool IsSingleCAD() const
    {
        return (ERREntityDescriptionType::SINGLE_CAD == ModelDescType);
    }
    FString GetModelDescTypeName() const
    {
        return URRTypeUtils::GetEnumValueAsString(TEXT("ERREntityDescriptionType"), ModelDescType);
    }

    UPROPERTY(EditAnywhere)
    FString WorldName;
    bool IsWorldModel() const
    {
        return (false == WorldName.IsEmpty());
    }

    UPROPERTY()
    bool bHasWorldJoint = false;

    //! World model: [ModelNameList] >= 1
    //! Pure model:  [ModelNameList] == 1 if Single else > 1
    UPROPERTY(EditAnywhere)
    TArray<FString> ModelNameList;
    FString GetModelName() const
    {
        return IsWorldModel() ? WorldName : ((ModelNameList.Num() > 0) ? ModelNameList[0] : EMPTY_STR);
    }

    //! Meshes are async loaded, & so their sizes
    UPROPERTY(VisibleAnywhere)
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
        for (const auto& modelInfo : ChildModelsData)
        {
            meshesNum += modelInfo.GetVisualsNum();
        }
        return meshesNum;
    }

    //! Path to the base package folder, from which meshes' CAD file paths are defined as relative to
    UPROPERTY(VisibleAnywhere)
    FString BaseFolderPath;

    //! Robot Model Description File (URDF/SDF)
    UPROPERTY(VisibleAnywhere)
    FString DescriptionFilePath;
    UPROPERTY(VisibleAnywhere)
    int32 CreatedInstancesNum = 0;
    UPROPERTY(EditAnywhere)
    FString ParentFrameName;
    UPROPERTY(EditAnywhere)
    FTransform RelativeTransform = FTransform::Identity;

    //! UE Component types
    UPROPERTY(EditAnywhere)
    int32 UEComponentTypeFlags = 0;
    bool IsUEComponentEnabled(const int32 InTypeMask) const
    {
        return (UEComponentTypeFlags & InTypeMask);
    }
    bool IsUEComponentEnabled(const ERRUEComponentType InTypeMask) const
    {
        return IsUEComponentEnabled(static_cast<int32>(InTypeMask));
    }
    void SetUEComponentEnabled(const int32 InTypeMask)
    {
        UEComponentTypeFlags |= InTypeMask;
    }
    void SetUEComponentEnabled(const ERRUEComponentType InTypeMask)
    {
        SetUEComponentEnabled(static_cast<int32>(InTypeMask));
    }
    bool IsPlainManipulatorModel() const
    {
        return (ERRUEComponentType::ARTICULATION_DRIVE == static_cast<ERRUEComponentType>(UEComponentTypeFlags));
    }
    bool IsPlainWheeledVehicleModel() const
    {
        return (ERRUEComponentType::WHEEL_DRIVE == static_cast<ERRUEComponentType>(UEComponentTypeFlags));
    }
    bool HasDriveComponents() const
    {
        return IsUEComponentEnabled(static_cast<int32>(ERRUEComponentType::ARTICULATION_DRIVE | ERRUEComponentType::DIFF_DRIVE |
                                                       ERRUEComponentType::WHEEL_DRIVE));
    }

    bool IsRobotModel() const
    {
        return (JointPropList.Num() > 0) || HasDriveComponents();
    }

    bool IsObjectModel() const
    {
        return (false == IsRobotModel());
    }

    // Link/Joint list
    UPROPERTY(EditAnywhere)
    FString BaseLinkName;
    UPROPERTY(EditAnywhere)
    TArray<FRRRobotLinkProperty> LinkPropList;
    UPROPERTY(EditAnywhere)
    TArray<FRRRobotJointProperty> JointPropList;

    //! Struct recursion is NOT yet supported for UPROPERTY
    // UPROPERTY(EditAnywhere)
    TArray<FRREntityModelData> ChildModelsData;

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

    /**
     * @brief Remove all link properties of InLinkName and its corresponding joint properties having their #ParentLinkName as InLinkName
     * @param InLinkName
     */
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

    /**
     * @brief Get the property of base link of which the name is denoted by #BaseLinkName
     * @return FRRRobotLinkProperty
     */
    FRRRobotLinkProperty GetBaseLinkProp() const
    {
        for (const auto& linkProp : LinkPropList)
        {
            if (linkProp.Name == BaseLinkName)
            {
                return linkProp;
            }
        }
        return FRRRobotLinkProperty();
    }

    /**
     * @brief Get the property of a link by name read from URDF/SDF or CAD like FBX/COLLADA
     * @param InLinkName
     * @return FRRRobotLinkProperty
     */
    FRRRobotLinkProperty GetLinkProp(const FString& InLinkName) const
    {
        for (const auto& linkProp : LinkPropList)
        {
            if (linkProp.Name == InLinkName)
            {
                return linkProp;
            }
        }
        return FRRRobotLinkProperty();
    }

    /**
     * @brief Get the property of a joint by name read from URDF/SDF or CAD like FBX/COLLADA
     * @param InJointName
     * @return FRRRobotJointProperty
     */
    FRRRobotJointProperty GetJointProp(const FString& InJointName) const
    {
        for (const auto& jointProp : JointPropList)
        {
            if (jointProp.Name == InJointName)
            {
                return jointProp;
            }
        }
        return FRRRobotJointProperty();
    }

    /**
     * @brief Get the property of the first joint having its child link name as InChildLinkName
     * @param InChildLinkName
     * @return FRRRobotJointProperty
     */
    FRRRobotJointProperty GetJointPropFromChildLinkName(const FString& InChildLinkName) const
    {
        for (const auto& jointProp : JointPropList)
        {
            if (jointProp.ChildLinkName == InChildLinkName)
            {
                return jointProp;
            }
        }
        return FRRRobotJointProperty();
    }

    /**
     * @brief Get the property of a link by its LinkIndex
     * NOTE: LinkIndex is assigned indendently from its index in #LinkPropList as building robot skeleton structure
     * @param InLinkIndex
     * @return FRRRobotLinkProperty
     */
    FRRRobotLinkProperty FindLinkProp(int8 InLinkIndex) const
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

    /**
     * @brief Get the property of a joint of which the child link name matchs the link property fetched by its LinkIndex
     * NOTE: LinkIndex is assigned indendently from its index in #LinkPropList as building robot skeleton structure
     * @param InLinkIndex
     * @return FRRRobotJointProperty
     */
    FRRRobotJointProperty FindJointPropByLinkIndex(int8 InLinkIndex) const
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

    /**
     * @brief Get visual offset transform of a link by its LinkIndex
     * NOTE: LinkIndex is assigned indendently from its index in #LinkPropList as building robot skeleton structure
     * @param InLinkIndex
     * @return FTransform
     */
    FTransform GetLinkVisualOffset(int8 InLinkIndex) const
    {
        const FRRRobotLinkProperty& linkProp = FindLinkProp(InLinkIndex);
        return linkProp.GetVisualOffset();
    }

    /**
     * @brief Get the absolute transform of a link (in owner robot's frame) by its LinkIndex
     * NOTE: LinkIndex is assigned indendently from its index in #LinkPropList as building robot skeleton structure
     * @param InLinkIndex
     * @return FTransform
     */
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

    /**
     * @brief Get the relative transform of a link to its parent by its LinkIndex
     * NOTE: LinkIndex is assigned indendently from its index in #LinkPropList as building robot skeleton structure
     * @param InLinkIndex
     * @return FTransform
     */
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

    /**
     * @brief Get the relative transform of a link to base link its LinkIndex
     * NOTE: LinkIndex is assigned indendently from its index in #LinkPropList as building robot skeleton structure
     * @param InLinkIndex
     * @return FTransform
     */
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

    /**
     * @brief Get the visual offset transform of a link to base link its LinkIndex
     * NOTE: LinkIndex is assigned indendently from its index in #LinkPropList as building robot skeleton structure
     * @param InLinkIndex
     * @return FTransform
     */
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

    /**
     * @brief Get model visual info
     */
    FRREntityGeometryInfo GetVisualInfo() const
    {
        return ((LinkPropList.Num() > 0) && (LinkPropList[0].VisualList.Num() > 0)) ? LinkPropList[0].VisualList[0]
                                                                                    : FRREntityGeometryInfo();
    }

    // UE-StaticMesh
    UPROPERTY(EditAnywhere)
    FString WholeBodyStaticMeshName;

    // Material
    UPROPERTY(EditAnywhere)
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
    UPROPERTY(EditAnywhere)
    TArray<FString> ArticulatedLinksNames;

    // Endtip
    UPROPERTY(EditAnywhere)
    TArray<FString> EndEffectorNames;

    // Wheels properties
    UPROPERTY(EditAnywhere)
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

    bool operator==(const FRREntityModelData& OtherModelData)
    {
        return (ModelDescType == OtherModelData.ModelDescType) && (ModelNameList == OtherModelData.ModelNameList) &&
               DescriptionFilePath.Equals(OtherModelData.DescriptionFilePath, ESearchCase::CaseSensitive);
    }
    bool IsValid(bool bIsLogged = false) const
    {
        // NOTE: Joints num could be zero for a single-link object
        if (ERREntityDescriptionType::NONE == ModelDescType)
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
        else if (ModelNameList[0].IsEmpty())
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(LogTemp, Log, TEXT("%s ModelName EMPTY!"), *DescriptionFilePath);
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
        else if (((ERREntityDescriptionType::URDF == ModelDescType) || (ERREntityDescriptionType::SDF == ModelDescType)) &&
                 BaseFolderPath.IsEmpty())
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(LogTemp, Log, TEXT("%s BaseFolderPath EMPTY!"), *FString::Join(ModelNameList, TEXT(",")));
            }
            return false;
        }
        else if ((false == IsSingleCAD()) && (false == IsWorldModel()) && (0 == LinkPropList.Num()))
        {
            if (bIsLogged)
            {
                UE_LOG_WITH_INFO(LogTemp, Log, TEXT("%s LinkPropList EMPTY!"), *FString::Join(ModelNameList, TEXT(",")));
            }
            return false;
        }
        else if ((0 == ChildModelsData.Num()) && (ModelNameList.Num() > 1))
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
        UE_LOG_WITH_INFO(LogTemp, Display, TEXT("BaseFolderPath: %s"), *FPaths::ConvertRelativePathToFull(BaseFolderPath));
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

        for (const auto& modelInfo : ChildModelsData)
        {
            UE_LOG_WITH_INFO(LogTemp, Warning, TEXT("SUBMODEL"));
            modelInfo.PrintSelf();
        }
    }
};    // END FRREntityModelData

/**
 * @brief Struct, inheriting from #FTableRowBase, storing entry data for #UDataTable
 * @sa [Ref](https://docs.unrealengine.com/5.2/en-US/API/Runtime/Engine/Engine/FTableRowBase)
 */
USTRUCT(BlueprintType)
struct RAPYUTASIMULATIONPLUGINS_API FRREntityModelTableRow : public FTableRowBase
{
    GENERATED_BODY()
public:
    FRREntityModelTableRow(const FRREntityModelData& InEntityModelData) : Data(InEntityModelData)
    {
    }
    FRREntityModelTableRow(FRREntityModelData&& InEntityModelData) : Data(MoveTemp(InEntityModelData))
    {
    }
    FRREntityModelTableRow()
    {
    }
    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FRREntityModelData Data;
};

/**
 * @brief Plain struct wrapping robot model data (#FRREntityModelData), being thread-safe accessible through TSharedRef if created with TSharedPtr
 */
struct RAPYUTASIMULATIONPLUGINS_API FRREntityModelInfo : public TSharedFromThis<FRREntityModelInfo, ESPMode::ThreadSafe>
{
public:
    FRREntityModelInfo(const FRREntityModelData& InEntityModelData) : Data(InEntityModelData)
    {
    }
    FRREntityModelInfo(FRREntityModelData&& InEntityModelData) : Data(MoveTemp(InEntityModelData))
    {
    }
    FRREntityModelInfo()
    {
    }

    //! Robot model data
    FRREntityModelData Data;

    FORCEINLINE const FString& GetBaseFolderPath() const
    {
        return Data.BaseFolderPath;
    }

    FORCEINLINE const FString& GetDescriptionFilePath() const
    {
        return Data.DescriptionFilePath;
    }

    FORCEINLINE const FString GetModelName() const
    {
        return Data.GetModelName();
    }

    FORCEINLINE const FString& GetWorldName() const
    {
        return Data.WorldName;
    }

    FORCEINLINE int32 GetCreatedInstancesNum() const
    {
        return Data.CreatedInstancesNum;
    }

    FORCEINLINE int32& GetCreatedInstancesNum()
    {
        return Data.CreatedInstancesNum;
    }

    FORCEINLINE bool HasWorldJoint() const
    {
        return Data.bHasWorldJoint;
    }
    FORCEINLINE int32 GetVisualsNum() const
    {
        return Data.GetVisualsNum();
    }

    //! UE Component types
    FORCEINLINE int32 GetUEComponentTypeFlags() const
    {
        return Data.UEComponentTypeFlags;
    }
    FORCEINLINE bool IsUEComponentEnabled(const uint8 InTypeMask) const
    {
        return Data.IsUEComponentEnabled(InTypeMask);
    }
    FORCEINLINE bool IsUEComponentEnabled(const ERRUEComponentType InTypeMask) const
    {
        return Data.IsUEComponentEnabled(InTypeMask);
    }
    FORCEINLINE bool IsPlainManipulatorModel() const
    {
        return Data.IsPlainManipulatorModel();
    }
    FORCEINLINE bool IsPlainWheeledVehicleModel() const
    {
        return Data.IsPlainWheeledVehicleModel();
    }
    FORCEINLINE bool IsRobotModel() const
    {
        return Data.IsRobotModel();
    }
    FORCEINLINE bool IsObjectModel() const
    {
        return Data.IsObjectModel();
    }

    // Link/Joint list
    FORCEINLINE const FString& GetBaseLinkName() const
    {
        return Data.BaseLinkName;
    }
    FORCEINLINE const TArray<FRRRobotLinkProperty>& GetLinkPropList() const
    {
        return Data.LinkPropList;
    }
    FORCEINLINE TArray<FRRRobotLinkProperty>& GetLinkPropList()
    {
        return Data.LinkPropList;
    }
    FORCEINLINE const TArray<FRRRobotJointProperty>& GetJointPropList() const
    {
        return Data.JointPropList;
    }
    FORCEINLINE TArray<FRRRobotJointProperty>& GetJointPropList()
    {
        return Data.JointPropList;
    }
    FORCEINLINE const TArray<FRREntityModelData>& GetChildModelsData() const
    {
        return Data.ChildModelsData;
    }

    FORCEINLINE void RemoveLinkJointProp(const FString& InLinkName)
    {
        Data.RemoveLinkJointProp(InLinkName);
    }

    FORCEINLINE FRRRobotLinkProperty GetBaseLinkProp() const
    {
        return Data.GetBaseLinkProp();
    }

    FORCEINLINE FRRRobotLinkProperty GetLinkProp(const FString& InLinkName) const
    {
        return Data.GetLinkProp(InLinkName);
    }

    FORCEINLINE FRRRobotJointProperty GetJointProp(const FString& InJointName) const
    {
        return Data.GetJointProp(InJointName);
    }

    FORCEINLINE FRRRobotJointProperty GetJointPropFromChildLinkName(const FString& InChildLinkName) const
    {
        return Data.GetJointPropFromChildLinkName(InChildLinkName);
    }

    FORCEINLINE const FRRRobotLinkProperty FindLinkProp(int8 InLinkIndex) const
    {
        return Data.FindLinkProp(InLinkIndex);
    }

    FORCEINLINE const FRRRobotJointProperty FindJointPropByLinkIndex(int8 InLinkIndex) const
    {
        return Data.FindJointPropByLinkIndex(InLinkIndex);
    }

    FORCEINLINE FTransform GetLinkVisualOffset(int8 InLinkIndex) const
    {
        return Data.GetLinkVisualOffset(InLinkIndex);
    }

    FORCEINLINE FTransform GetLinkAbsoluteTransform(int8 InLinkIndex) const
    {
        return Data.GetLinkAbsoluteTransform(InLinkIndex);
    }

    FORCEINLINE FTransform GetLinkRelativeTransform(int8 InLinkIndex) const
    {
        return Data.GetLinkRelativeTransform(InLinkIndex);
    }

    FORCEINLINE FTransform GetLinkRelTransformToBase(int8 InLinkIndex) const
    {
        return Data.GetLinkRelTransformToBase(InLinkIndex);
    }

    FORCEINLINE FTransform GetLinkVisualOffsetToBase(int8 InLinkIndex) const
    {
        return Data.GetLinkVisualOffsetToBase(InLinkIndex);
    }

    FORCEINLINE FRREntityGeometryInfo GetVisualInfo() const
    {
        return Data.GetVisualInfo();
    }

    // Material
    FORCEINLINE FRRMaterialProperty GetBodyMaterialInfo() const
    {
        return Data.GetBodyMaterialInfo();
    }

    FORCEINLINE FRRMaterialProperty GetVisualMaterialInfo() const
    {
        return Data.GetVisualMaterialInfo();
    }

    // Articulated link names
    FORCEINLINE const TArray<FString>& GetArticulatedLinksNames() const
    {
        return Data.ArticulatedLinksNames;
    }

    // Endtip
    FORCEINLINE const TArray<FString>& GetEndEffectorNames() const
    {
        return Data.EndEffectorNames;
    }

    // Wheels properties
    FORCEINLINE const TArray<FRRRobotWheelProperty>& GetWheelPropList() const
    {
        return Data.WheelPropList;
    }
    FORCEINLINE bool HasWheel(const FString& InWheelName) const
    {
        return Data.HasWheel(InWheelName);
    }

    FORCEINLINE bool operator==(const FRREntityModelInfo& OtherModelInfo)
    {
        return (Data == OtherModelInfo.Data);
    }
    FORCEINLINE bool IsValid(bool bIsLogged = false) const
    {
        return Data.IsValid(bIsLogged);
    }
    FORCEINLINE void PrintSelf() const
    {
        Data.PrintSelf();
    }
};

/**
 * @brief  This class will co-parent with [IFastXmlCallback], which is not a USTRUCT.
 *
 */
class RAPYUTASIMULATIONPLUGINS_API FRREntityDescriptionParser
{
public:
    static constexpr const int8 ELEMENT_INDEX_NONE = -1;
    virtual ~FRREntityDescriptionParser()
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
    virtual bool LoadModelInfoFromXML(const FString& InXMLString, FRREntityModelInfo& OutRobotModelInfo)
    {
        return true;
    }
    virtual FRREntityModelInfo LoadModelInfoFromFile(const FString& Path)
    {
        return FRREntityModelInfo();
    }
    virtual void ClearData()
    {
    }
};
