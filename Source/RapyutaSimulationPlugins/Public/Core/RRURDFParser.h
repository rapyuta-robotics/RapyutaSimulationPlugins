// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
#pragma once

// UE
#include "Runtime/XmlParser/Public/FastXml.h"

// RapyutaSimulationPlugins
#include "RapyutaSimulationPlugins.h"
#include "Robots/RRRobotStructs.h"

#define RAPYUTA_URDF_PARSER_DEBUG (0)

/**
 * URDF Parser to parse the information from the robot's URDF file.
 * References:
 * https://github.com/ros/urdfdom
 * https://github.com/ros/urdf_parser_py/blob/melodic-devel/src/urdf_parser_py/urdf.py
 * https://github.com/robcog-iai/URoboSim/blob/master/Source/URoboSim/Public/RURDFParser.h
 * https://github.com/cconti-rr/GeometryImporterPOC/blob/master/Plugins/URDFImporter/Source/URDFImporter/Public/URDFParser.h
 */
class RAPYUTASIMULATIONPLUGINS_API FRRURDFParser : public FRRRobotDescriptionParser, private IFastXmlCallback
{
public:
    FRRRobotModelInfo LoadModelInfoFromFile(const FString& InURDFPath) override;
    bool LoadModelInfoFromXML(const FString& InUrdfXml, FRRRobotModelInfo& OutRobotModelInfo) override;

private:
    static constexpr const TCHAR* GEOMETRY_TYPE_PREFIX_VISUAL = TEXT("visual");
    static constexpr const TCHAR* GEOMETRY_TYPE_PREFIX_COLLISION = TEXT("collision");
    static constexpr const TCHAR* GEOMETRY_TYPE_PREFIX_INERTIAL = TEXT("inertial");

    // Local model info properties
    FString ModelName;
    bool bHasWorldJoint = false;
    uint32 UEComponentTypeFlags = 0;
    bool IsPlainManipulatorModel() const
    {
        return (ERRUEComponentType::ARTICULATION_DRIVE == static_cast<ERRUEComponentType>(UEComponentTypeFlags));
    }

    bool IsPlainWheeledVehicleModel() const
    {
        return (ERRUEComponentType::WHEEL_DRIVE == static_cast<ERRUEComponentType>(UEComponentTypeFlags));
    }

    TArray<FRRRobotLinkProperty> LinkPropList;
    TArray<FRRRobotJointProperty> JointPropList;
    FString BaseLinkName;
    TArray<FString> ArticulatedLinksNames;
    TArray<FString> EndEffectorNames;
    TArray<FRRRobotWheelProperty> WheelPropList;
    FRRMaterialProperty WholeBodyMaterialInfo;

    // Element stack
    TArray<FString> ElemStack;

    // Elements' Name & Value attributes
    TMap<FString, FString> AttMap;
    void Reset()
    {
        ModelName = EMPTY_STR;
        bHasWorldJoint = false;
        UEComponentTypeFlags = 0;

        LinkPropList.Reset();
        JointPropList.Reset();
        BaseLinkName = EMPTY_STR;
        ArticulatedLinksNames.Reset();
        WheelPropList.Reset();
        EndEffectorNames.Reset();
        WholeBodyMaterialInfo = FRRMaterialProperty();

        ElemStack.Reset();
        AttMap.Reset();
    }

    // Parser methods
    FORCEINLINE FString ComposeAttributeKey(const FString& InElementName,
                                            const FString& InAttributeName,
                                            const TCHAR* InPrefix = nullptr)
    {
        return InPrefix ? FString::Printf(TEXT("%s_%s_%s"), InPrefix, *InElementName, *InAttributeName)
                        : FString::Printf(TEXT("%s_%s"), *InElementName, *InAttributeName);
    }

    void ParseUELinkMaterialAndSensorInfo();
    void ParseModelUESpecifics();

    FVector ParseVector(const FString& InElementName, bool bIsForLocation = true);
    FQuat ParseRotation(const FString& InElementName);
    FTransform ParsePose(const FString& InElementName);
    FVector ParseCylinderSize(const FString& InRadiusElementName, const FString& InLengthElementName);
    bool ParseGeometryInfo(const FString& InLinkName,
                           const ERRRobotGeometryType InGeometryType,
                           FRRRobotGeometryInfo& OutGeometryInfo);
    bool ParseJointProperty();
    bool ParseLinkProperty();
    bool ParseSensorProperty(FRRSensorProperty& OutSensorProp);

    // [IFastXmlCallback] methods
    virtual bool ProcessXmlDeclaration(const TCHAR* InElementData, int32 InXmlFileLineNumber)
    {
        return true;
    }
    bool ProcessComment(const TCHAR* InComment)
    {
        return true;
    }
    bool ProcessAttribute(const TCHAR* InAttributeName, const TCHAR* InAttributeValue);
    bool ProcessElement(const TCHAR* InElementName, const TCHAR* InElementData, int32 InXmlFileLineNumber);
    bool ProcessClose(const TCHAR* InElementName);
};
