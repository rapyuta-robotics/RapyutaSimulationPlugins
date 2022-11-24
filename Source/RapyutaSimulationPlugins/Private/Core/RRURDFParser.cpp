// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.

#include "Core/RRURDFParser.h"

// RapyutaSimulationPlugins
#include "Core/RRConversionUtils.h"
#include "Core/RRCoreUtils.h"

static TArray<const TCHAR*> UE_ELEMENT_LIST = {TEXT("ue_sensor_ray_scan_horizontal"),
                                               TEXT("ue_sensor_ray_scan_vertical"),
                                               TEXT("ue_sensor_ray_range"),
                                               TEXT("ue_sensor_ray_noise"),
                                               TEXT("ue_component"),
                                               TEXT("ue_base_link"),
                                               TEXT("ue_articulated_link"),
                                               TEXT("ue_wheel"),
                                               TEXT("ue_end_effector"),
                                               TEXT("ue_material_albedo"),
                                               TEXT("ue_material_orm"),
                                               TEXT("ue_material_normal")};

bool FRRURDFParser::ProcessAttribute(const TCHAR* InAttributeName, const TCHAR* InAttributeValueText)
{
    // (NOTE) Except critical error case, always return true to keep reading until end of file!
    FString attName(InAttributeName);
    FString attValueString(InAttributeValueText);

    if (!attName.IsEmpty() && !attValueString.IsEmpty())
    {
        TArray<int32> indexGroup;
        int32 max;

        FString elementStackTop = ElemStack.Top();
        FString elements = FString::Join(ElemStack, TEXT("_"));

        if (!attValueString.IsEmpty())
        {
            for (const auto& ueElement : UE_ELEMENT_LIST)
            {
                if (elements.Contains(ueElement))
                {
                    AttMap.Add(ComposeAttributeKey(ueElement, attName), attValueString);
                }
            }
        }

        if (elementStackTop.Equals(TEXT("robot")) && attName.Equals(TEXT("name")))
        {
            ModelName = attValueString;
        }
        else if (elementStackTop.Equals(TEXT("joint")))
        {
            // Look which element is the current one.
            const int32 indexTransmission = ElemStack.FindLast(TEXT("transmission"));
            const int32 indexJoint = ElemStack.FindLast(TEXT("joint"));
            const int32 indexLink = ElemStack.FindLast(TEXT("link"));
            const int32 indexUE = ElemStack.FindLast(TEXT("ue"));
            const int32 indexMaterial = ElemStack.FindLast(TEXT("material"));

            // Check current context
            indexGroup = {indexTransmission, indexJoint, indexLink, indexUE, indexMaterial};
            max = FMath::Max<int32>(indexGroup);

            // Handle special case 1 of joint tags within a transmission tag
            if ((ELEMENT_INDEX_NONE != indexTransmission) && (max == indexTransmission))
            {
                // When the parser later pops a joint tag from the stack, it needs to know that a joint
                // tag within transmission is a different case from a regular joint tag
                AttMap.Add(ComposeAttributeKey(elementStackTop, attName, TEXT("transmission")), attValueString);
            }

            // Handle special case 2 of joint tags within a gazebo tag
            else if ((ELEMENT_INDEX_NONE != indexUE) && (max == indexUE))
            {
                // When the parser later pops a joint tag from the stack, it needs to know that a
                // joint tag within transmission is a different case from a regular joint tag
                AttMap.Add(ComposeAttributeKey(elementStackTop, attName, TEXT("ue")), attValueString);
            }
            else
            {
                AttMap.Add(ComposeAttributeKey(elementStackTop, attName), attValueString);
            }
        }

        // Handles the sub categories for Links
        // (elementStackTop: origin, geometry(mesh, box, cylinder, sphere), material, mass, inertia)
        else if (!elementStackTop.Equals(TEXT("robot")) && !elementStackTop.Equals(TEXT("ue")) &&
                 !elementStackTop.Equals(TEXT("transmission")) && !elementStackTop.Equals(TEXT("link")) &&
                 (ElemStack.FindLast(TEXT("link")) > ElemStack.FindLast(TEXT("joint"))))
        {
            // Look which element is the current one.
            int32 indexVisual = ElemStack.FindLast(TEXT("visual"));
            int32 indexInertial = ElemStack.FindLast(TEXT("inertial"));
            int32 indexCollision = ElemStack.FindLast(TEXT("collision"));

            // Check current context
            indexGroup = {indexVisual, indexInertial, indexCollision};
            max = FMath::Max<int32>(indexGroup);
#if RAPYUTA_URDF_PARSER_DEBUG
            UE_LOG(
                LogRapyutaCore, VeryVerbose, TEXT("FRRURDFParser:: INDEX  %d %d %d"), indexVisual, indexInertial, indexCollision);
            UE_LOG(LogRapyutaCore, VeryVerbose, TEXT("FRRURDFParser::ProcessAttribute %s - %s"), *attName, *attValueString);
#endif

            if ((ELEMENT_INDEX_NONE != indexVisual) && (max == indexVisual))
            {
                AttMap.Add(ComposeAttributeKey(elementStackTop, attName, GEOMETRY_TYPE_PREFIX_VISUAL), attValueString);
            }
            else if ((ELEMENT_INDEX_NONE != indexInertial) && (max == indexInertial))
            {
                AttMap.Add(ComposeAttributeKey(elementStackTop, attName, GEOMETRY_TYPE_PREFIX_INERTIAL), attValueString);
            }
            else if ((ELEMENT_INDEX_NONE != indexCollision) && (max == indexCollision))
            {
                AttMap.Add(ComposeAttributeKey(elementStackTop, attName, GEOMETRY_TYPE_PREFIX_COLLISION), attValueString);
            }
            else
            {
                UE_LOG(LogRapyutaCore, Error, TEXT("Unexpected attribute in robot description %s - %s"), *attName, *attValueString);
            }
        }
        else if (elementStackTop.Equals(TEXT("sensor")) || elementStackTop.Equals(TEXT("component")) ||
                 elementStackTop.Equals(TEXT("base_link")) || elementStackTop.Equals(TEXT("articulated_link")) ||
                 elementStackTop.Equals(TEXT("wheel")) || elementStackTop.Equals(TEXT("end_effector")) ||
                 elementStackTop.Equals(TEXT("material")))
        {
            AttMap.Add(ComposeAttributeKey(elementStackTop, attName, TEXT("ue")), attValueString);
        }
        else if (elementStackTop.Equals(TEXT("albedo")) || elementStackTop.Equals(TEXT("orm")) ||
                 elementStackTop.Equals(TEXT("normal")))
        {
            AttMap.Add(ComposeAttributeKey(elementStackTop, attName, TEXT("ue_material")), attValueString);
        }
        else if (elementStackTop.Equals(TEXT("ray")) || elementStackTop.Equals(TEXT("lidar")))
        {
            AttMap.Add(ComposeAttributeKey(elementStackTop, attName, TEXT("ue_sensor")), attValueString);
        }
        else if (elementStackTop.Equals(TEXT("horizontal")))
        {
            AttMap.Add(ComposeAttributeKey(elementStackTop, attName, TEXT("ue_sensor_scan")), attValueString);
        }
        else if (elementStackTop.Equals(TEXT("range")))
        {
            AttMap.Add(ComposeAttributeKey(elementStackTop, attName, TEXT("ue_sensor_lidar")), attValueString);
        }
        else if (elementStackTop.Equals(TEXT("noise")))
        {
            AttMap.Add(ComposeAttributeKey(elementStackTop, attName, TEXT("ue_sensor_lidar")), attValueString);
        }
        else
        {
            // Add Stackelement from Top, Attributname and Attributvalue to a Map
            AttMap.Add(ComposeAttributeKey(elementStackTop, attName), attValueString);
        }
        return true;
    }
    else
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("AttributeName[%s] or AttributeValue[%s] empty"), *attName, *attValueString);
        return false;
    }
}

bool FRRURDFParser::ProcessElement(const TCHAR* InElementName, const TCHAR* InElementData, int32 XmlFileLineNumber)
{
    // (NOTE) Except critical error case, always return true to keep reading until end of file!
    const FString elementName(InElementName);
    const FString elementData(InElementData);

    if (!elementName.IsEmpty())
    {
        ElemStack.Push(elementName);
        FString elements = FString::Join(ElemStack, TEXT("_"));

        for (const auto& ueElement : UE_ELEMENT_LIST)
        {
            if (elements.Contains(ueElement) && !elementData.IsEmpty())
            {
                AttMap.Add(ComposeAttributeKey(ueElement, elementName), elementData);
            }
        }
        return true;
    }
    else
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("Empty element name encountered: %d"), XmlFileLineNumber);
        return false;
    }
}

bool FRRURDFParser::ProcessClose(const TCHAR* InElementName)
{
    // (NOTE) Except critical error case, always return true to keep reading until end of file!
    const FString elementStackTop = ElemStack.Top();
#if RAPYUTA_URDF_PARSER_DEBUG
    // For debugging only
    if (false == FString(InElementName).Equals(elementStackTop))
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("ProcessClose %s vs %s"), InElementName, *elementStackTop);
    }
#endif
    if (elementStackTop.IsEmpty())
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("[ProcessClose] ElementStack's Top() Name is empty"));
        return false;
    }

    bool bElementSupported = false;
    bool bResult = true;
    if (elementStackTop.Equals(TEXT("joint")))
    {
        bElementSupported = true;
        bResult = ParseJointProperty();
    }
    else if (elementStackTop.Equals(TEXT("link")))
    {
        bElementSupported = true;
        bResult = ParseLinkProperty();
    }
    // NOTE: <ue> tags must stay at the end of .urdf file, after all <link> ones & each cannot have duplicated child tags
    else if (elementStackTop.Equals(TEXT("ue")))
    {
        bElementSupported = true;
        // Parse link's material + sensor
        ParseUELinkMaterialAndSensorInfo();

        // Parse whole model's ue specifics
        ParseModelUESpecifics();
    }

    if (bElementSupported)
    {
        // Only with supported element, Clear [AttMap] so newly read attr would be recognized on a later parse
        AttMap.Reset();
    }

    // Pop elem out of the stack for the next read (ProcessClose)
    ElemStack.Pop();
    return bResult;
}

void FRRURDFParser::ParseUELinkMaterialAndSensorInfo()
{
    const FString ueRefLinkName = AttMap.FindRef(TEXT("ue_reference"));
    if (false == ueRefLinkName.IsEmpty())
    {
        for (auto& linkProp : LinkPropList)
        {
            if (linkProp.Name == ueRefLinkName)
            {
                // Parse material prop
                for (auto& linkVisual : linkProp.VisualList)
                {
                    linkVisual.MaterialInfo.Name = AttMap.FindRef(TEXT("ue_material_name"));
                }

                // Parse sensor property
                FRRSensorProperty sensorProp;
                ParseSensorProperty(sensorProp);
                linkProp.SensorList.Emplace(MoveTemp(sensorProp));

                break;
            }
        }
    }
}

void FRRURDFParser::ParseModelUESpecifics()
{
    // NOTE: This runs under a SINGLE <ue> tag only -> run MULTIPLE TIMES, which is why check for Empty/INDEX_NONE is essential
    // 1- Model custom UE components (diff/manipulator/chaos_wheel drive, etc.)
    FString componentTypeValueStr = AttMap.FindRef(TEXT("ue_component_type"));
    const int8 componentTypeVal = URRTypeUtils::GetEnumValueFromString(TEXT("ERRUEComponentType"), componentTypeValueStr);
    if (INDEX_NONE != componentTypeVal)
    {
        UEComponentTypeFlags |= componentTypeVal;
    }

    // 2- Base (main body) link
    FString baseLinkName = AttMap.FindRef(TEXT("ue_base_link_name"));
    if (false == baseLinkName.IsEmpty())
    {
        BaseLinkName = MoveTemp(baseLinkName);
    }

    // 3- ArticulatedLinks: only require parsing for ARTICULATION_DRIVE compound type
    if (false == IsPlainManipulatorModel())
    {
        FString arLinkName = AttMap.FindRef(TEXT("ue_articulated_link_name"));
        if (!arLinkName.IsEmpty())
        {
            ArticulatedLinksNames.Emplace(MoveTemp(arLinkName));
        }
    }
    // else all links would be automatically added to [ArticulatedLinksNames] later in the end when all <ue> have been processed

    // 4- Wheels: only require parsing for WHEEL_DRIVE compound type
    if (false == IsPlainWheeledVehicleModel())
    {
        FString wheelName = AttMap.FindRef(TEXT("ue_wheel_name"));
        if (!wheelName.IsEmpty())
        {
            WheelPropList.Emplace(FRRRobotWheelProperty(MoveTemp(wheelName)));
        }
    }
    // else all links would be automatically added to [WheelsNames] later in the end when all <ue> have been processed

    // 5- EndEffectors
    FString eeName = AttMap.FindRef(TEXT("ue_end_effector_name"));
    if (!eeName.IsEmpty())
    {
        EndEffectorNames.Add(eeName);
    }

    // 6- WholeBody's material
    WholeBodyMaterialInfo.Name = AttMap.FindRef(TEXT("ue_material_name"));
    FString albedo = AttMap.FindRef(TEXT("ue_material_albedo_name"));
    if (!albedo.IsEmpty())
    {
        WholeBodyMaterialInfo.AlbedoTextureNameList.Emplace(MoveTemp(albedo));
    }
    WholeBodyMaterialInfo.ORMTextureName = AttMap.FindRef(TEXT("ue_material_orm_name"));
    WholeBodyMaterialInfo.NormalTextureName = AttMap.FindRef(TEXT("ue_material_normal_name"));
}

bool FRRURDFParser::ParseJointProperty()
{
    // (NOTE) Except critical error case, always return true to keep reading until end of file!
    const FString jointName = AttMap.FindRef(TEXT("joint_name"));
    const FString jointTypeName = AttMap.FindRef(TEXT("joint_type"));
    const FString parentLinkName = AttMap.FindRef(TEXT("parent_link"));
    const FString childLinkName = AttMap.FindRef(TEXT("child_link"));

    // If a required value is missing.
    if (jointName.IsEmpty() || jointTypeName.IsEmpty())
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("Missing required attribute for joint [%s] type [%s]"), *jointName, *jointTypeName);
        return true;
    }

    if (parentLinkName.IsEmpty() || childLinkName.IsEmpty())
    {
        UE_LOG(LogRapyutaCore,
               Warning,
               TEXT("Missing parent link [%s] or child link [%s] for joint [%s]"),
               *jointName,
               *parentLinkName,
               *childLinkName);
        return true;
    }

    // UE does not handle world-link joint
    if (parentLinkName == TEXT("world"))
    {
        bHasWorldJoint = true;
#if RAPYUTA_URDF_PARSER_DEBUG
        UE_LOG(LogRapyutaCore, Warning, TEXT("Ignore attached-to-world-link joint"));
#endif
        return true;
    }

    FString mimicJointName = AttMap.FindRef(TEXT("mimic_joint"));
    float mimicMultipler = 1.0f;
    float mimicOffset = 0.0f;

    float limitUpper = 0.f;
    float limitLower = 0.f;

    FVector jointLocation = FVector::ZeroVector;
    FQuat jointRotation = FQuat::Identity;
    FVector jointAxis = FVector::XAxisVector;

    // For Location and Rotation...if not, then default value.
    if (AttMap.Contains(TEXT("origin_xyz")))
    {
        jointLocation = ParseVector(TEXT("origin_xyz"));
    }
    if (AttMap.Contains(TEXT("origin_rpy")))
    {
        jointRotation = ParseRotation(TEXT("origin_rpy"));
    }
    if (AttMap.Contains(TEXT("axis_xyz")))
    {
        jointAxis = ParseVector(TEXT("axis_xyz"), false);
    }
    limitLower = FCString::Atof(*(AttMap.FindRef(TEXT("limit_lower"))));
    limitUpper = FCString::Atof(*(AttMap.FindRef(TEXT("limit_upper"))));
    mimicMultipler = FCString::Atof(*(AttMap.FindRef("mimic_multiplier")));
    mimicOffset = FCString::Atof(*(AttMap.FindRef(TEXT("mimic_offset"))));

    // Required at prismatic and revolute
    if ((jointTypeName.Equals(TEXT("prismatic")) || jointTypeName.Equals(TEXT("revolute"))) &&
        (AttMap.FindRef(TEXT("limit_effort")).IsEmpty() || AttMap.FindRef(TEXT("limit_velocity")).IsEmpty()))
    {
        UE_LOG(LogRapyutaCore, Warning, TEXT("Missing required data for prismatic/revolute joint."));
        return true;
    }

    // Creates the joint and set the values of the struct.
    FRRRobotJointProperty newJointProp;
    newJointProp.Name = jointName;
    newJointProp.Type = FRRRobotJointProperty::GetERRRobotJointTypeValueFromString(TCHAR_TO_UTF8(*jointTypeName));
    newJointProp.Location = jointLocation;
    newJointProp.Rotation = jointRotation;
    newJointProp.ParentLinkName = parentLinkName;
    newJointProp.ChildLinkName = childLinkName;

    newJointProp.MimicJointName = mimicJointName;
    newJointProp.MimicMultiplier = mimicMultipler;
    newJointProp.MimicOffset = mimicOffset;

    newJointProp.Axis = jointAxis;
    newJointProp.AxisInParentFrame = jointRotation.RotateVector(jointAxis);
    newJointProp.LowerLimit = limitLower;
    newJointProp.UpperLimit = limitUpper;

    // Safety limits
    const float maxForceLimit = FCString::Atof(*AttMap.FindRef(TEXT("limit_effort")));
    if (maxForceLimit > 0.f)
    {
        newJointProp.DynamicParams.MaxForceLimit = maxForceLimit;
    }
    const float maxVelLimit = FCString::Atof(*AttMap.FindRef(TEXT("limit_velocity")));
    if (maxVelLimit > 0.f)
    {
        newJointProp.DynamicParams.MaxVelocity = maxVelLimit;
    }

    // Safety controller
    newJointProp.DynamicParams.KVelocity = FCString::Atof(*AttMap.FindRef(TEXT("safety_controller_k_velocity")));

    // Dynamics
    newJointProp.DynamicParams.SpringStiff = FCString::Atof(*AttMap.FindRef(TEXT("dynamics_stiffness")));
    newJointProp.DynamicParams.LimitSpringStiff = 2.f * newJointProp.DynamicParams.SpringStiff;
    newJointProp.DynamicParams.Damping = FCString::Atof(*AttMap.FindRef(TEXT("dynamics_damping")));
    newJointProp.DynamicParams.LimitDamping = 2.f * newJointProp.DynamicParams.Damping;
    newJointProp.DynamicParams.Friction = FCString::Atof(*AttMap.FindRef(TEXT("dynamics_friction")));

    // Add new joint data to the list
    JointPropList.Emplace(MoveTemp(newJointProp));

    return true;
}

bool FRRURDFParser::ParseLinkProperty()
{
    // (NOTE) Except critical error case, always return true to keep reading until end of file!
    const FString linkName = AttMap.FindRef(TEXT("link_name"));

    // Ignore empty & [world] links
    if (linkName.IsEmpty())
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("Missing link name in robot description."));
        return false;
    }
    else if (linkName.Equals(TEXT("world"), ESearchCase::IgnoreCase))
    {
#if RAPYUTA_URDF_PARSER_DEBUG
        UE_LOG(LogRapyutaCore, Warning, TEXT("Ignoring [world] link!"));
#endif
        return true;
    }

    // Create a new Link property struct
    FRRRobotLinkProperty newLinkProp;
    newLinkProp.Name = linkName;
    // ParentName is not specified in URDF link

    // Link Inerial elements
    FVector inertialLocation = FVector::ZeroVector;
    FQuat inertialRotation = FQuat::Identity;

    // Link Visual elements
    FVector visualLocation = FVector::ZeroVector;
    FQuat visualRotation = FQuat::Identity;
    FString visualMaterialName;
    FColor visualMaterialColor = FColor::Transparent;

    // Link Collision elements
    FVector collisionLocation = FVector::ZeroVector;
    FQuat collisionRotation = FQuat::Identity;

    // Set the optional origin xyz values under visual, inertial and collision
    if (AttMap.Contains(TEXT("inertial_origin_xyz")))
    {
        inertialLocation = ParseVector(TEXT("inertial_origin_xyz"));
    }
    if (AttMap.Contains(TEXT("visual_origin_xyz")))
    {
        visualLocation = ParseVector(TEXT("visual_origin_xyz"));
    }
    if (AttMap.Contains(TEXT("collision_origin_xyz")))
    {
        collisionLocation = ParseVector(TEXT("collision_origin_xyz"));
    }

    // set the optional origin rpy values of visual, inertial and collision
    if (AttMap.Contains(TEXT("inertial_origin_rpy")))
    {
        inertialRotation = ParseRotation(TEXT("inertial_origin_rpy"));
    }
    if (AttMap.Contains(TEXT("visual_origin_rpy")))
    {
        visualRotation = ParseRotation(TEXT("visual_origin_rpy"));
    }
    if (AttMap.Contains(TEXT("collision_origin_rpy")))
    {
        collisionRotation = ParseRotation(TEXT("collision_origin_rpy"));
    }

    // [INERTIA]
    newLinkProp.Inertia.Location = inertialLocation;
    newLinkProp.Inertia.Rotation = inertialRotation;
    newLinkProp.Inertia.Mass = FCString::Atof(*AttMap.FindRef(TEXT("inertial_mass_value")));
    newLinkProp.Inertia.Ixx = FCString::Atof(*AttMap.FindRef(TEXT("inertial_inertia_ixx")));
    newLinkProp.Inertia.Ixy = FCString::Atof(*AttMap.FindRef(TEXT("inertial_inertia_ixy")));
    newLinkProp.Inertia.Ixz = FCString::Atof(*AttMap.FindRef(TEXT("inertial_inertia_ixz")));
    newLinkProp.Inertia.Iyy = FCString::Atof(*AttMap.FindRef(TEXT("inertial_inertia_iyy")));
    newLinkProp.Inertia.Iyz = FCString::Atof(*AttMap.FindRef(TEXT("inertial_inertia_iyz")));
    newLinkProp.Inertia.Izz = FCString::Atof(*AttMap.FindRef(TEXT("inertial_inertia_izz")));

    // [VISUAL/COLLISION]
    FRRRobotGeometryInfo visualGeometryInfo;

    if (ParseGeometryInfo(linkName, ERRRobotGeometryType::VISUAL, visualGeometryInfo))
    {
        const FString& visualMeshName = visualGeometryInfo.MeshName;
        // [Visual's Geometry] --
        visualGeometryInfo.MeshName = visualMeshName;
        visualGeometryInfo.Location = visualLocation;
        visualGeometryInfo.Rotation = visualRotation;
#if RAPYUTA_URDF_PARSER_DEBUG
        UE_LOG(LogRapyutaCore, Warning, TEXT("URDF: VISUAL MESH FULL NAME: %s"), *visualMeshName);
#endif

        // !NOTE: newLinkProp.Location & newLinkProp.Rotation would be read from the joint that has NewLink as child link
        // --> Refer to FRRRobotModelInfo::UpdateLinksTransformInfoFromJoints()
        // Visual Color element
        if (AttMap.Contains(TEXT("visual_color_rgba")))
        {
            TArray<FString> visualColorArray;

            AttMap.FindRef(TEXT("visual_color_rgba")).ParseIntoArray(visualColorArray, URRActorCommon::SPACE_STR, true);
            visualMaterialColor = FColor(FCString::Atof(*visualColorArray[0]),
                                         FCString::Atof(*visualColorArray[1]),
                                         FCString::Atof(*visualColorArray[2]),
                                         FCString::Atof(*visualColorArray[3]));
        }

        // Visual Material element
        if (AttMap.Contains(TEXT("visual_material_name")))
        {
            visualMaterialName = AttMap.FindRef(TEXT("visual_material_name"));
        }

        visualGeometryInfo.MaterialInfo.Name = visualMaterialName;
        visualGeometryInfo.MaterialInfo.Color = visualMaterialColor;
        newLinkProp.VisualList.Emplace(MoveTemp(visualGeometryInfo));
    }

    FRRRobotGeometryInfo collisionGeometryInfo;
    bool isCollisionParsed = ParseGeometryInfo(linkName, ERRRobotGeometryType::COLLISION, collisionGeometryInfo);
    const FString& collisionMeshName = collisionGeometryInfo.MeshName;

    // [Collision's Geometry] --
    collisionGeometryInfo.MeshName = collisionMeshName;
    collisionGeometryInfo.Location = collisionLocation;
    collisionGeometryInfo.Rotation = collisionRotation;
    newLinkProp.CollisionList.Emplace(MoveTemp(collisionGeometryInfo));
#if RAPYUTA_URDF_PARSER_DEBUG
    UE_LOG(LogRapyutaCore, Warning, TEXT("URDF: COLLISION MESH FULL NAME: %s"), *collisionMeshName);
#endif

    // Add new link prop to the list
    LinkPropList.Emplace(MoveTemp(newLinkProp));

    return true;
}

bool FRRURDFParser::ParseSensorProperty(FRRSensorProperty& OutSensorProp)
{
    // (NOTE) Except critical error case, always return true to keep reading until end of file!
    const FString sensorName = AttMap.FindRef(TEXT("ue_sensor_name"));
    const int8 sensorTypeVal = URRTypeUtils::GetEnumValueFromString(TEXT("ERRSensorType"), AttMap.FindRef(TEXT("ue_sensor_type")));
    const ERRSensorType sensorType =
        (INDEX_NONE == sensorTypeVal) ? ERRSensorType::NONE : static_cast<ERRSensorType>(sensorTypeVal);

    // Sensor general props
    OutSensorProp.SensorName = sensorName;
    OutSensorProp.SensorType = sensorType;

    switch (sensorType)
    {
        case ERRSensorType::LIDAR:
        {
            auto& outLidarInfo = OutSensorProp.LidarInfo;
            // Ray sensor props
            FString lidarSensorTypeName = AttMap.FindRef(TEXT("ue_sensor_ray_type"));
            if (lidarSensorTypeName.IsEmpty())
            {
                lidarSensorTypeName = AttMap.FindRef(TEXT("ue_sensor_lidar_type"));
            }
            outLidarInfo.LidarType = lidarSensorTypeName.Equals(TEXT("2d"), ESearchCase::IgnoreCase)   ? ERRLidarSensorType::TWO_D
                                     : lidarSensorTypeName.Equals(TEXT("3d"), ESearchCase::IgnoreCase) ? ERRLidarSensorType::THREE_D
                                                                                                       : ERRLidarSensorType::NONE;

            // Scan's horizontal + vertical
            outLidarInfo.NHorSamplesPerScan = FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_scan_horizontal_samples")));
            outLidarInfo.HMinAngle =
                FMath::RadiansToDegrees(FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_scan_horizontal_min_angle"))));
            outLidarInfo.HMaxAngle =
                FMath::RadiansToDegrees(FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_scan_horizontal_max_angle"))));
            outLidarInfo.HResolution = FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_scan_horizontal_resolution")));

            outLidarInfo.NVerSamplesPerScan = FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_scan_vertical_samples")));
            outLidarInfo.VMinAngle =
                FMath::RadiansToDegrees(FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_scan_vertical_min_angle"))));
            outLidarInfo.VMaxAngle =
                FMath::RadiansToDegrees(FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_scan_vertical_max_angle"))));
            outLidarInfo.VResolution = FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_scan_vertical_resolution")));

            // Range
            outLidarInfo.MinRange =
                URRConversionUtils::DistanceROSToUE(FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_range_min"))));
            outLidarInfo.MaxRange =
                URRConversionUtils::DistanceROSToUE(FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_range_max"))));
            outLidarInfo.RangeResolution = FCString::Atof(*AttMap.FindRef(TEXT("ue_sensor_ray_range_resolution")));

            // Noise
            outLidarInfo.NoiseTypeName = AttMap.FindRef(TEXT("ue_sensor_ray_noise_type"));

            outLidarInfo.NoiseMean = FCString::Atod(*AttMap.FindRef(TEXT("ue_sensor_ray_noise_mean")));
            outLidarInfo.NoiseStdDev = FCString::Atod(*AttMap.FindRef(TEXT("ue_sensor_ray_noise_stddev")));
        }
    }    // End switch (sensorType)

    return true;
}

FVector FRRURDFParser::ParseVector(const FString& InElementName, bool bIsForLocation)
{
    TArray<FString> vectorText;

    // Output Vector's meaning is different for spheres.
    if (InElementName.Contains(TEXT("sphere_radius")))
    {
        AttMap.FindRef(InElementName).ParseIntoArray(vectorText, URRActorCommon::SPACE_STR, true);
        float diameter = 2.f * FCString::Atof(*vectorText[0]);

        return URRConversionUtils::SizeROSToUE(FVector(diameter));
    }

    const FString elementText = AttMap.FindRef(InElementName);
    elementText.ParseIntoArray(vectorText, URRActorCommon::SPACE_STR, true);
    if (vectorText.Num() < 3)
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[FRRURDFParser::ParseVector] [%s]: %s does not represent a 3d vector value!"),
               *InElementName,
               *elementText);
    }
    const FVector urdfVector(FCString::Atof(*vectorText[0]), FCString::Atof(*vectorText[1]), FCString::Atof(*vectorText[2]));
    return bIsForLocation ? URRConversionUtils::VectorROSToUE(urdfVector) : urdfVector;
}

FTransform FRRURDFParser::ParsePose(const FString& InElementName)
{
    TArray<FString> poseText;

    const FString elementText = AttMap.FindRef(InElementName);
    elementText.ParseIntoArray(poseText, URRActorCommon::SPACE_STR, true);
    if (poseText.Num() < 6)
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[FRRURDFParser::ParsePose] [%s]: %s does not represent a 6d pose value!"),
               *InElementName,
               *elementText);
    }
    const FVector translation(FCString::Atof(*poseText[0]), FCString::Atof(*poseText[1]), FCString::Atof(*poseText[2]));

    // URDF: rpy
    // FRotator(pitch, yaw, roll)
    const FRotator rotation(FCString::Atof(*poseText[4]), FCString::Atof(*poseText[5]), FCString::Atof(*poseText[3]));

    return FTransform(rotation, translation);
}

FQuat FRRURDFParser::ParseRotation(const FString& InElementName)
{
    TArray<FString> rotationText;

    //**********************************************************************
    // In URDF the order of axes might be different to the order in UE4
    //
    // radius: x*2,y*2   length: z
    // size z, y, x
    // rotation (rpy) roll, pitch, yaw (UE4 pitch, yaw, roll)
    //**********************************************************************
    const FString elementText = AttMap.FindRef(InElementName);
    elementText.ParseIntoArray(rotationText, URRActorCommon::SPACE_STR, true);
    if (rotationText.Num() < 3)
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[FRRURDFParser::ParseRotation] [%s]: %s does not represent a 3d rotation value!"),
               *InElementName,
               *elementText);
    }

    float urdf_R = FMath::RadiansToDegrees(FCString::Atof(*rotationText[0]));
    float urdf_P = FMath::RadiansToDegrees(FCString::Atof(*rotationText[1]));
    float urdf_Y = FMath::RadiansToDegrees(FCString::Atof(*rotationText[2]));

    return URRConversionUtils::QuatROSToUE(FQuat(FRotator(urdf_P, urdf_Y, urdf_R)));
}

FVector FRRURDFParser::ParseCylinderSize(const FString& InRadiusElementName, const FString& InLengthElementName)
{
    TArray<FString> radiusText, lengthText;

    // [Radius]
    const FString radiusElementText = AttMap.FindRef(InRadiusElementName);
    radiusElementText.ParseIntoArray(radiusText, URRActorCommon::SPACE_STR, true);
    if (radiusText.Num() < 1)
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[FRRURDFParser::ParseCylinderSize] [%s]: %s does not represent a radius value!"),
               *InRadiusElementName,
               *radiusElementText);
    }

    // [Length]
    const FString lengthElementText = AttMap.FindRef(InLengthElementName);
    lengthElementText.ParseIntoArray(lengthText, URRActorCommon::SPACE_STR, true);
    if (lengthText.Num() < 1)
    {
        UE_LOG(LogRapyutaCore,
               Error,
               TEXT("[FRRURDFParser::ParseCylinderSize] [%s]: %s does not represent a length value!"),
               *InLengthElementName,
               *lengthElementText);
    }

    float diameter = 2.f * FCString::Atof(*radiusText[0]);
    float length = FCString::Atof(*lengthText[0]);

    return URRConversionUtils::SizeROSToUE(FVector(diameter, diameter, length));
}

bool FRRURDFParser::ParseGeometryInfo(const FString& InLinkName,
                                      const ERRRobotGeometryType InGeometryType,
                                      FRRRobotGeometryInfo& OutGeometryInfo)
{
    bool parsed = true;
    OutGeometryInfo.LinkName = InLinkName;
    OutGeometryInfo.Name = InLinkName;

    const TCHAR* geometryTypePrefix = (ERRRobotGeometryType::VISUAL == InGeometryType)      ? GEOMETRY_TYPE_PREFIX_VISUAL
                                      : (ERRRobotGeometryType::COLLISION == InGeometryType) ? GEOMETRY_TYPE_PREFIX_COLLISION
                                                                                            : nullptr;
    check(geometryTypePrefix);

    // [BOX] --
    const FString boxSizeElementName = FString::Printf(TEXT("%s_box_size"), geometryTypePrefix);
    if (AttMap.Contains(boxSizeElementName))
    {
        OutGeometryInfo.LinkType = ERRShapeType::BOX;
        OutGeometryInfo.MeshName = TEXT("box");
        OutGeometryInfo.Size = ParseVector(boxSizeElementName, false);
    }
    else
    {
        // [CYLINDER] --
        const FString cylinderRadiusElementName = FString::Printf(TEXT("%s_cylinder_radius"), geometryTypePrefix);
        if (AttMap.Contains(cylinderRadiusElementName))
        {
            OutGeometryInfo.LinkType = ERRShapeType::CYLINDER;
            OutGeometryInfo.MeshName = TEXT("cylinder");
            OutGeometryInfo.Size =
                ParseCylinderSize(cylinderRadiusElementName, FString::Printf(TEXT("%s_cylinder_length"), geometryTypePrefix));
        }
        else
        {
            // [SPHERE] --
            const FString sphereRadiusElementName = FString::Printf(TEXT("%s_sphere_radius"), geometryTypePrefix);
            if (AttMap.Contains(sphereRadiusElementName))
            {
                OutGeometryInfo.LinkType = ERRShapeType::SPHERE;
                OutGeometryInfo.MeshName = TEXT("sphere");
                OutGeometryInfo.Size = ParseVector(sphereRadiusElementName, false);
            }
            else
            {
                // [MESH] --
                const FString meshFileElementName = FString::Printf(TEXT("%s_mesh_filename"), geometryTypePrefix);
                if (AttMap.Contains(meshFileElementName))
                {
                    OutGeometryInfo.LinkType = ERRShapeType::MESH;
                    OutGeometryInfo.MeshName = AttMap.FindRef(meshFileElementName);

                    const FString meshScaleElementName = FString::Printf(TEXT("%s_mesh_scale"), geometryTypePrefix);
                    if (AttMap.Contains(meshScaleElementName))
                    {
                        OutGeometryInfo.WorldScale = ParseVector(meshScaleElementName, false);
                    }
                }
                else
                {
                    parsed = false;
                }
            }
        }
    }

    return parsed;
}

FRRRobotModelInfo FRRURDFParser::LoadModelInfoFromFile(const FString& InURDFPath)
{
    FString outXMLContent;
    if (!FFileHelper::LoadFileToString(outXMLContent, *InURDFPath, FFileHelper::EHashOptions::None))
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("Failed reading URDF file [%s]"), *InURDFPath);
        return FRRRobotModelInfo();
    }
    outXMLContent = URRCoreUtils::GetSanitizedXMLString(outXMLContent);

    UE_LOG(LogRapyutaCore, Warning, TEXT("PARSE URDF CONTENT FROM FILE %s"), *InURDFPath);
    FRRRobotModelInfo robotModelInfo;
    robotModelInfo.ModelDescType = ERRRobotDescriptionType::URDF;
    robotModelInfo.DescriptionFilePath = InURDFPath;
    if (LoadModelInfoFromXML(outXMLContent, robotModelInfo))
    {
        robotModelInfo.UpdateLinksLocationFromJoints();
    }
    return robotModelInfo;
}

bool FRRURDFParser::LoadModelInfoFromXML(const FString& InUrdfXml, FRRRobotModelInfo& OutRobotModelInfo)
{
    if (InUrdfXml.IsEmpty())
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("!EMPTY URDF XML"));
        return false;
    }

    FText resultError;
    int32 resultLineNumb = -1;
    bool bResult = FFastXml::ParseXmlFile(static_cast<IFastXmlCallback*>(this),
                                          EMPTY_STR,
                                          const_cast<TCHAR*>(*InUrdfXml),
                                          nullptr,
                                          false,
                                          false,
                                          resultError,
                                          resultLineNumb);

    if (bResult)
    {
#if RAPYUTA_URDF_PARSER_DEBUG
        UE_LOG(LogRapyutaCore, Warning, TEXT("PARSING URDF SUCCEEDED[%s]!"), *ModelName);
#endif
        OutRobotModelInfo.ModelNameList.Emplace(MoveTemp(ModelName));
        OutRobotModelInfo.bHasWorldJoint = bHasWorldJoint;
        OutRobotModelInfo.UEComponentTypeFlags = UEComponentTypeFlags;

        // 0- BaseLink
        OutRobotModelInfo.BaseLinkName = MoveTemp(BaseLinkName);

        // 1- Links/Joints
        OutRobotModelInfo.LinkPropList = MoveTemp(LinkPropList);
        OutRobotModelInfo.JointPropList = MoveTemp(JointPropList);

        // 2- ArticulatedLinksNames
        // [Manipulator] as containing ARTICULATION_DRIVE only
        if (IsPlainManipulatorModel())
        {
            // By default all links are articulated for ARTICULATION_DRIVE-only manipulator type
            for (const auto& linkProp : OutRobotModelInfo.LinkPropList)
            {
                OutRobotModelInfo.ArticulatedLinksNames.Add(linkProp.Name);
            }
        }
        else
        {
            OutRobotModelInfo.ArticulatedLinksNames = MoveTemp(ArticulatedLinksNames);
        }

        // 3- WheelsNames
        OutRobotModelInfo.WheelPropList = MoveTemp(WheelPropList);

        // 4- EndEffectorNames
        OutRobotModelInfo.EndEffectorNames = MoveTemp(EndEffectorNames);

        // 4- WholeBodyMaterialInfo
        OutRobotModelInfo.WholeBodyMaterialInfo = MoveTemp(WholeBodyMaterialInfo);
    }
    else
    {
        UE_LOG(LogRapyutaCore, Error, TEXT("PARSING URDF FAILED[%s]: %s"), *ModelName, *resultError.ToString());
    }
    Reset();
    return bResult;
}
