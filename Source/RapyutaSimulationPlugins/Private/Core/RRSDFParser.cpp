// Copyright 2020-2022 Rapyuta Robotics Co., Ltd.
#include "Core/RRSDFParser.h"

// ignition
#include "ignition/math/Color.hh"
#include "ignition/math/Pose3.hh"
#include "ignition/math/Vector3.hh"

// sdf
#include "sdf/ParserConfig.hh"
#include "sdf/sdf.hh"

// RapyutaSimRobotImporter
#include "Core/RRConversionUtils.h"
#include "Core/RRCoreUtils.h"
#include "Core/RRMeshData.h"

static inline FVector GetLocationFromIgnitionPose(const ignition::math::Pose3d& InIgnPose)
{
    const auto loc = InIgnPose.Pos();
    return URRConversionUtils::VectorROSToUE(FVector(loc.X(), loc.Y(), loc.Z()));
}

static inline FQuat GetRotationFromIgnitionPose(const ignition::math::Pose3d& InIgnPose)
{
    const auto rot = InIgnPose.Rot();
    return URRConversionUtils::QuatROSToUE(FQuat(rot.X(), rot.Y(), rot.Z(), rot.W()));
}

FRREntityModelInfo FRRSDFParser::LoadModelInfoFromFile(const FString& InSDFPath)
{
    // Specify path to the model file uri in <include>
    // InModelName: <model://ModelPath>
    sdf::setFindCallback(
        [InSDFPath](const std::string& InModelName)
        {
            return URRCoreUtils::FToStdString(FString::Printf(
                TEXT("%s%s"),
                *FRREntityDescriptionParser::GetRealPathFromMeshName(FString(InModelName.c_str()), FPaths::GetPath(InSDFPath)),
                URRCoreUtils::GetSimFileExt(ERRFileType::SDF)));
        });
    sdf::Errors outErrors;
    sdf::SDFPtr outSDFContent = sdf::readFile(URRCoreUtils::FToStdString(InSDFPath), outErrors);
    if (nullptr == outSDFContent)
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Failed reading SDF file [%s]"), *InSDFPath);
        for (const auto& error : outErrors)
        {
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Error,
                             TEXT("SDF Error: %d %s"),
                             static_cast<uint8>(error.Code()),
                             *URRCoreUtils::StdToFString(error.Message()));
        }
        return FRREntityModelInfo();
    }

#if RAPYUTA_SDF_PARSER_DEBUG
    UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("PARSE SDF CONTENT FROM FILE %s"), *InSDFPath);
#endif
    FRREntityModelInfo robotModelInfo;
    auto& robotModelData = robotModelInfo.Data;
    robotModelData.ModelDescType = ERREntityDescriptionType::SDF;
    robotModelData.DescriptionFilePath = InSDFPath;
    if (LoadModelInfoFromSDF(outSDFContent, robotModelInfo))
    {
        robotModelData.UpdateLinksLocationFromJoints();
#if RAPYUTA_SDF_PARSER_DEBUG
        UE_LOG_WITH_INFO(
            LogRapyutaCore, Warning, TEXT("PARSING SDF SUCCEEDED[%s]!"), *FString::Join(robotModelData.ModelNameList, TEXT(",")));
#endif
    }
    else
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("PARSING SDF FAILED[%s]!"), *InSDFPath);
    }
    return robotModelInfo;
}

bool FRRSDFParser::LoadModelInfoFromSDF(const sdf::SDFPtr& InSDFContent, FRREntityModelInfo& OutRobotModelInfo)
{
    const sdf::ElementPtr rootElement = InSDFContent->Root();
    const sdf::ElementPtr worldElement = rootElement->FindElement(SDF_ELEMENT_WORLD);
    const sdf::ElementPtr modelElement = rootElement->FindElement(SDF_ELEMENT_MODEL);
    if ((nullptr == worldElement) && (nullptr == modelElement))
    {
        UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("SDF top is NOT either World OR Model Element!"));
        return false;
    }

    bool bResult = true;
    FRREntityModelData& outRobotModelData = OutRobotModelInfo.Data;

    const sdf::ElementPtr topElement = rootElement->GetFirstElement();
    // World
    if (worldElement == topElement)
    {
        outRobotModelData.WorldName = URRCoreUtils::StdToFString(worldElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));

        // World's Child Models info
        bResult &= LoadChildModelsData(worldElement, outRobotModelData);
    }

    // Model
    else if (modelElement == topElement)
    {
        outRobotModelData.ModelNameList.Emplace(URRCoreUtils::StdToFString(modelElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME)));

        // Model's pose info
        bResult &= LoadPoseInfo(modelElement, outRobotModelData);

        // Model's UE components info
        bResult &= ParseModelUESpecifics(modelElement, outRobotModelData);

        // Model's links/joints info
        bResult &= LoadLinksJointsInfo(modelElement, outRobotModelData);

        // Model's Child Models info
        bResult &= LoadChildModelsData(modelElement, outRobotModelData);
    }

    // Clear temp read data
    ClearData();
    return bResult;
}

bool FRRSDFParser::LoadChildModelsData(const sdf::ElementPtr& InParentElement, FRREntityModelData& OutRobotModelData)
{
    bool bResult = true;
    // Read <model> tags
    sdf::ElementPtr childModelElement = InParentElement->FindElement(SDF_ELEMENT_MODEL);
    while (childModelElement)
    {
        const FString modelName = URRCoreUtils::StdToFString(childModelElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));

        // To make sure each of [ModelNameList] is unique
        FString modelFullName = FString::Printf(TEXT("%s%s"), *OutRobotModelData.WorldName, *modelName);
        ensure(false == OutRobotModelData.ModelNameList.Contains(modelName));
#if RAPYUTA_SDF_PARSER_DEBUG
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Model %s"), *modelName);
#endif

        FRREntityModelData modelData({modelFullName});
        modelData.ModelDescType = ERREntityDescriptionType::SDF;
        modelData.BaseFolderPath = OutRobotModelData.BaseFolderPath;

        // 1- ChildModel's pose info
        bResult &= LoadPoseInfo(childModelElement, modelData);

        // 2- ChildModel's Links/Joints info
        bResult &= LoadLinksJointsInfo(childModelElement, modelData);

        // 3- ChildModel's UE components info, which requires prior Links/Joints info
        bResult &= ParseModelUESpecifics(childModelElement, modelData);

        // Update [ChildModelsData] & [ModelNameList]
        OutRobotModelData.ChildModelsData.Emplace(MoveTemp(modelData));
        OutRobotModelData.ModelNameList.Emplace(MoveTemp(modelFullName));

        // Move to next <model>
        childModelElement = childModelElement->GetNextElement(SDF_ELEMENT_MODEL);
    }
    return true;
}

bool FRRSDFParser::LoadPoseInfo(const sdf::ElementPtr& InElement, FRREntityModelData& OutRobotModelData)
{
    const auto poseElement = InElement->FindElement(SDF_ELEMENT_POSE);
    if (poseElement)
    {
        OutRobotModelData.ParentFrameName = URRCoreUtils::StdToFString(poseElement->Get<std::string>(SDF_ELEMENT_ATTR_RELATIVE_TO));

        const auto poseOffset = poseElement->Get<ignition::math::Pose3d>();
        OutRobotModelData.RelativeTransform.SetLocation(GetLocationFromIgnitionPose(poseOffset));
        OutRobotModelData.RelativeTransform.SetRotation(GetRotationFromIgnitionPose(poseOffset));
    }
    return true;
}

bool FRRSDFParser::ParseModelUESpecifics(const sdf::ElementPtr& InModelElement, FRREntityModelData& OutRobotModelData)
{
    sdf::ElementPtr ueElement = InModelElement->FindElement(SDF_ELEMENT_UE);
    if (nullptr == ueElement)
    {
        // Not all models has UE element
        return true;
    }

    // 1- Model custom UE components (diff/manipulator/chaos_wheel drive, etc.)
    sdf::ElementPtr componentElement = ueElement->FindElement(SDF_ELEMENT_COMPONENT);
    while (componentElement)
    {
        const int8 componentTypeVal = URRTypeUtils::GetEnumValueFromString(
            TEXT("ERRUEComponentType"), URRCoreUtils::StdToFString(componentElement->Get<std::string>(SDF_ELEMENT_ATTR_TYPE)));
        if (componentTypeVal != INDEX_NONE)
        {
            OutRobotModelData.SetUEComponentEnabled(componentTypeVal);
        }

        componentElement = componentElement->GetNextElement(SDF_ELEMENT_COMPONENT);
    }

    // 2- Base link name
    if (auto baseLinkElement = ueElement->FindElement(SDF_ELEMENT_BASE_LINK))
    {
        OutRobotModelData.BaseLinkName = URRCoreUtils::StdToFString(baseLinkElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
    }

    // 3- Articulated links, which uses OutRobotModelData.UEComponentTypeFlags
    if (OutRobotModelData.IsPlainManipulatorModel())
    {
        // By default all links are articulated for ARTICULATION_DRIVE-only manipulator type
        for (const auto& linkProp : OutRobotModelData.LinkPropList)
        {
            OutRobotModelData.ArticulatedLinksNames.Add(linkProp.Name);
        }
    }
    else
    {
        // Read designated ones only
        sdf::ElementPtr articulatedElement = ueElement->FindElement(SDF_ELEMENT_ARTICULATED_LINK);
        while (articulatedElement)
        {
            FString arLinkName = URRCoreUtils::StdToFString(articulatedElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
            if (false == arLinkName.IsEmpty())
            {
                OutRobotModelData.ArticulatedLinksNames.Emplace(MoveTemp(arLinkName));
            }

            articulatedElement = articulatedElement->GetNextElement(SDF_ELEMENT_ARTICULATED_LINK);
        }
    }

    // 4- Wheels, which uses OutRobotModelData.UEComponentTypeFlags
    if (OutRobotModelData.IsPlainWheeledVehicleModel())
    {
        // By default all links are articulated for WHEEL_DRIVE-only vehicle type
        for (const auto& linkProp : OutRobotModelData.LinkPropList)
        {
            OutRobotModelData.WheelPropList.Emplace(FRRRobotWheelProperty(linkProp.Name));
        }
    }
    else
    {
        // Read designated ones only
        sdf::ElementPtr wheelElement = ueElement->FindElement(SDF_ELEMENT_WHEEL);
        while (wheelElement)
        {
            FString wheelName = URRCoreUtils::StdToFString(wheelElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
            if (false == wheelName.IsEmpty())
            {
                OutRobotModelData.WheelPropList.Emplace(FRRRobotWheelProperty(MoveTemp(wheelName)));
            }

            wheelElement = wheelElement->GetNextElement(SDF_ELEMENT_WHEEL);
        }
    }

    // 5- EndEffectors
    sdf::ElementPtr endEffectorElement = ueElement->FindElement(SDF_ELEMENT_END_EFFECTOR);
    while (endEffectorElement)
    {
        FString eeName = URRCoreUtils::StdToFString(endEffectorElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
        if (false == eeName.IsEmpty())
        {
            OutRobotModelData.EndEffectorNames.Emplace(MoveTemp(eeName));
        }

        endEffectorElement = endEffectorElement->GetNextElement(SDF_ELEMENT_END_EFFECTOR);
    }

    // 6- WholeBody's static mesh (for World model only to make use of pre-baked UE static mesh)
    sdf::ElementPtr staticMeshElement = ueElement->FindElement(SDF_ELEMENT_LINK_STATIC_MESH);
    if (staticMeshElement)
    {
        OutRobotModelData.WholeBodyStaticMeshName =
            URRCoreUtils::StdToFString(staticMeshElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
    }

    // 7- WholeBody's material
    auto& materialInfo = OutRobotModelData.WholeBodyMaterialInfo;
    sdf::ElementPtr materialElement = ueElement->FindElement(SDF_ELEMENT_LINK_MATERIAL);
    if (materialElement)
    {
        auto albedoElement = materialElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ALBEDO);
        while (albedoElement)
        {
            materialInfo.AlbedoTextureNameList.Emplace(
                URRCoreUtils::StdToFString(albedoElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME)));
            if (auto albedoColorElement = albedoElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ALBEDO_COLOR))
            {
                const auto albedoColor = albedoColorElement->Get<ignition::math::Color>();
                materialInfo.AlbedoColorList.Emplace(
                    FLinearColor(albedoColor.R(), albedoColor.G(), albedoColor.B(), albedoColor.A()));
            }
            albedoElement = albedoElement->GetNextElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ALBEDO);
        }
        if (auto maskElement = materialElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_MASK))
        {
            materialInfo.MaskTextureName = URRCoreUtils::StdToFString(maskElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
        }
        if (auto ormElement = materialElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ORM))
        {
            materialInfo.ORMTextureName = URRCoreUtils::StdToFString(ormElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
        }
        if (auto normalElement = materialElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_NORMAL))
        {
            materialInfo.NormalTextureName = URRCoreUtils::StdToFString(normalElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
        }
    }

    return true;
}

bool FRRSDFParser::LoadLinksJointsInfo(const sdf::ElementPtr& InParentElement, FRREntityModelData& OutRobotModelData)
{
    // Link elements
    bool bResult = ParseLinksProperty(InParentElement, OutRobotModelData);
    if (bResult)
    {
        // Joint elements
        bResult = ParseJointsProperty(InParentElement, OutRobotModelData);
    }
    return bResult;
}

bool FRRSDFParser::ParseLinksProperty(const sdf::ElementPtr& InModelElement, FRREntityModelData& OutRobotModelData)
{
    sdf::ElementPtr linkElement = InModelElement->FindElement(SDF_ELEMENT_LINK);
    while (linkElement)
    {
        const FString linkName = URRCoreUtils::StdToFString(linkElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
        if (linkName.IsEmpty())
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Error, TEXT("Missing name in link element."));
            linkElement = linkElement->GetNextElement(SDF_ELEMENT_LINK);
            continue;
        }
        else if (linkName.Equals(TEXT("world"), ESearchCase::IgnoreCase))
        {
            UE_LOG_WITH_INFO(LogRapyutaCore, Verbose, TEXT("Ignore [world] link!"));
            linkElement = linkElement->GetNextElement(SDF_ELEMENT_LINK);
            continue;
        }
#if RAPYUTA_SDF_PARSER_DEBUG
        UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Found [%s] link!"), *linkName);
#endif

        FRRRobotLinkProperty newLinkProp;
        newLinkProp.Name = linkName;

        // Link pose --
        if (sdf::ElementPtr linkPoseElement = linkElement->FindElement(SDF_ELEMENT_POSE))
        {
            newLinkProp.ParentFrameName =
                URRCoreUtils::StdToFString(linkPoseElement->Get<std::string>(SDF_ELEMENT_ATTR_RELATIVE_TO));

            const auto linkPose = linkPoseElement->Get<ignition::math::Pose3d>();
            newLinkProp.Location = GetLocationFromIgnitionPose(linkPose);
            newLinkProp.Rotation = GetRotationFromIgnitionPose(linkPose);
        }

        // Link's visual/collision --
        ParseGeometryInfo(
            linkElement, ERREntityGeometryType::VISUAL, newLinkProp.Location, newLinkProp.Rotation, newLinkProp.VisualList);
        ParseGeometryInfo(
            linkElement, ERREntityGeometryType::COLLISION, newLinkProp.Location, newLinkProp.Rotation, newLinkProp.CollisionList);

        // Link's inertia --
        if (sdf::ElementPtr inertialElement = linkElement->FindElement(SDF_ELEMENT_LINK_INERTIAL))
        {
            newLinkProp.Inertia.Mass = inertialElement->Get<float>(SDF_ELEMENT_LINK_INERTIAL_MASS);

            // (Note) This is the pose of the inertial reference frame.
            // The origin of the inertial reference frame needs to be at the center of gravity.
            // The axes of the inertial reference frame do not need to be aligned with the principal axes of the inertia.
            sdf::ElementPtr poseElement = inertialElement->FindElement(SDF_ELEMENT_POSE);

            if (poseElement)
            {
                const auto inertialPose = inertialElement->Get<ignition::math::Pose3d>(SDF_ELEMENT_POSE);
                newLinkProp.Inertia.Location = GetLocationFromIgnitionPose(inertialPose);
                newLinkProp.Inertia.Rotation = GetRotationFromIgnitionPose(inertialPose);
            }
            else
            {
                newLinkProp.Inertia.Location = newLinkProp.Location;
                newLinkProp.Inertia.Rotation = newLinkProp.Rotation;
            }

            if (sdf::ElementPtr inertiaElement = inertialElement->FindElement(SDF_ELEMENT_LINK_INERTIA))
            {
                newLinkProp.Inertia.Ixx = inertiaElement->Get<float>(SDF_ELEMENT_INERTIA_IXX);
                newLinkProp.Inertia.Ixy = inertiaElement->Get<float>(SDF_ELEMENT_INERTIA_IXY);
                newLinkProp.Inertia.Ixz = inertiaElement->Get<float>(SDF_ELEMENT_INERTIA_IXZ);
                newLinkProp.Inertia.Iyy = inertiaElement->Get<float>(SDF_ELEMENT_INERTIA_IYY);
                newLinkProp.Inertia.Iyz = inertiaElement->Get<float>(SDF_ELEMENT_INERTIA_IYZ);
                newLinkProp.Inertia.Izz = inertiaElement->Get<float>(SDF_ELEMENT_INERTIA_IZZ);
            }
        }

        // Link's sensor --
        ParseSensorsProperty(linkElement, newLinkProp.SensorList);

        // Add new link prop to the list
        OutRobotModelData.LinkPropList.Emplace(MoveTemp(newLinkProp));

        linkElement = linkElement->GetNextElement(SDF_ELEMENT_LINK);
    }
    return true;
}

bool FRRSDFParser::ParseGeometryInfo(const sdf::ElementPtr& InLinkElement,
                                     const ERREntityGeometryType InGeometryType,
                                     const FVector& InLocationBase,
                                     const FQuat& InRotationBase,
                                     TArray<FRREntityGeometryInfo>& OutGeometryInfoList)
{
    const FString linkName = URRCoreUtils::StdToFString(InLinkElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
    const char* visualCollisionElementName = (ERREntityGeometryType::VISUAL == InGeometryType)      ? SDF_ELEMENT_LINK_VISUAL
                                             : (ERREntityGeometryType::COLLISION == InGeometryType) ? SDF_ELEMENT_LINK_COLLISION
                                                                                                    : nullptr;
    check(visualCollisionElementName);
    sdf::ElementPtr visualCollisionElement = InLinkElement->FindElement(visualCollisionElementName);

    while (visualCollisionElement)
    {
        FRREntityGeometryInfo geometryInfo;
        geometryInfo.LinkName = linkName;

        // Name
        geometryInfo.Name = URRCoreUtils::StdToFString(visualCollisionElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));

        // Pose
        sdf::ElementPtr poseElement = visualCollisionElement->FindElement(SDF_ELEMENT_POSE);

        if (poseElement)
        {
            auto pose = visualCollisionElement->Get<ignition::math::Pose3d>(SDF_ELEMENT_POSE);
            geometryInfo.Location = GetLocationFromIgnitionPose(pose);
            geometryInfo.Rotation = GetRotationFromIgnitionPose(pose);
        }
        else
        {
            geometryInfo.Location = InLocationBase;
            geometryInfo.Rotation = InRotationBase;
        }

        // Geometry
        sdf::ElementPtr geometryElement = visualCollisionElement->FindElement(SDF_ELEMENT_LINK_GEOMETRY);
        if (geometryElement)
        {
            //  MESH --
            sdf::ElementPtr meshElement = geometryElement->FindElement(SDF_ELEMENT_LINK_MESH);
            if (meshElement)
            {
                geometryInfo.LinkType = ERRShapeType::MESH;
                const auto meshURI = URRCoreUtils::StdToFString(meshElement->Get<std::string>(SDF_ELEMENT_LINK_URI));
                geometryInfo.MeshName = meshURI;
                if (meshElement->HasElement(SDF_ELEMENT_LINK_SCALE))
                {
                    const auto scale = meshElement->Get<ignition::math::Vector3d>(SDF_ELEMENT_LINK_SCALE);
                    geometryInfo.WorldScale = FVector(scale.X(), scale.Y(), scale.Z());
                }
            }

            // PLANE --
            else if (sdf::ElementPtr visualPlaneElement = geometryElement->FindElement(SDF_ELEMENT_LINK_PLANE))
            {
                geometryInfo.LinkType = ERRShapeType::PLANE;
                geometryInfo.MeshName = TEXT("plane");

                const auto planeSize = visualPlaneElement->Get<ignition::math::Vector2d>(SDF_ELEMENT_LINK_PLANE_SIZE);
                geometryInfo.Size = URRConversionUtils::SizeROSToUE(FVector(planeSize.X(), planeSize.Y(), 0.1f));
            }

            // BOX --
            else if (sdf::ElementPtr visualBoxElement = geometryElement->FindElement(SDF_ELEMENT_LINK_BOX))
            {
                geometryInfo.LinkType = ERRShapeType::BOX;
                geometryInfo.MeshName = TEXT("box");

                const auto boxSize = visualBoxElement->Get<ignition::math::Vector3d>(SDF_ELEMENT_LINK_BOX_SIZE);
                geometryInfo.Size = URRConversionUtils::SizeROSToUE(FVector(boxSize.X(), boxSize.Y(), boxSize.Z()));
            }

            // SPHERE --
            else if (sdf::ElementPtr sphereElement = geometryElement->FindElement(SDF_ELEMENT_LINK_SPHERE))
            {
                geometryInfo.LinkType = ERRShapeType::SPHERE;
                geometryInfo.MeshName = TEXT("sphere");

                const float sphereRadius = sphereElement->Get<float>(SDF_ELEMENT_LINK_SPHERE_RADIUS);
                geometryInfo.Size = 2.0f * URRConversionUtils::SizeROSToUE(FVector(sphereRadius));
            }

            // CYLINDER --
            else if (sdf::ElementPtr cylinderElement = geometryElement->FindElement(SDF_ELEMENT_LINK_CYLINDER))
            {
                geometryInfo.LinkType = ERRShapeType::CYLINDER;
                geometryInfo.MeshName = TEXT("cylinder");

                const float cylRadius = cylinderElement->Get<float>(SDF_ELEMENT_LINK_CYLINDER_RADIUS);
                const float cylLength = cylinderElement->Get<float>(SDF_ELEMENT_LINK_CYLINDER_LENGTH);
                geometryInfo.Size = URRConversionUtils::SizeROSToUE(FVector(cylRadius, cylRadius, cylLength));
            }
        }

        // Material
        auto& materialInfo = geometryInfo.MaterialInfo;
        sdf::ElementPtr materialElement = visualCollisionElement->FindElement(SDF_ELEMENT_LINK_MATERIAL);
        if (materialElement)
        {
            if (auto scriptElement = materialElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_SCRIPT))
            {
                if (auto scriptNameElement = scriptElement->FindElement(SDF_ELEMENT_ATTR_NAME))
                {
                    materialInfo.Name = URRCoreUtils::StdToFString(scriptNameElement->Get<std::string>());
                }
            }

            const auto color = materialElement->Get<ignition::math::Color>(SDF_ELEMENT_LINK_MATERIAL_DIFFUSE);
            materialInfo.Color = FLinearColor(color.R(), color.G(), color.B(), color.A());

            auto albedoElement = materialElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ALBEDO);
            while (albedoElement)
            {
                materialInfo.AlbedoTextureNameList.Emplace(
                    URRCoreUtils::StdToFString(albedoElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME)));
                if (auto albedoColorElement = albedoElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ALBEDO_COLOR))
                {
                    const auto albedoColor = albedoColorElement->Get<ignition::math::Color>();
                    materialInfo.AlbedoColorList.Emplace(
                        FLinearColor(albedoColor.R(), albedoColor.G(), albedoColor.B(), albedoColor.A()));
                }
                albedoElement = albedoElement->GetNextElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ALBEDO);
            }
            if (auto maskElement = materialElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_MASK))
            {
                materialInfo.MaskTextureName = URRCoreUtils::StdToFString(maskElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
            }
            if (auto ormElement = materialElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ORM))
            {
                materialInfo.ORMTextureName = URRCoreUtils::StdToFString(ormElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
            }
            if (auto normalElement = materialElement->FindElement(SDF_ELEMENT_LINK_MATERIAL_TEXTURE_NORMAL))
            {
                materialInfo.NormalTextureName = URRCoreUtils::StdToFString(normalElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
            }
        }

        // Auto assign a link's empty-mesh name -> ["box"]
        if ((ERREntityGeometryType::VISUAL == InGeometryType) && geometryInfo.MeshName.IsEmpty())
        {
            geometryInfo.MeshName = TEXT("box");
            geometryInfo.LinkType = ERRShapeType::BOX;
            UE_LOG_WITH_INFO(LogRapyutaCore,
                             Warning,
                             TEXT("Missing mesh name for custom mesh in robot description for [%s] -> Auto assigned as [Box]"),
                             *geometryInfo.LinkName);
        }
        // Add new geometry info prop to the list
        OutGeometryInfoList.Emplace(MoveTemp(geometryInfo));

        visualCollisionElement = visualCollisionElement->GetNextElement(visualCollisionElementName);
    }

    return true;
}

bool FRRSDFParser::ParseSensorsProperty(const sdf::ElementPtr& InLinkElement, TArray<FRRSensorProperty>& OutSensorPropList)
{
    const FString linkName = URRCoreUtils::StdToFString(InLinkElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
    sdf::ElementPtr sensorElement = InLinkElement->FindElement(SDF_ELEMENT_LINK_SENSOR);
    while (sensorElement)
    {
        FRRSensorProperty sensorProp;
        sensorProp.LinkName = linkName;

        // Sensor's name & type
        sensorProp.SensorName = URRCoreUtils::StdToFString(sensorElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
        const FString sensorTypeName = URRCoreUtils::StdToFString(sensorElement->Get<std::string>(SDF_ELEMENT_ATTR_TYPE));
        const int8 sensorTypeVal = URRTypeUtils::GetEnumValueFromString(TEXT("ERRSensorType"), sensorTypeName);
        sensorProp.SensorType = (INDEX_NONE == sensorTypeVal) ? ERRSensorType::NONE : static_cast<ERRSensorType>(sensorTypeVal);

        // [LIDAR] --
        if (ERRSensorType::LIDAR == sensorProp.SensorType)
        {
            auto& lidarInfo = sensorProp.LidarInfo;

            lidarInfo.TopicName = URRCoreUtils::StdToFString(sensorElement->Get<std::string>(SDF_ELEMENT_SENSOR_TOPIC));
            ensure(false == lidarInfo.TopicName.IsEmpty());
            lidarInfo.FrameId = URRCoreUtils::StdToFString(sensorElement->Get<std::string>(SDF_ELEMENT_SENSOR_FRAME_ID));
            lidarInfo.PublicationFrequencyHz = sensorElement->Get<float>(SDF_ELEMENT_SENSOR_UPDATE_RATE);

            // Make sure <lidar>/<ray> exists
            sdf::ElementPtr rayLidarElement = sensorElement->FindElement(SDF_ELEMENT_SENSOR_LIDAR);
            if (nullptr == rayLidarElement)
            {
                rayLidarElement = sensorElement->FindElement(SDF_ELEMENT_SENSOR_RAY);
            }
            ensure(rayLidarElement.get());

            // (NOTE) SDF does not support adding a custom element or attribute type, thus must make use of [SensorName]
            lidarInfo.LidarType = sensorProp.SensorName.EndsWith(TEXT("2d"))   ? ERRLidarSensorType::TWO_D
                                  : sensorProp.SensorName.EndsWith(TEXT("3d")) ? ERRLidarSensorType::THREE_D
                                                                               : ERRLidarSensorType::NONE;

            // Scan's horizontal + vertical
            sdf::ElementPtr scanElement = rayLidarElement->FindElement(SDF_ELEMENT_SENSOR_LIDAR_SCAN);
            sdf::ElementPtr horScanElement = scanElement->FindElement(SDF_ELEMENT_SENSOR_LIDAR_SCAN_HORIZONTAL);
            if (horScanElement)
            {
                lidarInfo.NHorSamplesPerScan = horScanElement->Get<int>(SDF_ELEMENT_SENSOR_LIDAR_SCAN_SAMPLES);
                lidarInfo.HMinAngle = FMath::RadiansToDegrees(horScanElement->Get<float>(SDF_ELEMENT_MIN_ANGLE));
                lidarInfo.HMaxAngle = FMath::RadiansToDegrees(horScanElement->Get<float>(SDF_ELEMENT_MAX_ANGLE));
                lidarInfo.HResolution = horScanElement->Get<float>(SDF_ELEMENT_RESOLUTION);
            }

            sdf::ElementPtr verScanElement = scanElement->FindElement(SDF_ELEMENT_SENSOR_LIDAR_SCAN_VERTICAL);
            if (verScanElement)
            {
                lidarInfo.NVerSamplesPerScan = verScanElement->Get<int>(SDF_ELEMENT_SENSOR_LIDAR_SCAN_SAMPLES);
                lidarInfo.VMinAngle = FMath::RadiansToDegrees(verScanElement->Get<float>(SDF_ELEMENT_MIN_ANGLE));
                lidarInfo.VMaxAngle = FMath::RadiansToDegrees(verScanElement->Get<float>(SDF_ELEMENT_MAX_ANGLE));
                lidarInfo.VResolution = verScanElement->Get<float>(SDF_ELEMENT_RESOLUTION);
            }

            // Range
            sdf::ElementPtr rangeElement = rayLidarElement->FindElement(SDF_ELEMENT_SENSOR_LIDAR_RANGE);
            if (rangeElement)
            {
                lidarInfo.MinRange = URRConversionUtils::DistanceROSToUE(rangeElement->Get<float>(SDF_ELEMENT_MIN));
                lidarInfo.MaxRange = URRConversionUtils::DistanceROSToUE(rangeElement->Get<float>(SDF_ELEMENT_MAX));
            }

            // Noise
            sdf::ElementPtr noiseElement = rayLidarElement->FindElement(SDF_ELEMENT_SENSOR_LIDAR_NOISE);
            if (noiseElement)
            {
                lidarInfo.NoiseTypeName =
                    URRCoreUtils::StdToFString(noiseElement->Get<std::string>(SDF_ELEMENT_SENSOR_LIDAR_NOISE_TYPE));

                lidarInfo.NoiseMean = noiseElement->Get<double>(SDF_ELEMENT_SENSOR_LIDAR_NOISE_MEAN);
                lidarInfo.NoiseStdDev = noiseElement->Get<double>(SDF_ELEMENT_SENSOR_LIDAR_NOISE_STDDEV);
            }
        }

        // Add new sensor prop to the list
        OutSensorPropList.Emplace(MoveTemp(sensorProp));

        sensorElement = sensorElement->GetNextElement(SDF_ELEMENT_LINK_SENSOR);
    }
    return true;
}

bool FRRSDFParser::ParseJointsProperty(const sdf::ElementPtr& InModelElement, FRREntityModelData& OutRobotModelData)
{
    sdf::ElementPtr jointElement = InModelElement->FindElement(SDF_ELEMENT_JOINT);
    while (jointElement)
    {
        const FString parentLinkName = URRCoreUtils::StdToFString(jointElement->Get<std::string>(SDF_ELEMENT_JOINT_PARENT));
        verify(false == parentLinkName.IsEmpty());

        if (parentLinkName.Equals(TEXT("world"), ESearchCase::IgnoreCase))
        {
#if RAPYUTA_SDF_PARSER_DEBUG
            UE_LOG_WITH_INFO(LogRapyutaCore, Warning, TEXT("Ignore attached-to-world-link joint!"));
#endif
            jointElement = jointElement->GetNextElement(SDF_ELEMENT_JOINT);
            OutRobotModelData.bHasWorldJoint = true;
            continue;
        }

        const FString jointName = URRCoreUtils::StdToFString(jointElement->Get<std::string>(SDF_ELEMENT_ATTR_NAME));
        const FString jointTypeName = URRCoreUtils::StdToFString(jointElement->Get<std::string>(SDF_ELEMENT_ATTR_TYPE));

        const FString childLinkName = URRCoreUtils::StdToFString(jointElement->Get<std::string>(SDF_ELEMENT_JOINT_CHILD));
        verify(false == childLinkName.IsEmpty());
#if RAPYUTA_SDF_PARSER_DEBUG
        UE_LOG_WITH_INFO(
            LogRapyutaCore, Warning, TEXT("Found Joint[%s] : %s link -> %s link"), *jointName, *parentLinkName, *childLinkName);
#endif

        // Creates the joint and set the values of the struct
        FRRRobotJointProperty newJointProp;
        newJointProp.Name = jointName;
        newJointProp.Type = FRRRobotJointProperty::GetERRRobotJointTypeValueFromString(jointTypeName);

        // Joint Pose --
        if (sdf::ElementPtr jointPoseElement = jointElement->FindElement(SDF_ELEMENT_POSE))
        {
            // Typically this would be the same as parent link name
            const FString parentBodyName =
                URRCoreUtils::StdToFString(jointPoseElement->Get<std::string>(SDF_ELEMENT_ATTR_RELATIVE_TO));
            if (false == parentLinkName.Equals(parentBodyName))
            {
                UE_LOG_WITH_INFO(LogRapyutaCore,
                                 Warning,
                                 TEXT("Joint[%s] has its parent link[%s] # parent body[%s]"),
                                 *jointName,
                                 *parentLinkName,
                                 *parentBodyName);
            }

            const auto jointPose = jointPoseElement->Get<ignition::math::Pose3d>();
            newJointProp.Location = GetLocationFromIgnitionPose(jointPose);
            newJointProp.Rotation = GetRotationFromIgnitionPose(jointPose);
            newJointProp.bIsTransformRelative = !parentBodyName.IsEmpty();
        }

        newJointProp.ParentLinkName = parentLinkName;
        newJointProp.ChildLinkName = childLinkName;

        // Joint Axis --
        if (sdf::ElementPtr jointAxisElement = jointElement->FindElement(SDF_ELEMENT_AXIS))
        {
            const auto jointAxis = jointAxisElement->Get<ignition::math::Vector3d>(SDF_ELEMENT_AXIS_XYZ);
            FVector axis = FVector(jointAxis.X(), jointAxis.Y(), jointAxis.Z());
            newJointProp.Axis = axis;
            newJointProp.AxisInParentFrame = newJointProp.Rotation.Inverse().RotateVector(axis);

            // Joint Limit --
            sdf::ElementPtr jointAxisLimitElement = jointAxisElement->FindElement(SDF_ELEMENT_LIMIT);
            if (jointAxisLimitElement)
            {
                newJointProp.LowerLimit = jointAxisLimitElement->Get<float>(SDF_ELEMENT_LIMIT_LOWER);
                newJointProp.UpperLimit = jointAxisLimitElement->Get<float>(SDF_ELEMENT_LIMIT_UPPER);
                const float maxEffortLimit = jointAxisLimitElement->Get<float>(SDF_ELEMENT_LIMIT_EFFORT);

                if (maxEffortLimit > 0.f)
                {
                    newJointProp.DynamicParams.MaxForceLimit = maxEffortLimit;
                }

                const float maxVelLimit = jointAxisLimitElement->Get<float>(SDF_ELEMENT_LIMIT_VELOCITY);

                if (maxVelLimit > 0.f)
                {
                    newJointProp.DynamicParams.MaxVelocity = maxVelLimit;
                }
            }

            // Joint Dynamics --
            if (sdf::ElementPtr jointDynamicsElement = jointAxisElement->FindElement(SDF_ELEMENT_DYNAMICS))
            {
                newJointProp.DynamicParams.Damping = jointDynamicsElement->Get<float>(SDF_ELEMENT_DYNAMICS_DAMPING);
                newJointProp.DynamicParams.Friction = jointDynamicsElement->Get<float>(SDF_ELEMENT_DYNAMICS_FRICTION);
                newJointProp.DynamicParams.SpringRef = jointDynamicsElement->Get<float>(SDF_ELEMENT_DYNAMICS_SPRING_REF);
                newJointProp.DynamicParams.SpringStiff = jointDynamicsElement->Get<float>(SDF_ELEMENT_DYNAMICS_SPRING_STIFF);
            }
        }

        // Add new joint data to the list
        OutRobotModelData.JointPropList.Emplace(MoveTemp(newJointProp));

        jointElement = jointElement->GetNextElement(SDF_ELEMENT_JOINT);
    }
    return true;
}
