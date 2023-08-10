// Copyright 2020-2023 Rapyuta Robotics Co., Ltd.
#pragma once

// sdformat
#include "sdf/sdf.hh"

// RapyutaSimulationPlugins
#include "Core/RREntityStructs.h"
#include "RapyutaSimulationPlugins.h"

#define RAPYUTA_SDF_PARSER_DEBUG (0)

/**
 * @brief [experimental] Parse SDF file and generate FRREntityModelInfo. Uses ignition library to parse SDF file.
 *
 */
class RAPYUTASIMULATIONPLUGINS_API FRRSDFParser : public FRREntityDescriptionParser
{
public:
    FRREntityModelInfo LoadModelInfoFromFile(const FString& InSDFPath) override;
    bool LoadModelInfoFromSDF(const sdf::SDFPtr& InSDFContent, FRREntityModelInfo& OutRobotModelInfo);

private:
    bool ParseModelUESpecifics(const sdf::ElementPtr& InModelElement, FRREntityModelData& OutRobotModelData);
    bool ParseGeometryInfo(const sdf::ElementPtr& InLinkElement,
                           const ERREntityGeometryType InGeometryType,
                           const FVector& InLocationBase,
                           const FQuat& InRotationBase,
                           TArray<FRREntityGeometryInfo>& OutGeometryInfoList);
    bool LoadChildModelsData(const sdf::ElementPtr& InParentElement, FRREntityModelData& OutRobotModelData);
    bool LoadPoseInfo(const sdf::ElementPtr& InElement, FRREntityModelData& OutRobotModelData);
    bool LoadLinksJointsInfo(const sdf::ElementPtr& InParentElement, FRREntityModelData& OutRobotModelData);
    bool ParseLinksProperty(const sdf::ElementPtr& InModelElement, FRREntityModelData& OutRobotModelData);
    bool ParseSensorsProperty(const sdf::ElementPtr& InLinkElement, TArray<FRRSensorProperty>& OutSensorPropList);
    bool ParseJointsProperty(const sdf::ElementPtr& InModelElement, FRREntityModelData& OutRobotModelData);

    static constexpr const char* SDF_ELEMENT_ATTR_NAME = "name";
    static constexpr const char* SDF_ELEMENT_ATTR_TYPE = "type";
    static constexpr const char* SDF_ELEMENT_ATTR_RELATIVE_TO = "relative_to";
    static constexpr const char* SDF_ELEMENT_WORLD = "world";
    static constexpr const char* SDF_ELEMENT_MODEL = "model";
    static constexpr const char* SDF_ELEMENT_POSE = "pose";
    static constexpr const char* SDF_ELEMENT_UE = "ue";
    static constexpr const char* SDF_ELEMENT_COMPONENT = "component";
    static constexpr const char* SDF_ELEMENT_BASE_LINK = "base_link";
    static constexpr const char* SDF_ELEMENT_ARTICULATED_LINK = "articulated_link";
    static constexpr const char* SDF_ELEMENT_WHEEL = "wheel";
    static constexpr const char* SDF_ELEMENT_END_EFFECTOR = "end_effector";
    static constexpr const char* SDF_ELEMENT_MIN_ANGLE = "min_angle";
    static constexpr const char* SDF_ELEMENT_MAX_ANGLE = "max_angle";
    static constexpr const char* SDF_ELEMENT_RESOLUTION = "resolution";
    static constexpr const char* SDF_ELEMENT_MIN = "min";
    static constexpr const char* SDF_ELEMENT_MAX = "max";

    static constexpr const char* SDF_ELEMENT_LINK = "link";
    static constexpr const char* SDF_ELEMENT_LINK_GRAVITY = "gravity";
    static constexpr const char* SDF_ELEMENT_LINK_INERTIAL = "inertial";
    static constexpr const char* SDF_ELEMENT_LINK_INERTIAL_MASS = "mass";
    static constexpr const char* SDF_ELEMENT_LINK_INERTIA = "inertia";
    static constexpr const char* SDF_ELEMENT_INERTIA_IXX = "ixx";
    static constexpr const char* SDF_ELEMENT_INERTIA_IXY = "ixy";
    static constexpr const char* SDF_ELEMENT_INERTIA_IXZ = "ixz";
    static constexpr const char* SDF_ELEMENT_INERTIA_IYY = "iyy";
    static constexpr const char* SDF_ELEMENT_INERTIA_IYZ = "iyz";
    static constexpr const char* SDF_ELEMENT_INERTIA_IZZ = "izz";

    static constexpr const char* SDF_ELEMENT_LINK_VISUAL = "visual";
    static constexpr const char* SDF_ELEMENT_LINK_COLLISION = "collision";
    static constexpr const char* SDF_ELEMENT_LINK_GEOMETRY = "geometry";
    static constexpr const char* SDF_ELEMENT_LINK_SENSOR = "sensor";
    // "lidar" and "ray" types are equivalent, the latter will be deprecated
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR = "lidar";
    static constexpr const char* SDF_ELEMENT_SENSOR_TOPIC = "topic";
    static constexpr const char* SDF_ELEMENT_SENSOR_FRAME_ID = "frame_id";
    static constexpr const char* SDF_ELEMENT_SENSOR_UPDATE_RATE = "update_rate";
    static constexpr const char* SDF_ELEMENT_SENSOR_RAY = "ray";
    // SDF seems not to support custom attr name, so one like "ue_sensor_type" is not recognized
    static constexpr const char* SDF_ELEMENT_ATTR_UE_SENSOR_TYPE = "type";
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR_SCAN = "scan";
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR_SCAN_SAMPLES = "samples";
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR_SCAN_HORIZONTAL = "horizontal";
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR_SCAN_VERTICAL = "vertical";
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR_RANGE = "range";
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR_NOISE = "noise";
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR_NOISE_TYPE = "type";
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR_NOISE_MEAN = "mean";
    static constexpr const char* SDF_ELEMENT_SENSOR_LIDAR_NOISE_STDDEV = "stddev";

    static constexpr const char* SDF_ELEMENT_LINK_MATERIAL = "material";
    static constexpr const char* SDF_ELEMENT_LINK_MATERIAL_SCRIPT = "script";
    static constexpr const char* SDF_ELEMENT_LINK_MATERIAL_DIFFUSE = "diffuse";

    static constexpr const char* SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ALBEDO = "albedo";
    static constexpr const char* SDF_ELEMENT_LINK_MATERIAL_TEXTURE_MASK = "mask";
    static constexpr const char* SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ALBEDO_COLOR = "color";
    static constexpr const char* SDF_ELEMENT_LINK_MATERIAL_TEXTURE_ORM = "orm";
    static constexpr const char* SDF_ELEMENT_LINK_MATERIAL_TEXTURE_NORMAL = "normal";

    static constexpr const char* SDF_ELEMENT_LINK_SCALE = "scale";
    static constexpr const char* SDF_ELEMENT_LINK_MESH = "mesh";
    static constexpr const char* SDF_ELEMENT_LINK_STATIC_MESH = "static_mesh";
    static constexpr const char* SDF_ELEMENT_LINK_URI = "uri";
    static constexpr const char* SDF_ELEMENT_LINK_CYLINDER = "cylinder";
    static constexpr const char* SDF_ELEMENT_LINK_CYLINDER_RADIUS = "radius";
    static constexpr const char* SDF_ELEMENT_LINK_CYLINDER_LENGTH = "length";
    static constexpr const char* SDF_ELEMENT_LINK_PLANE = "plane";
    static constexpr const char* SDF_ELEMENT_LINK_PLANE_SIZE = "size";
    static constexpr const char* SDF_ELEMENT_LINK_BOX = "box";
    static constexpr const char* SDF_ELEMENT_LINK_BOX_SIZE = "size";
    static constexpr const char* SDF_ELEMENT_LINK_SPHERE = "sphere";
    static constexpr const char* SDF_ELEMENT_LINK_SPHERE_RADIUS = "radius";

    static constexpr const char* SDF_ELEMENT_JOINT = "joint";
    static constexpr const char* SDF_ELEMENT_JOINT_PARENT = "parent";
    static constexpr const char* SDF_ELEMENT_JOINT_CHILD = "child";
    static constexpr const char* SDF_ELEMENT_AXIS = "axis";
    static constexpr const char* SDF_ELEMENT_AXIS_XYZ = "xyz";

    static constexpr const char* SDF_ELEMENT_LIMIT = "limit";
    static constexpr const char* SDF_ELEMENT_LIMIT_LOWER = "lower";
    static constexpr const char* SDF_ELEMENT_LIMIT_UPPER = "upper";
    static constexpr const char* SDF_ELEMENT_LIMIT_EFFORT = "effort";
    static constexpr const char* SDF_ELEMENT_LIMIT_VELOCITY = "velocity";

    static constexpr const char* SDF_ELEMENT_DYNAMICS = "dynamics";
    static constexpr const char* SDF_ELEMENT_DYNAMICS_DAMPING = "damping";
    static constexpr const char* SDF_ELEMENT_DYNAMICS_FRICTION = "friction";
    static constexpr const char* SDF_ELEMENT_DYNAMICS_SPRING_REF = "spring_reference";
    static constexpr const char* SDF_ELEMENT_DYNAMICS_SPRING_STIFF = "spring_stiffness";
};
