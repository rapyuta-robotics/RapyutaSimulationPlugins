// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Core/RRConversionUtils.h"

FVector URRConversionUtils::VectorUEToROS(const FVector& Input)
{
    return ConversionUtils::VectorUEToROS(Input);
}

FVector URRConversionUtils::RotationUEToROS(const FVector& Input)
{
    return ConversionUtils::RotationUEToROS(Input);
}

FQuat URRConversionUtils::QuatUEToROS(const FQuat& Input)
{
    return ConversionUtils::QuatUEToROS(Input);
}

FTransform URRConversionUtils::TransformUEToROS(const FTransform& Input)
{
    return ConversionUtils::TransformUEToROS(Input);
}

FROSOdometry URRConversionUtils::OdomUEToROS(const FROSOdometry& Input)
{
    return ConversionUtils::OdomUEToROS(Input);
}

FVector URRConversionUtils::VectorROSToUE(const FVector& Input)
{
    return ConversionUtils::VectorROSToUE(Input);
}

FVector URRConversionUtils::RotationROSToUE(const FVector& Input)
{
    return ConversionUtils::RotationROSToUE(Input);
}

FQuat URRConversionUtils::QuatROSToUE(const FQuat& Input)
{
    return ConversionUtils::QuatROSToUE(Input);
}

FTransform URRConversionUtils::TransformROSToUE(const FTransform& Input)
{
    return ConversionUtils::TransformROSToUE(Input);
}

FROSOdometry URRConversionUtils::OdomROSToUE(const FROSOdometry& Input)
{
    return ConversionUtils::OdomROSToUE(Input);
}
