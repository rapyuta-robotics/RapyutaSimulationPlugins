// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Sensors/RRLidar.h"

ARRLidar::ARRLidar()
{
    LidarComponent =
        Cast<URRBaseLidarComponent>(CreateDefaultSubobject(TEXT("Lidar"), LidarComponentClass, LidarComponentClass, true, false));
}
