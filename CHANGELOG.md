# Changelog for RapyutaSimulationPlugins repository

## 0.0.5 ##
* Add Content/SkeletalRobots/turtlebot3 BP actor classes for BallCasterSphereWheeled, ConvexWheeled, SphereWheeled, StaticMeshConstrained, FullLockConstrained, WheeledVehicle types
* Add Content/Robots & SkeletalRobots to lfs
* FRRMeshNodeData::BoneInfluences: `TArray<FRRBoneInfluenceList> -> TArray<FRRBoneInfluence>`
* Fix teleport option to get correct collision management #63
* Add default subscirbes #61
* Joint controller #59
* reset the array ContactPoints #60
* add missing InitMovementComponent to BeginPlay of RobotVehicleMovementComponent #58
* Sphinx and doxygen documentation #56 #66
* Add SpawnEntities and followFloor #57

## 0.0.4 ##
* Add the class `ARobotBaseVehicle` without skeletal mesh as root component. Class `ARobotVehicle` inherits from it.

## 0.0.3 ##
* Add Assimp as 3rd dependency (release sources + lib)
* Add URRProceduralMeshComponent & mesh util classes: `FRRMeshData, URRMeshUtils`

## 0.0.2 ##
* Add core sources (`URRActorCommon, URRObjectCommon, ARRBaseActor, ARRMeshActor, ARRSceneDirector, URRStaticMeshComponent, ARRGameState, URRGameInstance, ARRPlayerController, ~Utils`)
* `ARRRobotVehicle`: Add `InitMoveComponent()`, allowing `RobotVehicleMoveComponent` to be custom-initialized in child classes
* `URRGameSingleton`: Add skeletal mesh related resource types (`USkeletalMesh, USkeleton, UPhysicsAsset`) & their fetching apis

## 0.0.1 ##
* Add CHANGELOG
* Update maintainers
