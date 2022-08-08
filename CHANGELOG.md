# Changelog for RapyutaSimulationPlugins repository
## 0.0.12 ##
* `URRRobotROS2Interface::CreatePublisher()` Add QoS param to init publisher's frequent msg publishing callback
* Add `ARRCrowdROSController`, inheriting from `ARRCrowdAIController`, to init possessed robot's ROS2Interface
* `ARRRobotROS2Interface` add virtual `SetupROSParams()`, overridable in child classes (eg with bPublishOdom off if not used)
## 0.0.11 ##
* `ASimulationState`: Add `Server~` for service callbacks called on server only. There is a single instance owned by GameMode, which is server-only.
* Add replicatble `URRROS2SimulationStateClient` as UActorComponent, moving setup of ROS2 servicesthere from `ASimulationState`. One is created for each client's network player controller.
* Add `ARRNetworkGameMode, ARRNetworkGameState, ARRNetworkPlayerController`
* Turn on `bReplicates` for `ARRBaseRobot, ASimulationState, ARRNetworkGameState, ARRNetworkPlayerController`

## 0.0.10 ##
* `ARRBaseRobot` add ROS2Interface member & CreateRobotROS2Interface(), only instantiating it upon valid ROS2InterfaceClass
* `ARRRobotVehicleROSController`: ARobotVehicle -> ARRRobotBaseVehicle
* Add `ARRCrowAIController` since `ADetourCrowdAIController` lacks module API tag, thus is not compilable in EDITOR build
* Add Scripts/verify_ue4_env.sh, so users could use a common custom env var (as path to UE4Editor) for all running scripts in client project repos
* `URRRobotROS2Interface` Add `OnMessageReceived()`, being usable as a generic class callback

## 0.0.9 ##
* `SimulationState` Add `SpawnEntity()`, used by `SpawnEntitySrv + SpawnEntitiesSrv`

## 0.0.8 ##
* Add ARRBaseRobot, ARRRobotROS2Interface taking src from ARobotROSController
* Robot vehicle Collision fix2 (#69)
* URRGameSingleton Add meshName <-> shapeType conversion (#68)
* Prefix RR->RobotBaseVehicle 
* Rem ATurtleborROSController, use URRTurtlebotROS2Interface + Rem BP_TurlebotROSController, use BP_TurtlebotROS2Interface instead

## 0.0.7 ##
* URRGameSingleton Add meshName <-> shapeType conversion
* Add Content/DynamicContents/Materials/M_RapyutaPropMaster
* FRRMeshData add MeshSize for primitive type

## 0.0.6 ##
* Add data synth utils

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
