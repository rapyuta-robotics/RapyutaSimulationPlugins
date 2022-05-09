// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/SimulationStateData.h"
// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "Net/UnrealNetwork.h"


void ASimulationStateData::GetLifetimeReplicatedProps( TArray< FLifetimeProperty > & OutLifetimeProps ) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME( ASimulationStateData, EntityList );
    DOREPLIFETIME( ASimulationStateData, SpawnedEntityList );
}

void ASimulationStateData::AddEntity(AActor* Entity)
{
    if (IsValid(Entity))
    {
        Entities.Emplace(Entity->GetName(), Entity);
        EntityList.Emplace(Entity);
        for (auto& tag : Entity->Tags)
        {
            if (EntitiesWithTag.Contains(tag))
            {
                EntitiesWithTag[tag].Actors.Emplace(Entity);
            }
            else
            {
                FActors actors;
                actors.Actors.Emplace(Entity);
                EntitiesWithTag.Emplace(tag, actors);
            }
        }
    }
}

void ASimulationStateData::AddSpawnedEntity(AActor* Entity)
{
    if (IsValid(Entity))
    {
        SpawnedEntityList.Emplace(Entity);
    }
}
//void ASimulationStateData::AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities)
//{
//    for (auto& Elem :InSpawnableEntities)
//    {
//        SpawnableEntities.Emplace(Elem.Key, Elem.Value);
//    }
//}

TMap<FString, AActor*> ASimulationStateData::GetEntity()
{
    return Entities;
}

//TMap<FString, TSubclassOf<AActor>> ASimulationStateData::GetSpawnableEntities()
//{
//    return SpawnableEntities;
//}
//bool ASimulationStateData::RequestInSpawnableEntities(FString RequestName)
//{
//
//    if(SpawnableEntities.Contains(RequestName)) {
//        return true;
//    }
//    else{
//        return false;
//    }
//
//}
//
//bool IsValidRequestSpawnableEntities(FString RequestName)
//{
//    if(IsValid(SpawnableEntities[RequestName])) {
//        return true;
//    }
//    else{
//        return false;
//    }
//}
//
//TSubclassOf<AActor> GetSpawnableEntity(FString RequestName)
//{
//    return SpawnableEntities[RequestName];
//}