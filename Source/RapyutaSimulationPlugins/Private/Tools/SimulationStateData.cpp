// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

#include "Tools/SimulationStateData.h"
// UE
#include "Engine/World.h"
#include "EngineUtils.h"
#include "Kismet/GameplayStatics.h"
#include "Net/UnrealNetwork.h"


ASimulationStateData::ASimulationStateData()
{
    bReplicates = true;
    bNetLoadOnClient = true;
    bAlwaysRelevant=true;
}

void ASimulationStateData::GetLifetimeReplicatedProps( TArray< FLifetimeProperty > & OutLifetimeProps ) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);
    DOREPLIFETIME( ASimulationStateData, EntityList );
//    DOREPLIFETIME( ASimulationStateData, SpawnedEntityList );
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

//void ASimulationStateData::OnRep_Entity() {
//
//}
//Work around to replicating Entities and EntitiesWithTag since TMaps cannot be replicated
void ASimulationStateData::OnRep_Entity()
{
    for(AActor *Entity: EntityList) {
        if(Entity) {
            if (!Entities.Contains(Entity->GetName())) {
                Entities.Emplace(Entity->GetName(), Entity);
            }

            for (auto &tag: Entity->Tags) {
                AddTaggedEntities(Entity, tag);
            }

            UROS2Spawnable* EntitySpawnParam = Entity->FindComponentByClass<UROS2Spawnable>();
            if (EntitySpawnParam) {
                Entity->Rename(*EntitySpawnParam->GetName());
                for (auto &tag: EntitySpawnParam->ActorTags) {
                    AddTaggedEntities(Entity, FName(tag));
                }
            }
        }
    }


}
void ASimulationStateData::AddTaggedEntities(AActor* Entity, const FName& InTag){

    if (EntitiesWithTag.Contains(InTag)) {
        // Check if Actor in EntitiesWithTag

        bool ToEmplace = true;
        for (AActor *TaggedActor: EntitiesWithTag[InTag].Actors) {
            if (TaggedActor->GetName() == Entity->GetName()) {
                ToEmplace = false;
            }
        }
        if (ToEmplace) {
            EntitiesWithTag[InTag].Actors.Emplace(Entity);
        }
    } else {
        FActors actors;
        actors.Actors.Emplace(Entity);
        EntitiesWithTag.Emplace(InTag, actors);
    }
}


void ASimulationStateData::AddSpawnableEntities(TMap<FString, TSubclassOf<AActor>> InSpawnableEntities)
{
    for (auto& Elem : InSpawnableEntities)
    {
        SpawnableEntities.Emplace(Elem.Key, Elem.Value);
    }
}
//void ASimulationStateData::AddSpawnedEntity(AActor* Entity)
//{
//    if (IsValid(Entity))
//    {
//        SpawnedEntityList.Emplace(Entity);
//    }
//}

