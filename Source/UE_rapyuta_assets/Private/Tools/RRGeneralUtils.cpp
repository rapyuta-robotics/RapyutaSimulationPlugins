// Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

bool URRGeneralUtils::CheckWithTimeOut(const TFunctionRef<bool()>& Condition,
                                       const TFunctionRef<void()>& Action,
                                       const FDateTime& BeginTime,
                                       float TimeoutInSec)
{
    if (Condition())
    {
        return true;
    }

    double elapsed_time = FTimespan(FDateTime::UtcNow() - BeginTime).GetTotalSeconds();
    if (elapsed_time > TimeoutInSec)
    {
        Action();
    }
    return false;
}
