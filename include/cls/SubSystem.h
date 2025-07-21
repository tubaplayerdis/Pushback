//
// Created by aaron on 7/20/2025.
//

#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

//Used in motor.move()
#define FULL_POWER 127

enum EInitializationState
{
    NONAPPLICABLE = 0,
    INITIALIZED = 1,
    UNINITIALIZED = 2,
};

class SubSystem
{
    bool isActive;

    protected:
    EInitializationState Initialization;
    //Initialization needs to toggled to INITIALIZED in derived class constructor if bNeedsInit is true
    SubSystem(bool bNeedsInit, bool bStartActive);

    public:
    virtual ~SubSystem() = default;

    bool IsActive() const;
    EInitializationState GetInitializationState() const;

    protected:
    //Implement this function in derived class. Called inside Activate().
    virtual bool Activate_Implementation() = 0;
    //Implement this function in derived class. Called inside Deactivate().
    virtual bool Deactivate_Implementation() = 0;
    //Called in subsequent loop.
    virtual void Tick() = 0;

    public:
    bool Activate();
    bool Deactivate();
};

inline SubSystem::SubSystem(bool bNeedsInit, bool bStartActive)
{
    bNeedsInit ? Initialization = EInitializationState::UNINITIALIZED : Initialization = EInitializationState::NONAPPLICABLE;
    isActive = bStartActive;
}

inline bool SubSystem::IsActive() const
{
    return isActive;
}

inline EInitializationState SubSystem::GetInitializationState() const
{
    return Initialization;
}

inline bool SubSystem::Activate()
{
    bool bResult = Activate_Implementation();
    isActive = true;
    return bResult;
}

inline bool SubSystem::Deactivate()
{
    bool bResult = Deactivate_Implementation();
    isActive = false;
    return bResult;
}

#endif //SUBSYSTEM_H
