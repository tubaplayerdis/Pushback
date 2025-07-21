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

class subsystem
{
    bool isActive;

    protected:
    EInitializationState Initialization;
    //Initialization needs to toggled to INITIALIZED in derived class constructor if bNeedsInit is true
    subsystem(bool bNeedsInit, bool bStartActive);

    public:
    virtual ~subsystem() = default;

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

inline subsystem::subsystem(bool bNeedsInit, bool bStartActive)
{
    bNeedsInit ? Initialization = EInitializationState::UNINITIALIZED : Initialization = EInitializationState::NONAPPLICABLE;
    isActive = bStartActive;
}

inline bool subsystem::IsActive() const
{
    return isActive;
}

inline EInitializationState subsystem::GetInitializationState() const
{
    return Initialization;
}

inline bool subsystem::Activate()
{
    bool bResult = Activate_Implementation();
    isActive = true;
    return bResult;
}

inline bool subsystem::Deactivate()
{
    bool bResult = Deactivate_Implementation();
    isActive = false;
    return bResult;
}

#endif //SUBSYSTEM_H
