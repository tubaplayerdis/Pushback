//
// Created by aaron on 7/20/2025.
//

#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

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
    //Initialization needs to toggled to INITIALIZED in derived class constructor if bNeedsInit is true. Is activated by default.
    explicit subsystem(bool bNeedsInit = false, bool bStartActive = true);

    public:
    virtual ~subsystem() = default;

    [[nodiscard]] bool IsActive();
    [[nodiscard]] EInitializationState GetInitializationState();

    protected:
    //Implement this function in derived class. Called inside Activate(). Default returns true
    virtual bool Activate_Implementation();
    //Implement this function in derived class. Called inside Deactivate(). Default returns true
    virtual bool Deactivate_Implementation();
    //Called in subsequent loop. Has to be implemented
    virtual void Tick_Implementation() = 0;

    public:
    void Tick();
    bool Activate();
    bool Deactivate();
};

#endif //SUBSYSTEM_H
