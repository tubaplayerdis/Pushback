//
// Created by aaron on 7/20/2025.
//

#ifndef SUBSYSTEM_H
#define SUBSYSTEM_H

class subsystem
{
    /// Determines whether tick() executes tick_implementation();
    bool b_is_active;

    protected:

    /// Creates the subsystem.
    /// \param bStartActive Whether to allow the subsystem to work after creation of the class object. Defaults to true.
    explicit subsystem(bool bStartActive = true);

    public:

    /// Overrideable virtual constructor to allow for custom de-initialization code.
    virtual ~subsystem() = default;

    /// Whether the subsystem is active
    /// \return the b_is_active variable by value
    bool is_active();

    protected:

    /// Overrideable function to allow for a custom activation function is necessary.
    /// \return Whether activation failed or succeeded. Defaults to True
    virtual bool activate_implementation();

    /// Overrideable function to allow for a custom de-activation function is necessary.
    /// \return Whether de-activation failed or succeeded. Defaults to True
    virtual bool deactivate_implementation();

    /// Pure virtual function (mandatory implementation) that is called inside of tick() depending on if the subsystem is active. if b_is_active is false, this function will not be called and vice-versa
    virtual void tick_implementation() = 0;

    public:

    /// tick function that should be used inside of loops. Calls tick_implementation() depending of whether b_is_active is true
    void tick();

    /// activation function that calls activate_implementation. Sets b_is_active = true.
    bool activate();

    /// de-activation function that calls deactivate_implementation. Sets b_is_active = false.
    bool deactivate();
};

#endif //SUBSYSTEM_H
