# Pushback - 38535A

#### Private code repository for vex team 38535A's program.

## Contribution

To contribute, please contact Aaron for write privileges on the repository and then read the following guidelines along with the design philosophy section.

#### Contributing Guidelines

 - Update your local repository frequently as to allow new changes to come in.
 - Pull and rebase changes before every session.
 - Commit and push at the end of every session.
 - Include descriptive commit messages.
 - Resolve merge conflicts collaboratively.

## Design Philosophy

### Simplicity

As Terry Davis said: “An idiot admires complexity, a genius admires simplicity, a physicist tries to make it simple, for an idiot anything the more complicated it is the more he will admire it, if you make something so clusterfucked he can't understand it he's gonna think you're a god cause you made it so complicated nobody can understand it. That's how they write journals in Academics, they try to make it so complicated people think you're a genius”

One of the primary goals behind the design choices for this program was simplicity and modularity. Everything done was done in the most approvingly simplistic fashion for the means of contribution, documentation and expansion. For this reason I have limited this program to the following programming principles:

 - Inheritance (subclasses)
 - Polymorphism (virtual functions)
 - Singletons (accessors)

### Class Structure

All classes pertaining to interaction with physical devices on the robot are called subsystems.
A subsystem is organizational in that they each interact with different mechanical systems on the robot.

All classes inherit the base class **subsystem** (inheritance). subsystem is used as a base class to allow for the standardization of 3 critical functions of which only tick **must** to be implemented in a subclass. (polymorphism)

 - tick() - "ticks" the subsystem, IE executes whatever code should be ran in the main loop of the program, examples include gathering input to move motors or pneumatics.
 - activate() - activates the subsystem, essentially a way to enable things like a color sensor or setup other electronics after the class has been created.
 - deactivate() - deactivates the subsystem, useful for things like deleting heap variables and other special de-activation code.

All of these function include respective **_implementation** functions, which are virtual functions (functions that get implemented by subclasses for the purpose of subclass specific behavior)

### Singletons

All classes that are subsystems use what are called singletons. A singleton is a (singular) instance of class that is accessed and via pointers.
The rationale behind this decision:

 - Why not static or namespaces?: Originally, I was going to use namespaces and statics but then comes the need for added complexity with individual initialization/enabling functions and usage of singletons allowed me to optimize that with smart pointers (which automatically destroy the object) and a **single function to access and create the object, get().**
 - Mental mapping: It doesn't make sense to have multiple "drivetrain" objects floating around. Which one does which? will the other objects throw errors? Using a singleton object over multiple ones for each time it is needed saves the headache of complex errors and de-bugging which is caused by multiple definitions of objects that represent devices. Singletons also alleviate complexity through a good **virtual representation of the physical robot.**
 - Limited stack memory: pros and lvgl use alot of stack allocated memory; because of this, using singletons that are heap allocated saves the stack memory for more important tasks and allows us to take advantage of the **75mb of heap allocated memory** present on the Vex V5 brain.

Combined with singletons, are smart pointers and private constructors, which enforce the usage of the singletons and simplify control flow and readability.

### Storage Namespaces

Things like the controller namespace and ports use a storage namespace due to that they are not a subsystem that needs to be ticked. They only provide access to things like the controller and as such dont need outstanding complexity to achieve a concise way of accessing those variables.

### TitanSelect

titanselect is an auton selector I (Aaron Wilk) created due to that out previous auton selector (Robodash) went OOS (Out Of Support) and the developer abandoned the project. Normally this would not be of issue except for the fact that RoboDash uses lvgl 8 and pros now uses lvgl 9, which changed all the binding names. This mismatch led to me creating TitanSelect as an alternative to RoboDash and is now the standard for Team 38535A. Documentation for Titan-Select is available at its repo: [titanselect](https://github.com/tubaplayerdis/titanselect)

### Naming Schemes (naming_schemes)

 - lowercase and uses _ for spaces
 - follows naming standards set by c++ committee.