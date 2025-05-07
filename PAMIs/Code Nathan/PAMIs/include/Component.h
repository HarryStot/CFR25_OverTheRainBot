#ifndef COMPONENT_H
#define COMPONENT_H

#include <Arduino.h>

class Component {
protected:
    String name;

public:
    Component(String name) : name(name) {}
    virtual ~Component() {}

    String getName() const { return name; }

    virtual void update() = 0;
};

#endif // COMPONENT_H