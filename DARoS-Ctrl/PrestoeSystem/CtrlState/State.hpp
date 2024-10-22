#ifndef __CTRL_STATE_H__
#define __CTRL_STATE_H__

#include <cppTypes.h>
#include <Configuration.h>
#include <PrestoeDefinition.h>

template <typename T>
class State {
public:
    State(){}
    virtual ~State(){}

    virtual void Initialize() {}
    virtual void OnEnter() = 0;
    virtual void RunNominal() = 0;
    virtual void RunTransition() {}

protected:
};

#endif