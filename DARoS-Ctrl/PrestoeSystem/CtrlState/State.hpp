#ifndef __CTRL_STATE_H__
#define __CTRL_STATE_H__

template <typename T>
class State {
public:
    State();
    virtual ~State();

    virtual void Initialize() {}
    virtual void RunEnter() = 0;
    virtual void RunNominal() = 0;
    virtual void RunTransition() {}

    bool CheckInitialization() { return b_initialized; }
protected:
    bool b_initialized;
};

#endif