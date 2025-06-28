#pragma once

template <typename TFlags, TFlags Flag, typename TContext, void (TContext::*TMethod)()>
struct DispatchEntry {
    static constexpr TFlags flag = Flag;
    static constexpr auto method = TMethod;
};

template <typename TFlags, typename TContext, typename... Entries>
class Dispatcher {
public:
    Dispatcher(TContext* context) : m_context(context) {}

    void Dispatch(TFlags flags) {
        (DispatchSingle<Entries>(flags), ...);
    }

private:
    template <typename Entry>
    void DispatchSingle(TFlags flags) {
        if (flags & Entry::flag) {
            (m_context->*(Entry::method))();
        }
    }

    TContext* m_context;
};

