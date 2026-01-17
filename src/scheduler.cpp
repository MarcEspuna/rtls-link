#include "scheduler.hpp"
#include "logging/logging.hpp"

Scheduler Scheduler::scheduler;

Scheduler::Scheduler()
{

}

void Scheduler::Init()
{
    LOG_INFO("Scheduler initializing");
}