#include "front.hpp"
#include "logging/logging.hpp"

etl::vector<IFrontend*, Front::MAX_FRONTENDS>& Get();

void Front::AddFrontend(IFrontend* frontend)
{
    Get().push_back(frontend);
}

void Front::InitFrontends()
{
    LOG_INFO("Initializing frontends");
    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Get();

    // Initialize WiFi frontend first so UDP logging is available for other frontends
    for (size_t i = 0; i < frontends.size(); i++)
    {
        if (frontends[i]->GetParamGroup() == "wifi")
        {
            frontends[i]->Init();
        }
    }

    // Initialize remaining frontends
    for (size_t i = 0; i < frontends.size(); i++)
    {
        if (frontends[i]->GetParamGroup() != "wifi")
        {
            frontends[i]->Init();
        }
    }

    LOG_INFO("Frontends initialized");
}

ErrorParam Front::WriteGlobalParam(const char *group, const char *name, const void *data, uint32_t len)
{
    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Get();
    for (size_t i = 0; i < frontends.size(); i++)
    {
        IFrontend* frontend = frontends[i];
        if (frontend->GetParamGroup() == group)
        {
            return frontend->SetParam(name, data, len);
        }
    }
    return ErrorParam::GROUP_NOT_FOUND;
}

ErrorParam Front::ReadGlobalParam(const char *group, const char *name, char* value, uint32_t &len, ParamType &type)
{
    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Get();
    for (size_t i = 0; i < frontends.size(); i++)
    {
        IFrontend* frontend = frontends[i];
        if (frontend->GetParamGroup() == group)
        {
            return frontend->GetParam(name, value, len, type);
        }
    }
    return ErrorParam::GROUP_NOT_FOUND;
}

ErrorParam Front::LoadAllParams()
{
    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Get();
    ErrorParam result = ErrorParam::OK;
    
    for (size_t i = 0; i < frontends.size(); i++)
    {
        IFrontend* frontend = frontends[i];
        ErrorParam loadResult = frontend->LoadParams();
        if (loadResult != ErrorParam::OK && loadResult != ErrorParam::FILE_NOT_FOUND) {
            result = loadResult;
        }
    }
    return result;
}

ErrorParam Front::SaveAllParams()
{
    etl::vector<IFrontend*, Front::MAX_FRONTENDS>& frontends = Get();
    ErrorParam result = ErrorParam::OK;
    
    for (size_t i = 0; i < frontends.size(); i++)
    {
        IFrontend* frontend = frontends[i];
        ErrorParam saveResult = frontend->SaveParams();
        if (saveResult != ErrorParam::OK) {
            result = saveResult;
        }
    }
    return result;
}

etl::vector<IFrontend*, Front::MAX_FRONTENDS>& Front::Get() {
    static etl::vector<IFrontend*, Front::MAX_FRONTENDS> frontends;
    return frontends;
}
