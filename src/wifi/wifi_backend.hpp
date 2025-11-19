#pragma once



class WifiBackend {
public:
    WifiBackend() = default;

    virtual ~WifiBackend() = default;

    virtual void Update() = 0;
private:



};