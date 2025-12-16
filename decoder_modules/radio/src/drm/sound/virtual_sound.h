#pragma once

#include "soundinterface.h"

/* Classes ********************************************************************/
class CVirtualSoundIn : public CSoundInInterface {
public:
    CVirtualSoundIn() {}
    virtual ~CVirtualSoundIn() {}
    virtual bool Init(int, int, bool) {
        return true;
    }
    virtual bool Read(CVector<short>&) {
        return false;
    }
    virtual void Enumerate(std::vector<std::string>& choices, std::vector<std::string>&) {
        choices.push_back("(File or Network)");
    }
    virtual std::string GetDev() {
        return "Virtual Input";
    }
    virtual void SetDev(std::string sNewDev) {
        sDev = sNewDev;
    }
    virtual void Close() {}
    virtual std::string GetVersion() {
        return "1.0";
    }

private:
    std::string sDev;
};

class CVirtualSoundOut : public CSoundOutInterface {
public:
    CVirtualSoundOut() {}
    virtual ~CVirtualSoundOut() {}
    virtual void Enumerate(std::vector<std::string>& names, std::vector<std::string>& descriptions) {
    }
    virtual void SetDev(std::string sNewDevice) {
    }
    virtual std::string GetDev() {
        return "Virtual Output";
    }
    virtual std::string GetVersion() {
        return "1.0";
    }
    virtual bool Init(int iSampleRate, int iNewBufferSize, bool bNewBlocking)
    {
        return true;
    }
    virtual void Close()
    {
    }
    virtual bool Write(CVector<short>& psData)
    {
        return true;
    }
};