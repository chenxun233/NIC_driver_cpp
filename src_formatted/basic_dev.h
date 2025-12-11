#pragma once
#include "basic_interrupt_handler.h"

class BasicDev{

    public:
        virtual bool initialize()                                                           = 0;
        virtual bool reset()                                                                = 0;
        virtual bool mapBar()                                                               = 0;
        virtual bool enableDMA()                                                            = 0;
        virtual bool setInterruptHandler(BasicInterruptHandler &interrupt_handler)          = 0;
        virtual bool getMacAddress()                                                        = 0;
        virtual bool setMacAddress()                                                        = 0;
        virtual bool getLinkSpeed()                                                         = 0;
        virtual bool setPromiscuousMode()                                                   = 0;
        virtual bool readStats()                                                            = 0;
        virtual bool prepQueue()                                                            = 0;
    private:
    
};