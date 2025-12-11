#pragma once

class BasicInterruptHandler {
    public:
        virtual bool setup_interrupts()    = 0;
        virtual bool enable_interrupts()   = 0;
        virtual bool disable_interrupts()  = 0;
    private:
};