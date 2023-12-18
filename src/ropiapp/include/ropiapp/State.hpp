#pragma once

#include <memory>

#include "Context.hpp"

class State
{   
    public:
        /**
         * @brief Construct a new State object
         * 
         * @param contextPtr 
         */
        State(Context&  contextPtr) 
                    : contextPtr_(contextPtr){}
        virtual void Execute() {
            
        }

    protected:
        Context& contextPtr_;

};


