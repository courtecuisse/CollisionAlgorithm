#pragma once

#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

namespace sofa
{

namespace collisionAlgorithm
{

class ProximityPairHolder : public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(ProximityPairHolder, core::objectmodel::BaseObject);

    Data<DetectionOutput > d_input;
    Data<DetectionOutput > d_output;

    Data<bool> d_hold ;
    Data<std::string> d_toggleKey ;

    core::objectmodel::DataCallback c_callback;

    ProximityPairHolder()
    : d_input     (initData(&d_input, "input", "draw collision"))
    , d_output    (initData(&d_output, "output", "draw collision"))
    , d_hold      (initData(&d_hold, false,  "hold", "If true the component hold the last input before hold was at true" ))
    , d_toggleKey (initData(&d_toggleKey, std::string("tT"),  "key", "Key used to toggle it" ))
    {
        f_listening.setValue(true);
        c_callback.addInputs({&d_input});
        c_callback.addCallback(std::bind(&ProximityPairHolder::inputChanged,this));
    }

    void handleEvent(sofa::core::objectmodel::Event * event)
    {
        if (sofa::core::objectmodel::KeypressedEvent *ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent *>(event))
        {
            if (d_toggleKey.getValue().find(ev->getKey())!=std::string::npos)
            {
                d_hold.setValue(!d_hold.getValue());
            }
        }
    }

    void inputChanged()
    {
        if(!d_hold.getValue())
        {
            d_output.setValue(d_input.getValue());
        }
    }

};


}

}
