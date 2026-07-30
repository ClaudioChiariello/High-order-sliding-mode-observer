#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/ContactSensorData.hh>
#include <gz/plugin/Register.hh>
#include <iostream>


class ContactPrinter :
  public gz::sim::System,
  public gz::sim::ISystemPostUpdate
{
public:


    ContactPrinter()
    {
        std::cout << "ContactPrinter loaded!" << std::endl;
    }


    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) override
    {
        int count = 0;

        _ecm.Each <gz::sim::components::ContactSensorData> (
            [&](const gz::sim::Entity &_entity,
                const gz::sim::components::ContactSensorData *_data)
            {
                count++;

                std::cout << "Contact sensor detected" << std::endl;

                return true;
            });

    }

};


GZ_ADD_PLUGIN(
  ContactPrinter,
  gz::sim::System,
  gz::sim::ISystemPostUpdate
)