#include "../zy12pdn-oss/include/pd_sink.h"
#include "../zy12pdn-oss/include/i2c_bit_bang.h"

using namespace usb_pd;

static pd_sink power_sink;
mcu_hal usb_pd::hal;
static i2c_bit_bang i2c;

//
void sink_callback(callback_event event);

// Called when the USB PD controller triggers an event
void sink_callback(callback_event event)
{
    int voltage = 5000;

    switch (event)
    {
    case callback_event::source_caps_changed:

        // Take maximum voltage
        for (int i = 0; i < power_sink.num_source_caps; i++)
        {
            if (power_sink.source_caps[i].voltage > voltage)
            {
                voltage = power_sink.source_caps[i].voltage;
            }
        }

        // Limit voltage to 20V as the voltage regulator was likely selected to handle 20V max
        if (voltage > 20000)
        {
            voltage = 20000;
        }

        power_sink.request_power(voltage);
        break;

    case callback_event::power_ready:
        break;

    case callback_event::protocol_changed:
        break;

    default:
        break;
    }
}
