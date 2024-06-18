#include "../zy12pdn-oss/include/pd_sink.h"
#include "../zy12pdn-oss/include/i2c_bit_bang.h"

using namespace usb_pd;

static pd_sink power_sink;
mcu_hal usb_pd::hal;
static i2c_bit_bang i2c;

//
void sink_callback(callback_event event);

// int main() {
//     hal.init();

//     DEBUG_LOG("Saved mode: %d\r\n", desired_mode);

//     power_sink.set_event_callback(sink_callback);
//     power_sink.init();

//     // Work in regular loop
//     while (true) {
//         power_sink.poll();
//     }
// }

// Called when the source advertises new capabilities
// Be careful with debug output. If one of the capbilities is not
// requested in time, the power suplly will reset.
void on_source_caps_changed()
{
    int voltage = 5000;

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
        voltage = 20000;

    power_sink.request_power(voltage);
}

// Called when the USB PD controller triggers an event
void sink_callback(callback_event event)
{

    switch (event)
    {
    case callback_event::source_caps_changed:

        on_source_caps_changed();
        break;

    case callback_event::power_ready:

        break;

    case callback_event::protocol_changed:

        break;

    default:
        break;
    }
}
