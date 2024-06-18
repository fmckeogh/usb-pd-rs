# usb-pd-rs

[![CI](https://github.com/fmckeogh/usb-pd-rs/actions/workflows/build.yml/badge.svg)](https://github.com/fmckeogh/usb-pd-rs/actions/workflows/build.yml)
[![fusb302b docs](https://img.shields.io/badge/docs-fusb302b-blue)](https://fmckeogh.github.io/usb-pd-rs/fusb302b/)
[![usb_pd docs](https://img.shields.io/badge/docs-usb_pd-blue)](https://fmckeogh.github.io/usb-pd-rs/usb_pd/)

Rust USB-PD library, driver for the FUSB302B, and firmware for the ZY12PDN USB-PD trigger board (ported from [manuelbl/zy12pdn-oss](https://github.com/manuelbl/zy12pdn-oss)).

## Structure

The `usb-pd` library implements the USB-PD logic. A `Source` or `Sink` have corresponding `Driver` traits which are implemented by USB hardware drivers, such as `fusb302b`.

## Usage

```
$ DEFMT_LOG=trace DEFMT_RTT_BUFFER_SIZE=128 cargo r -r
```

MCU has limited flash so it will not build without the release profile and it's recommend to reduce the RTT buffer size.
