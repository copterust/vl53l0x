# `vl53l0x`

> no_std driver for the vl53l0x [vl53l0x](https://www.st.com/resource/en/datasheet/vl53l0x.pdf) (Time of Flight IR distance sensor).

![Build Status](https://github.com/copterust/vl53l0x/actions/workflows/ci.yaml/badge.svg)

## What works

- To be tested

## Supported chips

* `VL53L0X`;


## Basic usage

Include [library](https://crates.io/crates/vl53l0x) as a dependency in your Cargo.toml
[![crates.io](http://meritbadge.herokuapp.com/vl53l0x?style=flat-square)](https://crates.io/crates/vl53l0x):

```
[dependencies.vl53l0x]
version = "<version>"
```

Use embedded-hal implementation to get I2C handle and delay then create vl53l0x handle:

```rust
use vl53l0x;

// to create sensor with i2c:

let mut tof = vl53l0x::VL53L0x::new(i2c).expect("tof");
tof.set_measurement_timing_budget(200000).expect("time budget");
tof.start_continuous(0).expect("start");
let mls = tof.read_range_continuous_millimeters();
println!("{}", mls);
```

## More examples
### STM32
Number of examples can be found in [proving-ground](https://github.com/copterust/proving-ground) repo.

### ESP32-C3
https://github.com/ChocolateLoverRaj/rust-esp32c3-examples/tree/main/VL53L0X

## Documentation

API Docs available on [docs.rs](https://docs.rs/vl53l0x).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
