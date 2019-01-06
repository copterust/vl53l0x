# `vl53l0x`

> no_std driver for the [vl53l0x](https://www.st.com/resource/en/datasheet/vl53l0x.pdf) (Time of Flight IR distance sensor).

[![Build Status](https://travis-ci.org/copterust/vl53l0x.svg?branch=master)](https://travis-ci.org/copterust/vl53l0x)

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
extern crate vl53l0x; // or just use vl53l0x; if 2018 edition is used.

// to create sensor with default configuration:
let mut lsm = VL53L0X::default(l2c, &mut delay)?;
// to get all supported measurements:
let all = marg.all()?;
println!("{:?}", all);
```

## More examples

Number of examples can be found in [proving-ground](https://github.com/copterust/proving-ground) repo.

## Documentation

API Docs available on [docs.rs](https://docs.rs/vl53l0x).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
