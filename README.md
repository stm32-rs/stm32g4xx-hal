# `stm32g4xx-hal`

🚧 *Work in progress*

_stm32g4xx-hal_ contains a multi device hardware abstraction on top of the
peripheral access API for the STMicro STM32G4 series microcontrollers. The
selection of the MCU is done by feature gates, typically specified by board
support crates.

## Usage

This crate will eventually contain support for multiple microcontrollers in the
stm32g4 family. Which specific microcontroller you want to build for has to be
specified with a feature, for example `stm32g431`.

Currently supported configurations are:

* stm32g431
* stm32g441
* stm32g491
* stm32g473
* stm32g483
* stm32g474
* stm32g484

### Building an Example

If you are compiling the crate on its own for development or running examples, 
specify your microcontroller on the command line. For example:

```
cargo build --example blinky --features stm32g473
```

## Running examples

Examples can be built and run using `cargo run`. It is necessary to provide any
required features followed by the name of the chip. 

```
cargo run --example blinky_delay --features stm32g474,log-rtt,defmt --release -- --chip STM32G474RBTx
```

A list of chips supported by probe-rs can be found by running

```
probe-rs chip list
```

For furher information, see the documentation for [probe-rs](https://github.com/probe-rs/probe-rs).

### Using as a Dependency

When using this crate as a dependency in your project, the microcontroller can 
be specified as part of the `Cargo.toml` definition.

```
[dependencies]
stm32g4xx-hal = "0.0.0"
```

## Documentation

The documentation can be found at [docs.rs](https://docs.rs/stm32g4xx-hal/).

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
  http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall be
dual licensed as above, without any additional terms or conditions.