This project is a native Rust implementation of a flash utility based on the WinnerMicro IoT Software Development Kit. See: https://github.com/winnermicro/wm_iot_sdk/tree/master

## How to build
Install Rust tools and run
```
cargo build --release
```

## How to use

Identify a port which is used for your devive connection:
```bash
./target/relase/flash -l
```

Run the flasher with a maximum or specific baudrate
```bash
./target/release/flash -p /dev/tty.usbserial-1110 -i dso3d12_v3.0.6_III.fls
```

## Command line arguments
```
Flash tool for Winner Micro MCU

USAGE:
    flash [OPTIONS]

OPTIONS:
    -b, --baudrate <BAUDRATE>    serial baudrate
    -e, --erase                  erase device flash
    -h, --help                   Print help information
    -i, --image <IMAGE>          image file paths
    -l, --list                   list serial ports
    -m, --manual-reset           manual reset after burning
    -n, --name <NAME>            firmware names to burn. Ex:
                                 app,bootloader,partition_table,custom...
    -p, --port <PORT>            serial port
    -V, --version                Print version information
```
