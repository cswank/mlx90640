const std = @import("std");
const microzig = @import("microzig");
const drivers = microzig.drivers;
const rp2040 = microzig.hal;
const gpio = rp2040.gpio;
const i2c = rp2040.i2c;
const I2C_Device = rp2040.drivers.I2C_Device;
const time = rp2040.time;
const MLX90640 = drivers.sensor.MLX90640;

const uart = rp2040.uart.instance.num(0);
const baud_rate = 115200;
const uart_tx_pin = gpio.num(0);

const I2C_READ_LEN = 2048;
const SCALEALPHA = 0.000001;
const MLX90640_DEVICEID1 = 0x2407;
const MLX90640_DEVICEID2 = 0x2408;
const MLX90640_DEVICEID3 = 0x2409;
const MLX90640_REGISTER_1 = 0x800D;
const OPENAIR_TA_SHIFT = 8;

const addr = i2c.Address.new(0x33);

var frame_data: [834 * 2]u8 = undefined;
var frame: [834]u16 = undefined;

const pin_config = rp2040.pins.GlobalConfiguration{
    .GPIO0 = .{ .name = "gpio0", .function = .UART0_TX },
};

pub const microzig_options = microzig.Options{
    .log_level = .debug,
    .logFn = rp2040.uart.logFn,
};

const i2c0 = i2c.instance.num(0);
const i2c_device = I2C_Device.init(i2c0, @enumFromInt(0x33));

pub fn main() !void {
    uart_tx_pin.set_function(.uart);
    uart.apply(.{
        .baud_rate = baud_rate,
        .clock_config = rp2040.clock_config,
    });

    try i2c0.apply(i2c.Config{ .clock_config = rp2040.clock_config });

    rp2040.uart.init_logger(uart);
    pin_config.apply();

    std.log.info("Hello from mlx90640", .{});

    const scl_pin = gpio.num(5);
    const sda_pin = gpio.num(4);
    inline for (&.{ scl_pin, sda_pin }) |pin| {
        pin.set_slew_rate(.slow);
        pin.set_schmitt_trigger(.enabled);
        pin.set_function(.i2c);
    }

    const sn = try serial_number();
    std.log.info("camera serial number: {x}", .{sn});

    try set_refresh_rate(0);
    const rate = try refresh_rate();
    std.log.info("camera refresh rate: {b}", .{rate});

    const rez = try resolution();
    std.log.info("camera resolution: {b}", .{rez});

    while (true) {
        //try pixels();
        time.sleep_ms(1_000);
    }
}

fn image() ![834]u8 {}

fn set_refresh_rate(rate: u16) !void {
    var data: [2]u8 = undefined;

    var req = [4]u8{
        MLX90640_REGISTER_1 >> 8,
        MLX90640_REGISTER_1 & 0xFF,
        0,
        0,
    };

    try i2c0.write_then_read_blocking(addr, req[0..2], data[0..2], null);

    const val = std.mem.readInt(u16, data[0..2], .big);
    const mask: u16 = 0b111 << 7;
    const new_val: u16 = (val & ~mask) | ((rate << 7) & mask);

    req[2] = @as(u8, @truncate(new_val >> 8));
    req[3] = @as(u8, @truncate(new_val & 0xFF));

    try i2c0.write_blocking(addr, &req, null);
}

fn refresh_rate() !u3 {
    var data: [1]u16 = undefined;
    try write_then_read(MLX90640_REGISTER_1, data[0..1]);

    const val = data[0] >> 7 & 0b111;
    return @as(u3, @truncate(val));
}

fn resolution() !u2 {
    var data: [1]u16 = undefined;

    try write_then_read(MLX90640_REGISTER_1, data[0..1]);

    const val = data[0] >> 10 & 0b11;
    return @as(u2, @truncate(val));
}

fn serial_number() !u48 {
    var data: [3]u16 = undefined;

    try write_then_read(MLX90640_DEVICEID1, data[0..1]);
    try write_then_read(MLX90640_DEVICEID2, data[1..2]);
    try write_then_read(MLX90640_DEVICEID3, data[2..3]);

    return @as(u48, data[0]) << 32 |
        @as(u48, data[1]) << 16 |
        @as(u48, data[2]);
}

fn pixels() !void {
    var ready = try isReady();
    while (!ready) {
        ready = try isReady();
        time.sleep_ms(10);
    }

    try write_then_read(0x0400, frame[0..832]);
    std.log.info("frame {x}", .{frame});
}

fn isReady() !bool {
    var buf: [1]u16 = undefined;
    try write_then_read(0x800, &buf);
    return (buf[0] >> 4 | 0b1) > 0;
}

fn write_then_read(address: u16, buf: []u16) !void {
    const req = [2]u8{ @as(u8, @truncate(address >> 8)), @as(u8, @truncate(address & 0xFF)) };
    try i2c0.write_then_read_blocking(addr, &req, frame_data[0 .. buf.len * 2], null);
    for (0.., buf) |i, _| {
        buf[i] = std.mem.readInt(u16, frame_data[i * 2 .. (i * 2) + 2][0..2], .big);
    }
}
