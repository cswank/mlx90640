const std = @import("std");
const microzig = @import("microzig");
const drivers = microzig.drivers;
const rp2040 = microzig.hal;
const gpio = rp2040.gpio;
const i2c = rp2040.i2c;
const I2C_Device = rp2040.drivers.I2C_Device;
const time = rp2040.time;

const uart = rp2040.uart.instance.num(0);
const baud_rate = 115200;
const uart_tx_pin = gpio.num(0);

const SCALEALPHA = 0.000001;
const OPENAIR_TA_SHIFT = 8;

const REGISTER_1 = 0x800D;

const addr = i2c.Address.new(0x33);

var frame_data: [834 * 2]u8 = undefined;
var frame: [834]u16 = undefined;

const refresh_rate_mask: u16 = 0b111 << 7;

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
    try init();

    const sn = try serial_number();
    std.log.info("camera serial number: {x}", .{sn});

    try set_refresh_rate(1);
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
    try write_then_read(REGISTER_1, frame[0..1]);
    const new_val: u16 = (frame[0] & ~refresh_rate_mask) | ((rate << 7) & refresh_rate_mask);
    try write(REGISTER_1, new_val);
}

fn refresh_rate() !u3 {
    try write_then_read(REGISTER_1, frame[0..1]);
    const val = frame[0] >> 7 & 0b111;
    return @as(u3, @truncate(val));
}

fn resolution() !u2 {
    try write_then_read(REGISTER_1, frame[0..1]);
    const val = frame[0] >> 10 & 0b11;
    return @as(u2, @truncate(val));
}

fn serial_number() !u48 {
    try write_then_read(0x2407, frame[0..1]);
    try write_then_read(0x2408, frame[1..2]);
    try write_then_read(0x2409, frame[2..3]);

    return @as(u48, frame[0]) << 32 |
        @as(u48, frame[1]) << 16 |
        @as(u48, frame[2]);
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
    try write_then_read(0x800, frame[0..1]);
    return (frame[0] >> 4 | 0b1) > 0;
}

fn write_then_read(address: u16, buf: []u16) !void {
    const req = [2]u8{ @as(u8, @truncate(address >> 8)), @as(u8, @truncate(address & 0xFF)) };
    try i2c0.write_then_read_blocking(addr, &req, frame_data[0 .. buf.len * 2], null);
    for (0.., buf) |i, _| {
        buf[i] = std.mem.readInt(u16, frame_data[i * 2 .. (i * 2) + 2][0..2], .big);
    }
}

fn write(address: u16, val: u48) !void {
    const req = [4]u8{
        @as(u8, @truncate(address >> 8)),
        @as(u8, @truncate(address & 0xFF)),
        @as(u8, @truncate(val >> 8)),
        @as(u8, @truncate(val & 0xFF)),
    };

    try i2c0.write_blocking(addr, &req, null);
}

fn init() !void {
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
}
