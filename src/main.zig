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

const CONTROL_REGISTER = 0x800D;
const STATUS_REGISTER = 0x8000;

const i2c_address = i2c.Address.new(0x33);

var frame_data: [834 * 2]u8 = undefined;
var eeprom: [832]u16 = undefined;
var frame: [834]u16 = undefined;
const frame_loop = [2]u1{ 0, 1 };
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
    std.log.info("camera serial number: 0x{x}", .{sn});

    try set_refresh_rate(0b011);
    const rate = try refresh_rate();
    std.log.info("camera refresh rate: 0b{b:0>3}", .{rate});

    const rez = try resolution();
    std.log.info("camera resolution: 0b{b:0>2}", .{rez});

    try write_then_read(CONTROL_REGISTER, frame[0..1]);
    std.log.info("control register: 0b{b:0>16}", .{frame[0]});

    try write_then_read(0x2400, &eeprom);

    var params: parameters = parameters{};
    extractParameters(&params);

    while (true) {
        try pixels();
        time.sleep_ms(100);
    }
}

fn extractParameters(params: *parameters) void {
    extract_vdd(params);
    extract_ptat(params);
    // extract_gain(params);
    // extract_tgc(params);
    // extract_resolution(params);
    // extract_ksta(params);
    // extract_kst0(params);
    // extract_cp(params);
    // extract_alpha(params);
    // extract_offset(params);
    // extract_ktapixel(params);
    // extract_kvpixel(params);
    // extract_cilc(params);
}

fn image() ![834]u8 {}

fn set_refresh_rate(rate: u16) !void {
    try write_then_read(CONTROL_REGISTER, frame[0..1]);
    const val: u16 = (frame[0] & ~refresh_rate_mask) | ((rate << 7) & refresh_rate_mask);
    try write(CONTROL_REGISTER, val);
}

fn refresh_rate() !u3 {
    try write_then_read(CONTROL_REGISTER, frame[0..1]);
    const val = frame[0] >> 7 & 0b111;
    return @as(u3, @truncate(val));
}

fn resolution() !u2 {
    try write_then_read(CONTROL_REGISTER, frame[0..1]);
    const val = frame[0] >> 10 & 0b11;
    return @as(u2, @truncate(val));
}

fn serial_number() !u48 {
    try write_then_read(0x2407, frame[0..3]);
    return @as(u48, frame[0]) << 32 |
        @as(u48, frame[1]) << 16 |
        @as(u48, frame[2]);
}

// fn getTa() !u16 {

// }

fn pixels() !void {
    var ready: bool = false;
    for (frame_loop) |i| {
        while (!ready) {
            try write_then_read(STATUS_REGISTER, frame[833..834]);
            ready = isReady(i, frame[833]);
            time.sleep_ms(10);
        }

        try write_then_read(0x0400, frame[0..832]);
    }

    try write_then_read(CONTROL_REGISTER, frame[832..833]);

    std.log.info("{x}", .{frame[400..410]});
}

fn isReady(i: u1, status: u16) bool {
    return @as(u1, @truncate(status & 0b1)) == i and @as(u1, @truncate((status >> 3) & 0b1)) == 1;
}

fn write_then_read(address: u16, buf: []u16) !void {
    const req = [2]u8{ @as(u8, @truncate(address >> 8)), @as(u8, @truncate(address & 0xFF)) };
    try i2c0.write_then_read_blocking(i2c_address, &req, frame_data[0 .. buf.len * 2], null);
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

    try i2c0.write_blocking(i2c_address, &req, null);
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

const parameters = struct {
    kVdd: i16 = 0,
    vdd25: i16 = 0,
    KvPTAT: f32 = 0,
    KtPTAT: f32 = 0,
    uvPTAT25: i16 = 0,
    alphaPTAT: f32 = 0,
    gainEE: i16 = 0,
    tgc: f32 = 0,
    cpKv: f32 = 0,
    cpKta: f32 = 0,
    resolutionEE: u8 = 0,
    calibrationModeEE: u8 = 0,
    KsTa: f32 = 0,
    ksTo: [5]f32 = undefined,
    ct: [5]i16 = undefined,
    alpha: [768]i16 = undefined,
    alphaScale: u8 = 0,
    offset: [768]i16 = undefined,
    kta: [768]i8 = undefined,
    ktaScale: u8 = 0,
    kv: [768]i8 = undefined,
    kvScale: u8 = 0,
    cpAlpha: [2]f32 = undefined,
    cpOffset: [2]i16 = undefined,
    ilChessC: [3]f32 = undefined,
    brokenPixels: [5]u16 = undefined,
    outlierPixels: [5]u16 = undefined,
};

fn extract_vdd(params: *parameters) void {
    params.kVdd = @intCast((eeprom[51] & 0xFF00) >> 8);
    if (params.kVdd > 127) {
        params.kVdd = (params.kVdd - 256) * 32;
    }
    params.vdd25 = @intCast((((eeprom[51] & 0x00FF) - 256) << 5) - 8192);
}

fn extract_ptat(params: *parameters) void {
    params.KvPTAT = @floatFromInt((eeprom[50] & 0xFC00) >> 10);
    if (params.KvPTAT > 31) {
        params.KvPTAT = params.KvPTAT - 64;
    }
    params.KvPTAT = params.KvPTAT / 4096;

    params.KtPTAT = @floatFromInt(eeprom[50] & 0x03FF);
    if (params.KtPTAT > 511) {
        params.KtPTAT = params.KtPTAT - 1024;
    }

    params.KtPTAT = params.KtPTAT / 8;

    params.uvPTAT25 = @intCast(eeprom[49]);

    const x: f32 = @floatFromInt(eeprom[16] & 0xF000);
    const y: f32 = std.math.pow(f32, 2, 14);
    params.alphaPTAT = (x / y) + 8.0;
}

// fn extract_gain(params: *parameters) void {}
// fn extract_tgc(params: *parameters) void {}
// fn extract_resolution(params: *parameters) void {}
// fn extract_ksta(params: *parameters) void {}
// fn extract_kst0(params: *parameters) void {}
// fn extract_cp(params: *parameters) void {}
// fn extract_alpha(params: *parameters) void {}
// fn extract_offset(params: *parameters) void {}
// fn extract_ktapixel(params: *parameters) void {}
// fn extract_kvpixel(params: *parameters) void {}
// fn extract_cilc(params: *parameters) void {}
