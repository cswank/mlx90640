const std = @import("std");
const microzig = @import("microzig");
const lib = @import("mlx90640.zig");
const drivers = microzig.drivers;
const rp2040 = microzig.hal;
const gpio = rp2040.gpio;
const i2c = rp2040.i2c;
const I2C_Device = rp2040.drivers.I2C_Device;
const time = rp2040.time;

const uart = rp2040.uart.instance.num(0);
const baud_rate = 115200;
const uart_tx_pin = gpio.num(0);

var i2c0 = i2c.instance.num(0);
const i2c_device = I2C_Device.init(i2c0, @enumFromInt(0x33));

const pin_config = rp2040.pins.GlobalConfiguration{
    .GPIO0 = .{ .name = "gpio0", .function = .UART0_TX },
};

pub const microzig_options = microzig.Options{
    .log_level = .debug,
    .logFn = rp2040.uart.logFn,
};

pub fn PicoI2C() type {
    return struct {
        i2c: *rp2040.i2c.I2C,
        address: i2c.Address,
        interface: lib.I2C(),
        frame_data: [834 * 2]u8,
        const Self = @This();

        pub fn init(i: *rp2040.i2c.I2C, address: i2c.Address) Self {
            return Self{
                .i2c = i,
                .address = address,
                .frame_data = [_]u8{0} ** (834 * 2),
                .interface = lib.I2C(){
                    .write_then_read_fn = writeThenRead,
                    .write_fn = write,
                },
            };
        }

        pub fn writeThenRead(comm: *lib.I2C(), address: u16, buf: []u16) !void {
            const self: *Self = @fieldParentPtr("interface", comm);

            const req = [2]u8{ @as(u8, @truncate(address >> 8)), @as(u8, @truncate(address & 0xFF)) };
            try self.i2c.write_then_read_blocking(self.address, &req, self.frame_data[0 .. buf.len * 2], null);
            for (0.., buf) |i, _| {
                buf[i] = std.mem.readInt(u16, self.frame_data[i * 2 .. (i * 2) + 2][0..2], .big);
            }
        }

        pub fn write(comm: *lib.I2C(), address: u16, val: u48) !void {
            const self: *Self = @fieldParentPtr("interface", comm);

            const req = [4]u8{
                @as(u8, @truncate(address >> 8)),
                @as(u8, @truncate(address & 0xFF)),
                @as(u8, @truncate(val >> 8)),
                @as(u8, @truncate(val & 0xFF)),
            };

            try self.i2c.write_blocking(self.address, &req, null);
        }
    };
}

pub fn main() !void {
    try init();

    var pi2c: PicoI2C() = PicoI2C().init(&i2c0, i2c.Address.new(0x33));
    var camera = lib.MLX90640().init(&pi2c.interface);

    const sn = try camera.serialNumber();
    std.log.info("camera serial number: 0x{x}", .{sn});

    try camera.setRefreshRate(0b011);
    const rate = try camera.refreshRate();
    std.log.info("camera refresh rate: 0b{b:0>3}", .{rate});

    const rez = try camera.resolution();
    std.log.info("camera resolution: 0b{b:0>2}", .{rez});

    camera.extractParameters() catch |err| {
        std.log.info("error: bad pixels {}", .{err});
        return;
    };

    const emissivity: f32 = 0.95;
    const tr: f32 = 23.15;
    var temp: [834]f32 = undefined;

    var x: [24][32]f32 = undefined;

    while (true) {
        try camera.temperature(&temp, emissivity, tr);
        for (0..24) |i| {
            for (0..32) |j| {
                x[i][j] = temp[i + (i * j)];
            }
        }
        for (0..24) |i| {
            std.log.debug("{d:.3}\n", .{x[i]});
        }
        time.sleep_ms(100);
    }
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
