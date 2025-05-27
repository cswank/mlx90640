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
        const Self = @This();

        pub fn init(i: *rp2040.i2c.I2C, address: i2c.Address) Self {
            return Self{
                .i2c = i,
                .address = address,
                .interface = lib.I2C(){
                    .write_then_read_fn = writeThenRead,
                    .write_fn = write,
                },
            };
        }

        pub fn writeThenRead(comm: *lib.I2C(), req: []u8, buf: []u8) !void {
            const self: *Self = @fieldParentPtr("interface", comm);
            try self.i2c.write_then_read_blocking(self.address, req, buf, null);
        }

        pub fn write(comm: *lib.I2C(), buf: []u8) !void {
            const self: *Self = @fieldParentPtr("interface", comm);
            try self.i2c.write_blocking(self.address, buf, null);
        }
    };
}

pub fn main() !void {
    try init();

    var pi2c = PicoI2C().init(&i2c0, i2c.Address.new(0x33));
    var cd = rp2040.drivers.ClockDevice{};
    const cfg = lib.MLX90640_Config{
        .i2c = &pi2c.interface,
        .clock_device = cd.clock_device(),
    };

    var camera = lib.MLX90640().init(cfg);

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

    var temp: [834]f32 = undefined;
    var x: [24][32]f32 = undefined;

    // the temperatures are all wack the first time
    try camera.loadFrame();
    time.sleep_ms(100);

    var maxi: i8 = 0;
    var maxj: i8 = 0;
    var max: f32 = 0;

    while (true) {
        try camera.temperature(&temp);

        for (0..24) |i| {
            for (0..32) |j| {
                x[i][j] = temp[i + (i * j)];
                if (x[i][j] > max) {
                    max = x[i][j];
                    maxi = @intCast(i);
                    maxj = @intCast(j);
                }
            }
        }

        for (0..24) |i| {
            std.log.debug("{d:.3}\n", .{x[i]});
        }
        time.sleep_ms(100);
        maxi = 0;
        maxj = 0;
        max = 0;
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
