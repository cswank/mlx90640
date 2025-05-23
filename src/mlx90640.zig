const std = @import("std");

const SCALEALPHA = 0.000001;
const OPENAIR_TA_SHIFT = 8;

const CONTROL_REGISTER = 0x800D;
const STATUS_REGISTER = 0x8000;

const frame_loop: [2]u1 = [2]u1{ 0, 1 };
const ksto_loop: [4]u2 = [4]u2{ 0, 1, 2, 3 };
const refresh_rate_mask: u16 = 0b111 << 7;

pub fn I2C() type {
    return struct {
        const Self = @This();
        write_then_read_fn: *const fn (self: *Self, address: u16, buf: []u16) anyerror!void,
        write_fn: *const fn (self: *Self, address: u16, val: u48) anyerror!void,

        pub fn writeThenRead(self: *Self, address: u16, buf: []u16) !void {
            return self.write_then_read_fn(self, address, buf);
        }

        pub fn write(self: *Self, address: u16, val: u48) !void {
            return self.write_fn(self, address, val);
        }
    };
}

pub fn MLX90640() type {
    return struct {
        const Self = @This();
        eeprom: [832]u16 = undefined,
        frame: [4]u16 = undefined,
        i2c: *I2C(),
        params: parameters,

        pub fn init(i: *I2C()) MLX90640() {
            return .{
                .i2c = i,
                .params = parameters{},
            };
        }

        pub fn serialNumber(self: *Self) !u48 {
            try self.i2c.writeThenRead(0x2407, self.frame[0..3]);
            return @as(u48, self.frame[0]) << 32 |
                @as(u48, self.frame[1]) << 16 |
                @as(u48, self.frame[2]);
        }

        pub fn setRefreshRate(self: *Self, rate: u16) !void {
            try self.i2c.writeThenRead(CONTROL_REGISTER, self.frame[0..1]);
            const val: u16 = (self.frame[0] & ~refresh_rate_mask) | ((rate << 7) & refresh_rate_mask);
            try self.i2c.write(CONTROL_REGISTER, val);
        }

        pub fn refreshRate(self: *Self) !u3 {
            try self.i2c.writeThenRead(CONTROL_REGISTER, self.frame[0..1]);
            const val = self.frame[0] >> 7 & 0b111;
            return @as(u3, @truncate(val));
        }

        pub fn resolution(self: *Self) !u2 {
            try self.i2c.writeThenRead(CONTROL_REGISTER, self.frame[0..1]);
            const val = self.frame[0] >> 10 & 0b11;
            return @as(u2, @truncate(val));
        }

        pub fn pixels(self: *Self, frame: []u16) !void {
            var ready: bool = false;
            for (frame_loop) |i| {
                while (!ready) {
                    try self.i2c.writeThenRead(STATUS_REGISTER, frame[833..834]);
                    ready = self.isReady(i, frame[833]);
                    //time.sleep_ms(10); how to handle generic time?
                }

                try self.i2c.writeThenRead(0x0400, frame[0..832]);
            }

            try self.i2c.writeThenRead(CONTROL_REGISTER, frame[832..833]);
        }

        fn isReady(_: *Self, i: u1, status: u16) bool {
            return @as(u1, @truncate(status & 0b1)) == i and @as(u1, @truncate((status >> 3) & 0b1)) == 1;
        }

        pub fn extractParameters(self: *Self) !void {
            try self.i2c.writeThenRead(0x2400, &self.eeprom);
            self.extractVdd();
            self.extractPtat();
            self.extractGain();
            self.extractTgc();
            self.extractResolution();
            self.extractKta();
            self.extractKst0();
            self.extractCp();
            // self.alpha(self.params);
            // self.offset(self.params);
            // self.ktapixel(self.params);
            // self.kvpixel(self.params);
            // self.cilc(self.params);
        }

        fn extractVdd(self: *Self) void {
            self.params.kVdd = @intCast((self.eeprom[51] & 0xFF00) >> 8);
            if (self.params.kVdd > 127) {
                self.params.kVdd = (self.params.kVdd - 256) * 32;
            }
            self.params.vdd25 = @intCast((((self.eeprom[51] & 0x00FF) - 256) << 5) - 8192);
        }

        fn extractPtat(self: *Self) void {
            self.params.KvPTAT = @floatFromInt((self.eeprom[50] & 0xFC00) >> 10);
            if (self.params.KvPTAT > 31) {
                self.params.KvPTAT = self.params.KvPTAT - 64;
            }
            self.params.KvPTAT = self.params.KvPTAT / 4096;

            self.params.KtPTAT = @floatFromInt(self.eeprom[50] & 0x03FF);
            if (self.params.KtPTAT > 511) {
                self.params.KtPTAT = self.params.KtPTAT - 1024;
            }

            self.params.KtPTAT = self.params.KtPTAT / 8;

            self.params.uvPTAT25 = @intCast(self.eeprom[49]);

            const x: f32 = @floatFromInt(self.eeprom[16] & 0xF000);
            const y: f32 = std.math.pow(f32, 2, 14);
            self.params.alphaPTAT = (x / y) + 8.0;
        }

        fn extractGain(self: *Self) void {
            self.params.gainEE = @intCast(self.eeprom[48]);
            if (self.params.gainEE > 32767) {
                self.params.gainEE -= -65536;
            }
        }

        fn extractTgc(self: *Self) void {
            self.params.tgc = @floatFromInt(self.eeprom[60] & 0x00FF);
            if (self.params.tgc > 127) {
                self.params.tgc -= -256;
            }
            self.params.tgc /= 32.0;
        }

        fn extractResolution(self: *Self) void {
            self.params.resolutionEE = @truncate((self.eeprom[56] & 0x3000) >> 12);
        }

        fn extractKsta(self: *Self) void {
            self.params.KsTa = @floatFromInt((self.eeprom[60] & 0xFF00) >> 8);
            if (self.params.KsTa > 127) {
                self.params.KsTa -= 256;
            }
            self.params.KsTa /= 8192.0;
        }

        fn extractKst0(self: *Self) void {
            const step: i16 = @intCast(((self.eeprom[63] & 0x3000) >> 12) * 10);
            self.params.ct[0] = -40;
            self.params.ct[1] = 0;
            self.params.ct[2] = @intCast((self.eeprom[63] & 0x00F0) >> 4);
            self.params.ct[2] *= step;
            self.params.ct[3] = @intCast((self.eeprom[63] & 0x0F00) >> 8);
            self.params.ct[3] *= step;

            self.params.ksTo[0] = @floatFromInt(self.eeprom[61] & 0x00FF);
            self.params.ksTo[1] = @floatFromInt((self.eeprom[61] & 0xFF00) >> 8);
            self.params.ksTo[2] = @floatFromInt(self.eeprom[62] & 0x00FF);
            self.params.ksTo[3] = @floatFromInt((self.eeprom[62] & 0xFF00) >> 8);

            const x: u4 = @intCast((self.eeprom[63] & 0x000F) + 8);
            const y: u16 = @as(u16, 1) << x;
            const KsToScale: f32 = @floatFromInt(y);

            for (ksto_loop) |i| {
                if (self.params.ksTo[i] > 127) {
                    self.params.ksTo[i] -= 256;
                }
                self.params.ksTo[i] /= KsToScale;
            }
            self.params.ksTo[4] = -0.0002;
        }

        fn extractCp(self: *Self) void {
            //alphaScale = ((self.eeprom[32] & 0xF000) >> 12) + 27;
            self.params.cpOffset[0] = @intCast(self.eeprom[58] & 0x03FF);
            if (self.params.cpOffset[0] > 511) {
                self.params.cpOffset[0] = self.params.cpOffset[0] - 1024;
            }

            self.params.cpOffset[1] = @intCast((self.eeprom[58] & 0xFC00) >> 10);
            if (self.params.cpOffset[1] > 31) {
                self.params.cpOffset[1] = self.params.cpOffset[1] - 64;
            }

            self.params.cpAlpha[0] = @floatFromInt(self.eeprom[57] & 0x03FF);
            if (self.params.cpAlpha[0] > 511) {
                self.params.cpAlpha[0] -= 1024;
            }

            self.params.cpAlpha[1] = @floatFromInt((self.eeprom[57] & 0xFC00) >> 10);
            if (self.params.cpAlpha[1] > 31) {
                self.params.cpAlpha[1] -= 64;
            }

            self.params.cpKta = @floatFromInt(self.eeprom[59] & 0x00FF);
            if (self.params.cpKta > 127) {
                self.params.cpKta -= 256;
            }

            const ktaScale1: f32 = @floatFromInt(((self.eeprom[56] & 0x00F0) >> 4) + 8);
            self.params.cpKta /= std.math.pow(f32, 2, ktaScale1);

            self.params.cpKv = @floatFromInt((self.eeprom[59] & 0xFF00) >> 8);
            if (self.params.cpKv > 127) {
                self.params.cpKv -= 256;
            }

            const kvScale: f32 = @floatFromInt((self.eeprom[56] & 0x0F00) >> 8);
            self.params.cpKv /= std.math.pow(f32, 2, kvScale);
        }
    };
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

// // fn extract_alpha(self.params: *parameters) void {

// // }
// // fn extract_offset(self.params: *parameters) void {}
// // fn extract_ktapixel(self.params: *parameters) void {}
// // fn extract_kvpixel(self.params: *parameters) void {}
// // fn extract_cilc(self.params: *parameters) void {}
