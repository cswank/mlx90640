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

        pub fn write_then_read(self: *Self, address: u16, buf: []u16) !void {
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
        frame: [834]u16 = undefined,
        i2c: *I2C(),

        pub fn init(i: *I2C()) MLX90640() {
            return .{
                .i2c = i,
            };
        }

        pub fn serial_number(self: *Self) !u48 {
            try self.i2c.write_then_read(0x2407, self.frame[0..3]);
            return @as(u48, self.frame[0]) << 32 |
                @as(u48, self.frame[1]) << 16 |
                @as(u48, self.frame[2]);
        }

        pub fn set_refresh_rate(self: *Self, rate: u16) !void {
            try self.i2c.write_then_read(CONTROL_REGISTER, self.frame[0..1]);
            const val: u16 = (self.frame[0] & ~refresh_rate_mask) | ((rate << 7) & refresh_rate_mask);
            try self.i2c.write(CONTROL_REGISTER, val);
        }

        pub fn refresh_rate(self: *Self) !u3 {
            try self.i2c.write_then_read(CONTROL_REGISTER, self.frame[0..1]);
            const val = self.frame[0] >> 7 & 0b111;
            return @as(u3, @truncate(val));
        }

        pub fn resolution(self: *Self) !u2 {
            try self.i2c.write_then_read(CONTROL_REGISTER, self.frame[0..1]);
            const val = self.frame[0] >> 10 & 0b11;
            return @as(u2, @truncate(val));
        }
    };
}

// fn extractParameters(params: *parameters) void {
//     extract_vdd(params);
//     extract_ptat(params);
//     extract_gain(params);
//     extract_tgc(params);
//     extract_resolution(params);
//     extract_ksta(params);
//     extract_kst0(params);
//     extract_cp(params);
//     // extract_alpha(params);
//     // extract_offset(params);
//     // extract_ktapixel(params);
//     // extract_kvpixel(params);
//     // extract_cilc(params);
// }

// fn image() ![834]u8 {}

// // fn getTa() !u16 {

// // }

// fn pixels() !void {
//     var ready: bool = false;
//     for (frame_loop) |i| {
//         while (!ready) {
//             try write_then_read(STATUS_REGISTER, frame[833..834]);
//             ready = isReady(i, frame[833]);
//             time.sleep_ms(10);
//         }

//         try write_then_read(0x0400, frame[0..832]);
//     }

//     try write_then_read(CONTROL_REGISTER, frame[832..833]);

//     std.log.info("{x}", .{frame[400..410]});
// }

// fn isReady(i: u1, status: u16) bool {
//     return @as(u1, @truncate(status & 0b1)) == i and @as(u1, @truncate((status >> 3) & 0b1)) == 1;
// }

// const parameters = struct {
//     kVdd: i16 = 0,
//     vdd25: i16 = 0,
//     KvPTAT: f32 = 0,
//     KtPTAT: f32 = 0,
//     uvPTAT25: i16 = 0,
//     alphaPTAT: f32 = 0,
//     gainEE: i16 = 0,
//     tgc: f32 = 0,
//     cpKv: f32 = 0,
//     cpKta: f32 = 0,
//     resolutionEE: u8 = 0,
//     calibrationModeEE: u8 = 0,
//     KsTa: f32 = 0,
//     ksTo: [5]f32 = undefined,
//     ct: [5]i16 = undefined,
//     alpha: [768]i16 = undefined,
//     alphaScale: u8 = 0,
//     offset: [768]i16 = undefined,
//     kta: [768]i8 = undefined,
//     ktaScale: u8 = 0,
//     kv: [768]i8 = undefined,
//     kvScale: u8 = 0,
//     cpAlpha: [2]f32 = undefined,
//     cpOffset: [2]i16 = undefined,
//     ilChessC: [3]f32 = undefined,
//     brokenPixels: [5]u16 = undefined,
//     outlierPixels: [5]u16 = undefined,
// };

// fn extract_vdd(params: *parameters) void {
//     params.kVdd = @intCast((eeprom[51] & 0xFF00) >> 8);
//     if (params.kVdd > 127) {
//         params.kVdd = (params.kVdd - 256) * 32;
//     }
//     params.vdd25 = @intCast((((eeprom[51] & 0x00FF) - 256) << 5) - 8192);
// }

// fn extract_ptat(params: *parameters) void {
//     params.KvPTAT = @floatFromInt((eeprom[50] & 0xFC00) >> 10);
//     if (params.KvPTAT > 31) {
//         params.KvPTAT = params.KvPTAT - 64;
//     }
//     params.KvPTAT = params.KvPTAT / 4096;

//     params.KtPTAT = @floatFromInt(eeprom[50] & 0x03FF);
//     if (params.KtPTAT > 511) {
//         params.KtPTAT = params.KtPTAT - 1024;
//     }

//     params.KtPTAT = params.KtPTAT / 8;

//     params.uvPTAT25 = @intCast(eeprom[49]);

//     const x: f32 = @floatFromInt(eeprom[16] & 0xF000);
//     const y: f32 = std.math.pow(f32, 2, 14);
//     params.alphaPTAT = (x / y) + 8.0;
// }

// fn extract_gain(params: *parameters) void {
//     params.gainEE = @intCast(eeprom[48]);
//     if (params.gainEE > 32767) {
//         params.gainEE -= -65536;
//     }
// }

// fn extract_tgc(params: *parameters) void {
//     params.tgc = @floatFromInt(eeprom[60] & 0x00FF);
//     if (params.tgc > 127) {
//         params.tgc -= -256;
//     }
//     params.tgc /= 32.0;
// }

// fn extract_resolution(params: *parameters) void {
//     params.resolutionEE = @truncate((eeprom[56] & 0x3000) >> 12);
// }

// fn extract_ksta(params: *parameters) void {
//     params.KsTa = @floatFromInt((eeprom[60] & 0xFF00) >> 8);
//     if (params.KsTa > 127) {
//         params.KsTa -= 256;
//     }
//     params.KsTa /= 8192.0;
// }

// fn extract_kst0(params: *parameters) void {
//     const step: i16 = @intCast(((eeprom[63] & 0x3000) >> 12) * 10);
//     params.ct[0] = -40;
//     params.ct[1] = 0;
//     params.ct[2] = @intCast((eeprom[63] & 0x00F0) >> 4);
//     params.ct[2] *= step;
//     params.ct[3] = @intCast((eeprom[63] & 0x0F00) >> 8);
//     params.ct[3] *= step;

//     params.ksTo[0] = @floatFromInt(eeprom[61] & 0x00FF);
//     params.ksTo[1] = @floatFromInt((eeprom[61] & 0xFF00) >> 8);
//     params.ksTo[2] = @floatFromInt(eeprom[62] & 0x00FF);
//     params.ksTo[3] = @floatFromInt((eeprom[62] & 0xFF00) >> 8);

//     const x: u4 = @intCast((eeprom[63] & 0x000F) + 8);
//     const y: u16 = @as(u16, 1) << x;
//     const KsToScale: f32 = @floatFromInt(y);

//     for (ksto_loop) |i| {
//         if (params.ksTo[i] > 127) {
//             params.ksTo[i] -= 256;
//         }
//         params.ksTo[i] /= KsToScale;
//     }
//     params.ksTo[4] = -0.0002;
// }

// fn extract_cp(params: *parameters) void {
//     //alphaScale = ((eeprom[32] & 0xF000) >> 12) + 27;
//     params.cpOffset[0] = @intCast(eeprom[58] & 0x03FF);
//     if (params.cpOffset[0] > 511) {
//         params.cpOffset[0] = params.cpOffset[0] - 1024;
//     }

//     params.cpOffset[1] = @intCast((eeprom[58] & 0xFC00) >> 10);
//     if (params.cpOffset[1] > 31) {
//         params.cpOffset[1] = params.cpOffset[1] - 64;
//     }

//     params.cpAlpha[0] = @floatFromInt(eeprom[57] & 0x03FF);
//     if (params.cpAlpha[0] > 511) {
//         params.cpAlpha[0] -= 1024;
//     }

//     params.cpAlpha[1] = @floatFromInt((eeprom[57] & 0xFC00) >> 10);
//     if (params.cpAlpha[1] > 31) {
//         params.cpAlpha[1] -= 64;
//     }

//     params.cpKta = @floatFromInt(eeprom[59] & 0x00FF);
//     if (params.cpKta > 127) {
//         params.cpKta -= 256;
//     }

//     const ktaScale1: f32 = @floatFromInt(((eeprom[56] & 0x00F0) >> 4) + 8);
//     params.cpKta /= std.math.pow(f32, 2, ktaScale1);

//     params.cpKv = @floatFromInt((eeprom[59] & 0xFF00) >> 8);
//     if (params.cpKv > 127) {
//         params.cpKv -= 256;
//     }

//     const kvScale: f32 = @floatFromInt((eeprom[56] & 0x0F00) >> 8);
//     params.cpKv /= std.math.pow(f32, 2, kvScale);
// }

// // fn extract_alpha(params: *parameters) void {

// // }
// // fn extract_offset(params: *parameters) void {}
// // fn extract_ktapixel(params: *parameters) void {}
// // fn extract_kvpixel(params: *parameters) void {}
// // fn extract_cilc(params: *parameters) void {}
