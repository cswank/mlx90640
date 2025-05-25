const std = @import("std");

const SCALEALPHA = 0.000001;
const OPENAIR_TA_SHIFT = 8;

const CONTROL_REGISTER = 0x800D;
const STATUS_REGISTER = 0x8000;

const frame_loop: [2]u1 = [2]u1{ 0, 1 };
const ksto_loop: [4]u2 = [4]u2{ 0, 1, 2, 3 };
const refresh_rate_mask: u16 = 0b111 << 7;

const Mlx90649Error = error{
    BadPixels,
};

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
        frame: [834]u16 = undefined,
        scratch_data: [768]f32 = undefined,
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
            self.extractAlpha();
            self.extractOffset();
            self.extractKtaPixel();
            self.extractKvpixel();
            self.extractCilc();
            const err: i16 = self.extractDeviatingPixels();
            if (err > 0) {
                return Mlx90649Error.BadPixels;
            }
        }

        fn extractVdd(self: *Self) void {
            self.params.kVdd = @intCast((self.eeprom[51] & 0xFF00) >> 8);
            if (self.params.kVdd > 127) {
                self.params.kVdd = (self.params.kVdd - 256);
            }

            self.params.kVdd *= 32;

            var vdd25: i32 = self.eeprom[51] & 0x00FF;
            vdd25 = ((vdd25 - 256) << 5) - 8192;
            self.params.vdd25 = @intCast(vdd25);
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

        fn extractKta(self: *Self) void {
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

            const x: u5 = @intCast((self.eeprom[63] & 0x000F) + 8);
            const y: u32 = @as(u32, 1) << x;
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
            const alphaScale: u16 = ((self.eeprom[32] & 0xF000) >> 12) + 27;

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

            self.params.cpAlpha[0] /= (std.math.pow(f32, 2, @floatFromInt(alphaScale)));

            self.params.cpAlpha[1] = @floatFromInt((self.eeprom[57] & 0xFC00) >> 10);
            if (self.params.cpAlpha[1] > 31) {
                self.params.cpAlpha[1] -= 64;
            }

            self.params.cpAlpha[1] = (1 + self.params.cpAlpha[1] / 128) * self.params.cpAlpha[0];

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

        fn extractAlpha(self: *Self) void {
            const accRemScale: u3 = @intCast(self.eeprom[32] & 0x0F);
            const accColumnScale: u4 = @intCast((self.eeprom[32] & 0x00F0) >> 4);
            const accRowScale: u4 = @intCast((self.eeprom[32] & 0x0F00) >> 8);
            var alphaScale: f32 = @floatFromInt(((self.eeprom[32] & 0xF000) >> 12) + 30);
            const alphaRef: i32 = @intCast(self.eeprom[33]);

            var accRow: [24]i16 = undefined;
            var accColumn: [32]i16 = undefined;

            for (0..6) |i| {
                const p = i * 4;
                accRow[p + 0] = @intCast(self.eeprom[34 + i] & 0x000F);
                accRow[p + 1] = @intCast((self.eeprom[34 + i] & 0x00F0) >> 4);
                accRow[p + 2] = @intCast((self.eeprom[34 + i] & 0x0F00) >> 8);
                accRow[p + 3] = @intCast((self.eeprom[34 + i] & 0xF000) >> 12);
            }

            for (0..24) |i| {
                if (accRow[i] > 7) {
                    accRow[i] = accRow[i] - 16;
                }
            }

            for (0..8) |i| {
                const p = i * 4;
                accColumn[p + 0] = @intCast(self.eeprom[40 + i] & 0x000F);
                accColumn[p + 1] = @intCast((self.eeprom[40 + i] & 0x00F0) >> 4);
                accColumn[p + 2] = @intCast((self.eeprom[40 + i] & 0x0F00) >> 8);
                accColumn[p + 3] = @intCast((self.eeprom[40 + i] & 0xF000) >> 12);
            }

            for (0..32) |i| {
                if (accColumn[i] > 7) {
                    accColumn[i] = accColumn[i] - 16;
                }
            }

            for (0..24) |i| {
                for (0..32) |j| {
                    const p = 32 * i + j;
                    self.scratch_data[p] = @floatFromInt((self.eeprom[64 + p] & 0x03F0) >> 4);
                    if (self.scratch_data[p] > 31) {
                        self.scratch_data[p] = self.scratch_data[p] - 64;
                    }
                    const x: f32 = @floatFromInt(@as(u8, 1) << accRemScale);
                    self.scratch_data[p] = self.scratch_data[p] * x;
                    const y: f32 = @floatFromInt((alphaRef + (accRow[i] << accRowScale) + (accColumn[j] << accColumnScale)));
                    self.scratch_data[p] = y + self.scratch_data[p];
                    self.scratch_data[p] /= std.math.pow(f32, 2, alphaScale);
                    self.scratch_data[p] = self.scratch_data[p] - self.params.tgc * (self.params.cpAlpha[0] + self.params.cpAlpha[1]) / 2;
                    self.scratch_data[p] = SCALEALPHA / self.scratch_data[p];
                }
            }

            var temp = self.scratch_data[0];
            for (1..768) |i| {
                if (self.scratch_data[i] > temp) {
                    temp = self.scratch_data[i];
                }
            }

            alphaScale = 0;
            while (temp < 32768) {
                temp *= 2;
                alphaScale += alphaScale;
            }

            for (0..768) |i| {
                temp = self.scratch_data[i] * std.math.pow(f32, 2, alphaScale);
                self.params.alpha[i] = @intFromFloat(temp + 0.5);
            }

            self.params.alphaScale = @intFromFloat(alphaScale);
        }

        fn extractOffset(self: *Self) void {
            var occRow: [24]i16 = undefined;
            var occColumn: [32]i16 = undefined;
            const occRemScale: u3 = @intCast(self.eeprom[16] & 0x000F);
            const occColumnScale: u4 = @intCast((self.eeprom[16] & 0x00F0) >> 4);
            const occRowScale: u4 = @intCast((self.eeprom[16] & 0x0F00) >> 8);
            var offsetRef: i32 = @intCast(self.eeprom[17]);

            if (offsetRef > 32767) {
                offsetRef -= 65536;
            }

            for (0..6) |i| {
                const p = i * 4;
                occRow[p + 0] = @intCast(self.eeprom[18 + i] & 0x000F);
                occRow[p + 1] = @intCast((self.eeprom[18 + i] & 0x00F0) >> 4);
                occRow[p + 2] = @intCast((self.eeprom[18 + i] & 0x0F00) >> 8);
                occRow[p + 3] = @intCast((self.eeprom[18 + i] & 0xF000) >> 12);
            }

            for (0..24) |i| {
                if (occRow[i] > 7) {
                    occRow[i] -= 16;
                }
            }

            for (0..8) |i| {
                const p = i * 4;
                occColumn[p + 0] = @intCast(self.eeprom[24 + i] & 0x000F);
                occColumn[p + 1] = @intCast((self.eeprom[24 + i] & 0x00F0) >> 4);
                occColumn[p + 2] = @intCast((self.eeprom[24 + i] & 0x0F00) >> 8);
                occColumn[p + 3] = @intCast((self.eeprom[24 + i] & 0xF000) >> 12);
            }

            for (0..32) |i| {
                if (occColumn[i] > 7) {
                    occColumn[i] = occColumn[i] - 16;
                }
            }

            for (0..24) |i| {
                for (0..32) |j| {
                    const p = 32 * i + j;
                    self.params.offset[p] = @intCast((self.eeprom[64 + p] & 0xFC00) >> 10);
                    if (self.params.offset[p] > 31) {
                        self.params.offset[p] = self.params.offset[p] - 64;
                    }
                    self.params.offset[p] = self.params.offset[p] * (@as(u8, 1) << occRemScale);

                    const x: i32 = offsetRef + @as(i32, (occRow[i] << occRowScale));
                    const y: i32 = (occColumn[j] << occColumnScale);
                    const z: i32 = self.params.offset[p];
                    self.params.offset[p] = x + y + z;
                }
            }
        }

        fn extractKtaPixel(self: *Self) void {
            var ktaRC: [4]i8 = undefined;
            var ktaRoCo: i8 = @intCast((self.eeprom[54] & 0xFF00) >> 8);
            if (ktaRoCo > 127) {
                ktaRoCo = ktaRoCo - 256;
            }
            ktaRC[0] = ktaRoCo;

            var ktaReCo: i8 = @intCast(self.eeprom[54] & 0xFF);
            if (ktaReCo > 127) {
                ktaReCo = ktaReCo - 256;
            }
            ktaRC[2] = ktaReCo;

            var ktaRoCe: i8 = @intCast((self.eeprom[55] & 0xFF00) >> 8);
            if (ktaRoCe > 127) {
                ktaRoCe = ktaRoCe - 256;
            }
            ktaRC[1] = ktaRoCe;

            var ktaReCe: i8 = @intCast((self.eeprom[55] & 0xFF));
            if (ktaReCe > 127) {
                ktaReCe = ktaReCe - 256;
            }
            ktaRC[3] = ktaReCe;

            var ktaScale1: u8 = @intCast(((self.eeprom[56] & 0x00F0) >> 4) + 8);
            const ktaScale2: u3 = @intCast(self.eeprom[56] & 0x000F);

            for (0..24) |i| {
                for (0..32) |j| {
                    const p = 32 * i + j;
                    const split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
                    self.scratch_data[p] = @floatFromInt((self.eeprom[64 + p] & 0x000E) >> 1);
                    if (self.scratch_data[p] > 3) {
                        self.scratch_data[p] = self.scratch_data[p] - 8;
                    }
                    const x: f32 = @floatFromInt(@as(u8, 1) << ktaScale2);
                    self.scratch_data[p] = self.scratch_data[p] * x;
                    const y: f32 = @floatFromInt(ktaRC[split]);
                    self.scratch_data[p] = y + self.scratch_data[p];
                    self.scratch_data[p] = self.scratch_data[p] / std.math.pow(f32, 2, @floatFromInt(ktaScale1));
                    //self.scratch_data[p] = self.scratch_data[p] * self.params.offset[p];
                }
            }

            var temp: f32 = @abs(self.scratch_data[0]);
            for (1..768) |i| {
                if (@abs(self.scratch_data[i]) > temp) {
                    temp = @abs(self.scratch_data[i]);
                }
            }

            ktaScale1 = 0;
            while (temp < 64) {
                temp = temp * 2;
                ktaScale1 = ktaScale1 + 1;
            }

            for (0..768) |i| {
                temp = self.scratch_data[i] * std.math.pow(f32, 2, @floatFromInt(ktaScale1));
                if (temp < 0) {
                    self.params.kta[i] = @intFromFloat(temp - 0.5);
                } else {
                    self.params.kta[i] = @intFromFloat(temp + 0.5);
                }
            }

            self.params.ktaScale = ktaScale1;
        }

        fn extractKvpixel(self: *Self) void {
            var KvT: [4]u8 = undefined;
            var KvRoCo: i8 = @intCast((self.eeprom[52] & 0xF000) >> 12);
            if (KvRoCo > 7) {
                KvRoCo = KvRoCo - 16;
            }
            KvT[0] = @intCast(KvRoCo);

            var KvReCo: i8 = @intCast((self.eeprom[52] & 0x0F00) >> 8);
            if (KvReCo > 7) {
                KvReCo = KvReCo - 16;
            }
            KvT[2] = @intCast(KvReCo);

            var KvRoCe: i8 = @intCast((self.eeprom[52] & 0x00F0) >> 4);
            if (KvRoCe > 7) {
                KvRoCe = KvRoCe - 16;
            }
            KvT[1] = @intCast(KvRoCe);

            var KvReCe: i8 = @intCast(self.eeprom[52] & 0x000F);
            if (KvReCe > 7) {
                KvReCe = KvReCe - 16;
            }
            KvT[3] = @intCast(KvReCe);

            var kvScale: u8 = @intCast((self.eeprom[56] & 0x0F00) >> 8);

            for (0..24) |i| {
                for (0..32) |j| {
                    const p = 32 * i + j;
                    const split = 2 * (p / 32 - (p / 64) * 2) + p % 2;
                    self.scratch_data[p] = @floatFromInt(KvT[split]);
                    self.scratch_data[p] = self.scratch_data[p] / std.math.pow(f32, 2, @floatFromInt(kvScale));
                    //self.scratch_data[p] = self.scratch_data[p] * mlx90640->offset[p];
                }
            }

            var temp: f32 = @abs(self.scratch_data[0]);
            for (1..768) |i| {
                if (@abs(self.scratch_data[i]) > temp) {
                    temp = @abs(self.scratch_data[i]);
                }
            }

            kvScale = 0;
            while (temp < 64) {
                temp = temp * 2;
                kvScale = kvScale + 1;
            }

            for (0..768) |i| {
                temp = self.scratch_data[i] * std.math.pow(f32, 2, @floatFromInt(kvScale));
                if (temp < 0) {
                    self.params.kv[i] = @intFromFloat(temp - 0.5);
                } else {
                    self.params.kv[i] = @intFromFloat(temp + 0.5);
                }
            }

            self.params.kvScale = kvScale;
        }

        fn extractCilc(self: *Self) void {
            var offsetSP: [2]i16 = undefined;
            var alphaSP: [2]f32 = undefined;
            const alphaScale: u8 = @intCast(((self.eeprom[32] & 0xF000) >> 12) + 27);

            offsetSP[0] = @intCast(self.eeprom[58] & 0x03FF);
            if (offsetSP[0] > 511) {
                offsetSP[0] = offsetSP[0] - 1024;
            }

            offsetSP[1] = @intCast((self.eeprom[58] & 0xFC00) >> 10);
            if (offsetSP[1] > 31) {
                offsetSP[1] = offsetSP[1] - 64;
            }
            offsetSP[1] = offsetSP[1] + offsetSP[0];

            alphaSP[0] = @floatFromInt(self.eeprom[57] & 0x03FF);
            if (alphaSP[0] > 511) {
                alphaSP[0] = alphaSP[0] - 1024;
            }
            alphaSP[0] = alphaSP[0] / std.math.pow(f32, 2, @floatFromInt(alphaScale));

            alphaSP[1] = @floatFromInt((self.eeprom[57] & 0xFC00) >> 10);
            if (alphaSP[1] > 31) {
                alphaSP[1] = alphaSP[1] - 64;
            }
            alphaSP[1] = (1 + alphaSP[1] / 128) * alphaSP[0];

            self.params.cpKta = @floatFromInt(self.eeprom[59] & 0x00FF);
            if (self.params.cpKta > 127) {
                self.params.cpKta = self.params.cpKta - 256;
            }
            const ktaScale1: u8 = @intCast(((self.eeprom[56] & 0x00F0) >> 4) + 8);
            self.params.cpKta /= std.math.pow(f32, 2, @floatFromInt(ktaScale1));

            var cpKv: f32 = @floatFromInt((self.eeprom[59] & 0xFF00) >> 8);
            if (cpKv > 127) {
                cpKv = cpKv - 256;
            }
            const kvScale: u8 = @intCast((self.eeprom[56] & 0x0F00) >> 8);
            self.params.cpKv = cpKv / std.math.pow(f32, 2, @floatFromInt(kvScale));

            self.params.cpAlpha[0] = alphaSP[0];
            self.params.cpAlpha[1] = alphaSP[1];
            self.params.cpOffset[0] = offsetSP[0];
            self.params.cpOffset[1] = offsetSP[1];
        }

        fn extractDeviatingPixels(self: *Self) i16 {
            var pixCnt: u32 = 0;
            for (0..5) |i| {
                pixCnt = @intCast(i);
                self.params.brokenPixels[pixCnt] = 0xFFFF;
                self.params.outlierPixels[pixCnt] = 0xFFFF;
            }

            var brokenPixCnt: u16 = 0;
            var outlierPixCnt: u16 = 0;
            pixCnt = 0;
            while (pixCnt < 768 and brokenPixCnt < 5 and outlierPixCnt < 5) {
                if (self.eeprom[pixCnt + 64] == 0) {
                    self.params.brokenPixels[brokenPixCnt] = @intCast(pixCnt);
                    brokenPixCnt = brokenPixCnt + 1;
                } else if ((self.eeprom[pixCnt + 64] & 0x0001) != 0) {
                    self.params.outlierPixels[outlierPixCnt] = @intCast(pixCnt);
                    outlierPixCnt = outlierPixCnt + 1;
                }

                pixCnt = pixCnt + 1;
            }

            var warn: i16 = 0;
            if (brokenPixCnt > 4) {
                // Serial.print("Broken pixels: ");
                // Serial.println(brokenPixCnt);
                warn = -3;
            } else if (outlierPixCnt > 4) {
                //Serial.print("Outlier pixels: ");
                //Serial.println(outlierPixCnt);
                warn = -4;
            } else if ((brokenPixCnt + outlierPixCnt) > 4) {
                //Serial.print("Broken+outlier pixels: ");
                //Serial.println(brokenPixCnt + outlierPixCnt);
                warn = -5;
            } else {
                for (0..brokenPixCnt) |x| {
                    pixCnt = @intCast(x);
                    for (pixCnt + 1..brokenPixCnt) |i| {
                        warn = self.checkAdjacentPixels(self.params.brokenPixels[pixCnt], self.params.brokenPixels[i]);
                        if (warn != 0) {
                            //Serial.println("Broken pixel has adjacent broken pixel");
                            return warn;
                        }
                    }
                }

                for (0..outlierPixCnt) |x| {
                    pixCnt = @intCast(x);
                    for (pixCnt + 1..outlierPixCnt) |i| {
                        warn = self.checkAdjacentPixels(self.params.outlierPixels[pixCnt], self.params.outlierPixels[i]);
                        if (warn != 0) {
                            //Serial.println("Outlier pixel has adjacent outlier pixel");
                            return warn;
                        }
                    }
                }

                for (0..brokenPixCnt) |x| {
                    pixCnt = @intCast(x);
                    for (0..outlierPixCnt) |i| {
                        warn = self.checkAdjacentPixels(self.params.brokenPixels[pixCnt], self.params.outlierPixels[i]);
                        if (warn != 0) {
                            //Serial.println("Broken pixel has adjacent outlier pixel");
                            return warn;
                        }
                    }
                }
            }

            return warn;
        }

        fn checkAdjacentPixels(_: *Self, pix1: u16, pix2: u16) i16 {
            var pixPosDif: i32 = 0;

            pixPosDif = pix1 - pix2;
            if (pixPosDif > -34 and pixPosDif < -30) {
                return -6;
            }
            if (pixPosDif > -2 and pixPosDif < 2) {
                return -6;
            }
            if (pixPosDif > 30 and pixPosDif < 34) {
                return -6;
            }

            return 0;
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
    alpha: [768]u16 = undefined,
    alphaScale: u8 = 0,
    offset: [768]i32 = undefined,
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

fn MockI2C() type {
    return struct {
        eeprom: [832]u16,
        interface: I2C(),
        const Self = @This();

        pub fn init(e: [832]u16) Self {
            return Self{
                .eeprom = e,
                .interface = I2C(){
                    .write_then_read_fn = writeThenRead,
                    .write_fn = write,
                },
            };
        }

        pub fn writeThenRead(comm: *I2C(), _: u16, buf: []u16) !void {
            const self: *Self = @fieldParentPtr("interface", comm);
            for (0.., self.eeprom) |i, _| {
                buf[i] = self.eeprom[i];
            }
        }

        pub fn write(_: *I2C(), _: u16, _: u48) !void {
            //const self: *Self = @fieldParentPtr("interface", comm);
        }
    };
}

test "extract params" {
    const eepromTestData = [832]u16{ 0xb5, 0xa99f, 0x0, 0x2061, 0x5, 0x320, 0x3e0, 0x191f, 0x9c76, 0x18b, 0x48d, 0x0, 0x1901, 0x0, 0x0, 0xbe33, 0x4220, 0xffc0, 0x1213, 0x202, 0xf202, 0xe1f2, 0xd1e1, 0x90c0, 0x1122, 0x112, 0xf002, 0xe0f1, 0xe0f2, 0xe0f2, 0xe0e2, 0xc1e1, 0x79a6, 0x2ea7, 0xedbb, 0x110f, 0x3322, 0x3333, 0x122, 0xcdef, 0xddcb, 0xffe, 0x2111, 0x2222, 0x2233, 0x1122, 0xff00, 0xcdee, 0x1767, 0x2fa3, 0x3553, 0xa584, 0x4433, 0x18c, 0x3533, 0x3030, 0x2352, 0x74, 0x17bf, 0x320, 0xec00, 0xf300, 0x9ec5, 0x2558, 0x760, 0xfb12, 0xc5e, 0xe8cc, 0xf8d0, 0xf420, 0xc7e, 0xf3ae, 0xf78e, 0x370, 0x140e, 0xf6fe, 0xffc0, 0x3c0, 0x13be, 0xfbce, 0xfaf0, 0xfb00, 0xfce, 0xefbe, 0xf770, 0xf360, 0xfc0, 0xebae, 0xf830, 0xfbf0, 0x850, 0xe01e, 0xf480, 0xe850, 0xfc62, 0xe4f0, 0xff2, 0xfbd0, 0xecdc, 0x110, 0xfd24, 0xf46e, 0xf0cc, 0xbf0, 0xfbd2, 0x3c0, 0xf46c, 0x1340, 0x20, 0x42e, 0xf42c, 0x1020, 0x354, 0xfb6e, 0xf02c, 0xc00, 0xfbd4, 0xf7b0, 0xf42e, 0x7f0, 0x84, 0x40, 0xf49e, 0x50, 0xc2, 0xf490, 0xe4ae, 0x550, 0x1050, 0x50, 0x1c7e, 0xf4be, 0xca0, 0x7f0, 0x149e, 0xfbde, 0xf62, 0xbb0, 0x201e, 0xff7e, 0x13d0, 0xc10, 0x1c20, 0x4e, 0x750, 0xb30, 0x182e, 0xffee, 0x3b2, 0xffb0, 0x181e, 0xf3ee, 0x452, 0x422, 0x1450, 0xec0e, 0x850, 0xfc42, 0xc22, 0xf100, 0x884, 0xf04e, 0xec4c, 0x70, 0x72, 0xf7be, 0xe47c, 0x3ae, 0x342, 0xfb8e, 0xf7fc, 0xb40, 0x7c2, 0x3ee, 0xf00c, 0xc20, 0xff42, 0xfb1e, 0xec0e, 0xfc0, 0xfb92, 0xf39e, 0xeffe, 0x3b0, 0xfc24, 0xfff0, 0xf01e, 0xffc0, 0x412, 0xf000, 0xe7ee, 0xc2, 0x18f0, 0x50, 0x183e, 0xf47e, 0x1080, 0x7c0, 0x1850, 0xffa0, 0x7b0, 0xbc0, 0x202e, 0xff9e, 0x1bf2, 0x1030, 0x185e, 0x46e, 0xb92, 0x770, 0x1c10, 0xfc0e, 0x2, 0xfff0, 0x1810, 0xfbde, 0x832, 0x422, 0x1810, 0xeffe, 0x40, 0xfc10, 0xbe0, 0xf0c0, 0x18b2, 0xfc0e, 0xffee, 0xc20, 0x1042, 0x38e, 0xf82c, 0x1360, 0xb82, 0x79e, 0x7fe, 0x136e, 0x1bd2, 0xc00, 0xfc3e, 0x1830, 0xf64, 0x74e, 0xfffe, 0x1be0, 0x7d2, 0xffce, 0xffde, 0x13a0, 0x13f4, 0x7f0, 0x3de, 0xbb0, 0x802, 0xffc0, 0xf790, 0x870, 0xc80, 0xfc12, 0x1bf0, 0xf03e, 0x830, 0x790, 0x1050, 0xf7ae, 0x380, 0x7c0, 0x185e, 0xfb7e, 0x840, 0x850, 0x147e, 0xfc8e, 0xfbb0, 0x380, 0x1440, 0xf45e, 0xffe2, 0xffb0, 0x1400, 0xf7c0, 0x22, 0x2, 0x1bd0, 0xefae, 0x3f0, 0xffa0, 0xf82, 0xf44e, 0xcb2, 0xf030, 0xf02c, 0x50, 0x462, 0xfbc0, 0xec9e, 0x7e0, 0x3c2, 0xfc00, 0xf89c, 0x1390, 0x492, 0x49e, 0xf4ce, 0x14b0, 0xfbf2, 0x3c0, 0xf48e, 0x890, 0x24, 0xf800, 0xf44e, 0xc10, 0x464, 0x40, 0xf80e, 0x7d2, 0x422, 0xffd0, 0xf3ae, 0x862, 0x880, 0x3e2, 0x1bce, 0xf03e, 0x430, 0x3a2, 0x1050, 0xf7be, 0x770, 0x7f0, 0x14b0, 0xf7ce, 0xc30, 0x4a0, 0x10ae, 0xf8ce, 0xffe2, 0xffe0, 0xcae, 0xf09e, 0xf800, 0xf810, 0xc50, 0xf00e, 0x32, 0x22, 0x1010, 0xe7de, 0x3e0, 0x372, 0xf42, 0xf030, 0x4a2, 0xeffe, 0xefee, 0xf850, 0x64, 0xf7d0, 0xe88e, 0x3e0, 0xffd2, 0xf820, 0xecee, 0x7f0, 0x862, 0xfcde, 0xecde, 0xcf0, 0xf814, 0xf40e, 0xe8de, 0x4c0, 0xf832, 0xf04e, 0xec8e, 0x430, 0x464, 0xfc50, 0xf03e, 0xfff0, 0x404, 0x390, 0xef60, 0x452, 0x1432, 0xffb2, 0x1780, 0xf3e0, 0x812, 0x372, 0xc30, 0xf760, 0xffc0, 0x3e2, 0xc9e, 0xf79e, 0x870, 0x842, 0x4de, 0xf89e, 0xfbd0, 0xffd2, 0x8b0, 0xf080, 0xffe2, 0xffd2, 0x840, 0xeffe, 0xf450, 0x3f2, 0xbce, 0xeb70, 0x390, 0xff62, 0x332, 0xf3e0, 0x1c54, 0xf7b0, 0xfbae, 0x400, 0xc24, 0xff8e, 0xf05e, 0xf90, 0x7e2, 0x3f0, 0xf8be, 0x13a0, 0x1492, 0x860, 0xf0fe, 0x18b0, 0x7f2, 0x3e0, 0xf0de, 0x1090, 0x804, 0xfff0, 0xf45e, 0x1002, 0x474, 0x800, 0xffee, 0xb80, 0x13a2, 0x770, 0xf74e, 0x17f2, 0xfc62, 0xffc2, 0xfb0, 0xefe0, 0xf460, 0xff92, 0x460, 0xf390, 0xf7d0, 0xfc00, 0x1080, 0xeffe, 0xfc60, 0xc2, 0x4f0, 0xf0ee, 0xf012, 0xf822, 0x880, 0xec90, 0xec32, 0xf7f2, 0x450, 0xe800, 0xf842, 0xfc22, 0xbf0, 0xe7be, 0xf7d0, 0xfb82, 0xfb70, 0xec10, 0x72, 0xebc0, 0xebbe, 0x3e0, 0xf862, 0xf790, 0xe47e, 0x792, 0xfbd2, 0xf800, 0xf08e, 0x7f0, 0x462, 0xfcc0, 0xe8fe, 0xce0, 0xf812, 0xf820, 0xec8e, 0x890, 0xf832, 0xf800, 0xf05e, 0x800, 0x844, 0x20, 0xf7fe, 0x7b0, 0x7e2, 0xff80, 0xeb7e, 0xc12, 0x50, 0xba2, 0x1b80, 0xfbe0, 0xbe0, 0xb62, 0x860, 0xfb9e, 0xffb0, 0xbf0, 0x14a0, 0xff9e, 0x830, 0x8c2, 0x10ae, 0xb0, 0xffc2, 0x7c0, 0xc90, 0xf870, 0xf802, 0x2, 0x1010, 0xf010, 0xfc50, 0x812, 0x17d0, 0xefa0, 0x7a2, 0xb52, 0xf22, 0xf800, 0xfc42, 0xf790, 0xf37e, 0x3e0, 0xbd2, 0xfb50, 0xe84e, 0xb80, 0xffa2, 0xffe0, 0xf09e, 0x1380, 0xc22, 0xa0, 0xf09e, 0x1490, 0x7b2, 0x3b0, 0xf07e, 0x1050, 0x7f4, 0xfff0, 0xf80e, 0xff0, 0x444, 0x800, 0xffce, 0xb90, 0x1794, 0xf40, 0xf720, 0x17f2, 0x872, 0xbc4, 0x13b0, 0xfbf0, 0x20, 0xb52, 0x450, 0xf780, 0xffa0, 0x7e2, 0x890, 0xfba0, 0x822, 0xc52, 0x870, 0xfcb0, 0xffc2, 0x7d2, 0x8b0, 0xf47e, 0xfbf2, 0xfc22, 0xc30, 0xf02e, 0xfc42, 0xff2, 0xbd0, 0xefc0, 0xffe0, 0x782, 0x360, 0xf420, 0xfc62, 0xf3b0, 0xeb9e, 0x3d2, 0x7c2, 0xfb20, 0xdc3e, 0x362, 0xfb82, 0xf7b0, 0xe86e, 0x782, 0x802, 0x20, 0xec5e, 0x880, 0x394, 0xfba0, 0xe49e, 0xc50, 0x3d4, 0xf7f0, 0xf00e, 0xbf2, 0x424, 0x7c0, 0xf7be, 0x3a0, 0xbb4, 0x360, 0xef5e, 0xc10, 0xf080, 0xfff4, 0xba0, 0xe830, 0xf410, 0x372, 0x20, 0xf380, 0xf360, 0x3c2, 0x830, 0xf370, 0xffd0, 0x802, 0x840, 0xf850, 0xf762, 0xffa2, 0x810, 0xf020, 0xf3c0, 0xfbd2, 0x7f0, 0xefd0, 0xf402, 0xbc2, 0x7f0, 0xef90, 0xfbb0, 0xffa2, 0xfb72, 0xf430, 0xf8d2, 0xe850, 0xe80e, 0xf892, 0xf872, 0xfbd0, 0xe46e, 0x7d0, 0xfba2, 0xfc00, 0xf07e, 0xfb2, 0x822, 0x840, 0xec9e, 0x1892, 0x7a4, 0xffe0, 0xf05e, 0x1062, 0x14, 0xfc10, 0xf44e, 0x1012, 0x864, 0xc10, 0xf44e, 0x13e2, 0x1012, 0xbf0, 0xf3d0, 0x1880, 0xec80, 0x434, 0x43e, 0xf472, 0xf470, 0x7d2, 0x840, 0xff90, 0xfb72, 0xbc2, 0x1040, 0xff50, 0x2, 0x1412, 0xc4e, 0x440, 0xfb72, 0xb82, 0x850, 0xf840, 0xf7c0, 0x7d2, 0x13e0, 0xffb0, 0xfc22, 0x13f2, 0xc00, 0xfbb0, 0x10, 0xff2, 0x7c2, 0x90, 0xec62, 0xec60, 0xdc8e, 0xfcc0, 0xf4c2, 0xf420, 0xe88e, 0xbc2, 0xffb2, 0xfc00, 0xf08e, 0x1782, 0x842, 0xc50, 0xf08e, 0x1c70, 0x3a4, 0x7a0, 0xec8e, 0x1472, 0x2, 0x2, 0xf82e, 0x1bf0, 0x874, 0x1430, 0xf86e, 0x1802, 0x1462, 0x1042, 0xf420, 0x24e2, 0xfb42, 0xb44, 0xfc40, 0xec90, 0xf470, 0x3d2, 0xf870, 0xf390, 0xf750, 0xb92, 0x40e, 0xf740, 0xfbde, 0xff2, 0xe, 0x3e0, 0xf730, 0xb42, 0x7f0, 0xfbe0, 0xf7a0, 0x782, 0xbd0, 0xfbb0, 0xec30, 0xc12, 0x2, 0xf3f0, 0xf050, 0xc12, 0xec10, 0xfcf0, 0xff42, 0xf710, 0xe05e, 0xe0, 0xfcc2, 0xf820, 0xdcce, 0xbe2, 0x792, 0x3d0, 0xf43e, 0xf70, 0xc02, 0xc20, 0xf44e, 0x1c20, 0xb72, 0xb70, 0xf43e, 0x1c10, 0xbe2, 0xfb0, 0xfc1e, 0x1ff0, 0x882, 0x1060, 0x6e, 0x1040, 0x14b2, 0x1460, 0xf870, 0x1d40 };
    var mi2c: MockI2C() = MockI2C().init(eepromTestData);
    var camera = MLX90640().init(&mi2c.interface);
    try camera.extractParameters();
}
//
