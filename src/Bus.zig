const std = @import("std");

const Cartridge = @import("Cartridge");
const Cpu = @import("Cpu");

const Bus = @This();
pub const Address = u16;

//cpu: *Cpu,
//cartridge: *Cartridge,
//ram: [0x2000]u8 = .{0} ** 0x2000,

ram: [0xFFFF]u8 = .{0} ** 0xFFFF,

// pub fn read(self: Bus, addr: Address) u8 {
//     return switch (addr) {
//         // account for memory mirroring every 0x0800 bytes
//         //0x0000...0x1FFF => self.ram[addr & 0x07FF],
//         //0x8000...0xFFFF => self.read_prg_rom(addr),
//         else => 0,
//     };
// }

// pub fn write(self: *Bus, addr: Address, value: u8) void {
//     switch (addr) {
//         // account for memory mirroring every 0x0800 bytes
//         0x0000...0x1FFF => self.ram[addr & 0x0800] = value,
//         0x8000...0xFFFF => {
//             std.log.info("error: bus: wrong memory access 1", .{});
//             unreachable;
//         },
//         else => {
//             std.log.info("error: bus: wrong memory access else", .{});
//             unreachable;
//         },
//     }
// }

pub fn read(self: Bus, addr: Address) u8 {
    switch (addr) {
        0x0000...0xFFFF => {
            //std.log.info("bus: reading at address: {}", .{addr});
            return self.ram[addr];
        },
    }
}

pub fn write(self: *Bus, addr: Address, value: u8) void {
    switch (addr) {
        0x0000...0xFFFF => self.ram[addr] = value,
    }
}

//fn read_prg_rom(self: Bus, addr: Address) u8 {
//    var offset_addr: Address = addr - 0x8000;
//    if (self.cartridge.prg_rom.len == 0x4000 and offset_addr >= 0x4000) {
//        offset_addr %= 0x4000;
//    }
//    return self.Cartridge.prg_rom[offset_addr];
//}
