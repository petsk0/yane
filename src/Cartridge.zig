const std = @import("std.zig");
const mem = std.mem;

const Cartridge = @This();
const INES_HEADER_MAGIC: [4]u8 = .{ 0x4E, 0x45, 0x53, 0x1A };
const HEADER_SIZE: usize = 0x10;
const TRAINER_SIZE: usize = 0x200;
const PRG_ROM_PAGE_SIZE: usize = 0x4000;
const CHR_ROM_PAGE_SIZE: usize = 0x2000;

prg_rom: []u8,
chr_rom: []u8,
mapper_id: u8,
//mapper: Mapper,
mirror: Mirror,

const Mirror = enum(u8) {
    horizontal,
    vertical,
    four_screen,
};

const Error = error{
    WrongFormat,
    Ines2NotSupported,
};

//const Mapper = struct {
//    id: u8,
//};

// NOTE: caller frees the memory
pub fn initFromInes(raw_ines: *[]u8, allocator: *mem.Allocator) Error!Cartridge {
    if (!mem.eql(u8, INES_HEADER_MAGIC[0..], raw_ines[0..4])) {
        return error.WrongFormat;
    }
    if (raw_ines[7] & 0x0F != 0) {
        return error.Ines2NotSupported;
    }
    const has_trainer = raw_ines[6] & 0x04 != 0;
    const mirror = if (raw_ines[6] & 0x0F != 0) .four_screen else raw_ines[6] & 0x01;

    const prg_rom_size = raw_ines[4] * PRG_ROM_PAGE_SIZE;
    const chr_rom_size = if (raw_ines[5] == 0) CHR_ROM_PAGE_SIZE else raw_ines[5] * CHR_ROM_PAGE_SIZE;

    var prg_rom = allocator.alloc(u8, prg_rom_size);
    var chr_rom = allocator.alloc(u8, chr_rom_size);

    const prg_rom_begin = HEADER_SIZE + if (has_trainer) TRAINER_SIZE else 0;
    const chr_rom_begin = prg_rom_begin + prg_rom_size;

    const prg_rom_end = prg_rom_begin + prg_rom_size;
    const chr_rom_end = chr_rom_begin + chr_rom_size;

    mem.copy(u8, prg_rom, raw_ines[prg_rom_begin..prg_rom_end]);
    mem.copy(u8, chr_rom, raw_ines[chr_rom_begin..chr_rom_end]);

    return .{
        .prg_rom = prg_rom,
        .chr_rom = chr_rom,
        .mapper_id = (raw_ines[7] & 0xF0) | (raw_ines[6] >> 4),
        .mirror = mirror,
    };
}
