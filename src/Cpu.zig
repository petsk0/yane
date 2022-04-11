const std = @import("std");

const util = @import("util.zig");
const Bus = @import("Bus.zig");

const Cpu = @This();
const Address = Bus.Address;
const FREQUENCY: u32 = 1789773;

var bussy = Bus{};
bus: *Bus = &bussy,
interrupt: Interrupt = .none,
regs: Registers = .{},
clock: usize = 0,
mode: Mode = .immediate,
addr: Address = 0,

const Flag = enum(u8) {
    carry = 1 << 0,
    zero = 1 << 1,
    interrupt_disable = 1 << 2,
    decimal_mode = 1 << 3,
    break_mode = 1 << 4,
    unused = 1 << 5,
    overflow = 1 << 6,
    negative = 1 << 7,
};

const Loc = enum(u16) {
    stack = 0x0100,
    non_maskable_interrupt = 0xFFFA,
    reset = 0xFFFC,
    interrupt = 0xFFFE,
};

const Mode = enum {
    absolute,
    absolute_indirect,
    absolute_x,
    absolute_y,
    accumulator,
    immediate,
    implied,
    indirect_x,
    indirect_y,
    relative,
    zero_page,
    zero_page_x,
    zero_page_y,

    const Result = struct {
        addr: Address,
        extra_cycle: bool,
    };

    fn fetch(self: Mode, cpu: Cpu) Result {
        const temp_pc: u16 = cpu.regs.pc + 1;
        switch (self) {
            .absolute => {
                return .{
                    .addr = cpu.read16(temp_pc),
                    .extra_cycle = false,
                };
            },
            // NOTE: the NES cpu does net use this mode even though it is present
            // in "generic" 6502 cpus
            .absolute_indirect => {
                std.log.info("cannot use absolute indirect mode", .{});
                unreachable;
            },
            .absolute_x => {
                const temp_addr = cpu.read16(temp_pc);
                return .{
                    .addr = util.add16(temp_addr, cpu.regs.x),
                    .extra_cycle = if (util.pageCross(temp_addr, cpu.regs.x)) true else false,
                };
            },
            .absolute_y => {
                const temp_addr = cpu.read16(temp_pc);
                return .{
                    .addr = util.add16(temp_addr, cpu.regs.y),
                    .extra_cycle = if (util.pageCross(temp_addr, cpu.regs.y)) true else false,
                };
            },
            .accumulator => {
                return .{
                    .addr = 0,
                    .extra_cycle = false,
                };
            },
            //TODO: check if this is correct
            .immediate => {
                return .{
                    .addr = temp_pc,
                    .extra_cycle = false,
                };
            },
            .implied => {
                return .{
                    .addr = 0,
                    .extra_cycle = false,
                };
            },
            .indirect_x => {
                const temp = cpu.read8(temp_pc);
                const low_addr = util.add8(temp, cpu.regs.x);
                const high_addr = util.add8(low_addr, 1);
                return .{
                    .addr = (@as(Address, cpu.read8(high_addr)) << 8) | low_addr,
                    .extra_cycle = false,
                };
            },
            .indirect_y => {
                const low_addr = cpu.read8(temp_pc);
                const high_addr = cpu.read8(util.add8(low_addr, 1));
                const temp_addr = (@as(Address, cpu.read8(high_addr)) << 8) | low_addr;
                return .{
                    .addr = util.add16(temp_addr, cpu.regs.y),
                    .extra_cycle = if (util.pageCross(temp_addr, cpu.regs.y)) true else false,
                };
            },
            .relative => {
                const off = cpu.read8(temp_pc);
                return .{
                    .addr = if (off < 0x80) util.add16(off, util.add16(temp_pc, 1)) else util.add16(off, util.sub16(temp_pc, 0xFF)),
                    .extra_cycle = false,
                };
            },
            .zero_page => {
                return .{
                    .addr = cpu.read8(temp_pc),
                    .extra_cycle = false,
                };
            },
            .zero_page_x => {
                return .{
                    .addr = util.add8(cpu.read8(temp_pc), cpu.regs.x),
                    .extra_cycle = false,
                };
            },
            .zero_page_y => {
                return .{
                    .addr = util.add8(cpu.read8(temp_pc), cpu.regs.y),
                    .extra_cycle = false,
                };
            },
        }
    }
};

const Interrupt = enum {
    none,
    non_maskable,
    request,

    fn handle(self: Interrupt, cpu: *Cpu) Interrupt {
        switch (self) {
            .none => return .none,
            .non_maskable => {
                cpu.push16(cpu.regs.pc);
                cpu.push8(cpu.regs.status);
                cpu.regs.pc = cpu.read16(@enumToInt(Loc.non_maskable_interrupt));
                cpu.regs.setFlag(.interrupt_disable, true);
                cpu.clock += 7;
            },
            .request => {
                cpu.push16(cpu.regs.pc);
                cpu.push8(cpu.regs.status);
                cpu.regs.pc = cpu.read16(@enumToInt(Loc.interrupt));
                cpu.regs.setFlag(.interrupt_disable, true);
                cpu.clock += 7;
            },
        }
        return .none;
    }
};

const Registers = struct {
    acc: u8 = 0,
    x: u8 = 0,
    y: u8 = 0,
    pc: u16 = 0,
    sp: u8 = @enumToInt(Cpu.Loc.stack) - 3,
    status: u8 = 0x24,

    fn getFlag(self: Registers, flag: Flag) bool {
        return if (self.status & @enumToInt(flag) > 0) true else false;
    }

    fn setFlag(self: *Registers, flag: Flag, on: bool) void {
        if (on) self.status |= @enumToInt(flag) else self.status &= ~@enumToInt(flag);
    }

    fn bumpPc(self: *Registers, mode: Mode) void {
        switch (mode) {
            .accumulator, .implied => self.pc += 1,
            .immediate, .indirect_x, .indirect_y, .relative, .zero_page, .zero_page_x, .zero_page_y => self.pc += 2,
            .absolute, .absolute_x, .absolute_y, .absolute_indirect => self.pc += 3,
        }
    }

    // https://www.pagetable.com/?p=410
    fn reset(self: *Registers, cpu: Cpu) void {
        self.acc = 0;
        self.x = 0;
        self.y = 0;
        self.pc = cpu.read16(@enumToInt(Loc.reset));
        self.sp = @enumToInt(Loc.stack) - 3;
        self.status = 0x24;
    }
};

const OpcodeData = struct {
    const instruction_table: [256]*const fn (*Cpu) void = .{
        &Cpu.brk, &Cpu.ora, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.ora, &Cpu.asl, &Cpu.nai, &Cpu.php, &Cpu.ora, &Cpu.asl, &Cpu.nai, &Cpu.nai, &Cpu.ora, &Cpu.asl, &Cpu.nai,
        &Cpu.bpl, &Cpu.ora, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.ora, &Cpu.asl, &Cpu.nai, &Cpu.clc, &Cpu.ora, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.ora, &Cpu.asl, &Cpu.nai,
        &Cpu.jsr, &Cpu.And, &Cpu.nai, &Cpu.nai, &Cpu.bit, &Cpu.And, &Cpu.rol, &Cpu.nai, &Cpu.plp, &Cpu.And, &Cpu.rol, &Cpu.nai, &Cpu.bit, &Cpu.And, &Cpu.rol, &Cpu.nai,
        &Cpu.bmi, &Cpu.And, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.And, &Cpu.rol, &Cpu.nai, &Cpu.sec, &Cpu.And, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.And, &Cpu.rol, &Cpu.nai,
        &Cpu.rti, &Cpu.eor, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.eor, &Cpu.lsr, &Cpu.nai, &Cpu.pha, &Cpu.eor, &Cpu.lsr, &Cpu.nai, &Cpu.jmp, &Cpu.eor, &Cpu.lsr, &Cpu.nai,
        &Cpu.bvc, &Cpu.eor, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.eor, &Cpu.lsr, &Cpu.nai, &Cpu.cli, &Cpu.eor, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.eor, &Cpu.lsr, &Cpu.nai,
        &Cpu.rts, &Cpu.adc, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.adc, &Cpu.ror, &Cpu.nai, &Cpu.pla, &Cpu.adc, &Cpu.ror, &Cpu.nai, &Cpu.jmp, &Cpu.adc, &Cpu.ror, &Cpu.nai,
        &Cpu.bvs, &Cpu.adc, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.adc, &Cpu.ror, &Cpu.nai, &Cpu.sei, &Cpu.adc, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.adc, &Cpu.ror, &Cpu.nai,
        &Cpu.nai, &Cpu.sta, &Cpu.nai, &Cpu.nai, &Cpu.sty, &Cpu.sta, &Cpu.stx, &Cpu.nai, &Cpu.dey, &Cpu.nai, &Cpu.txa, &Cpu.nai, &Cpu.sty, &Cpu.sta, &Cpu.stx, &Cpu.nai,
        &Cpu.bcc, &Cpu.sta, &Cpu.nai, &Cpu.nai, &Cpu.sty, &Cpu.sta, &Cpu.stx, &Cpu.nai, &Cpu.tya, &Cpu.sta, &Cpu.txs, &Cpu.nai, &Cpu.nai, &Cpu.sta, &Cpu.nai, &Cpu.nai,
        &Cpu.ldy, &Cpu.lda, &Cpu.ldx, &Cpu.nai, &Cpu.ldy, &Cpu.lda, &Cpu.ldx, &Cpu.nai, &Cpu.tay, &Cpu.lda, &Cpu.tax, &Cpu.nai, &Cpu.ldy, &Cpu.lda, &Cpu.ldx, &Cpu.nai,
        &Cpu.bcs, &Cpu.lda, &Cpu.nai, &Cpu.nai, &Cpu.ldy, &Cpu.lda, &Cpu.ldx, &Cpu.nai, &Cpu.clv, &Cpu.lda, &Cpu.tsx, &Cpu.nai, &Cpu.ldy, &Cpu.lda, &Cpu.ldx, &Cpu.nai,
        &Cpu.cpy, &Cpu.cmp, &Cpu.nai, &Cpu.nai, &Cpu.cpy, &Cpu.cmp, &Cpu.dec, &Cpu.nai, &Cpu.iny, &Cpu.cmp, &Cpu.dex, &Cpu.nai, &Cpu.cpy, &Cpu.cmp, &Cpu.dec, &Cpu.nai,
        &Cpu.bne, &Cpu.cmp, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.cmp, &Cpu.dec, &Cpu.nai, &Cpu.cld, &Cpu.cmp, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.cmp, &Cpu.dec, &Cpu.nai,
        &Cpu.cpx, &Cpu.sbc, &Cpu.nai, &Cpu.nai, &Cpu.cpx, &Cpu.sbc, &Cpu.inc, &Cpu.nai, &Cpu.inx, &Cpu.sbc, &Cpu.nop, &Cpu.nai, &Cpu.cpx, &Cpu.sbc, &Cpu.inc, &Cpu.nai,
        &Cpu.beq, &Cpu.sbc, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.sbc, &Cpu.inc, &Cpu.nai, &Cpu.sed, &Cpu.sbc, &Cpu.nai, &Cpu.nai, &Cpu.nai, &Cpu.sbc, &Cpu.inc, &Cpu.nai,
    };

    const mode_table: [256]Mode = .{
        .implied,   .indirect_x, .implied,   .indirect_x, .zero_page,   .zero_page,   .zero_page,   .zero_page,   .implied, .immediate,  .accumulator, .immediate,  .absolute,          .absolute,   .absolute,   .absolute,
        .relative,  .indirect_y, .implied,   .indirect_y, .zero_page_x, .zero_page_x, .zero_page_x, .zero_page_x, .implied, .absolute_y, .implied,     .absolute_y, .absolute_x,        .absolute_x, .absolute_x, .absolute_x,
        .absolute,  .indirect_x, .implied,   .indirect_x, .zero_page,   .zero_page,   .zero_page,   .zero_page,   .implied, .immediate,  .accumulator, .immediate,  .absolute,          .absolute,   .absolute,   .absolute,
        .relative,  .indirect_y, .implied,   .indirect_y, .zero_page_x, .zero_page_x, .zero_page_x, .zero_page_x, .implied, .absolute_y, .implied,     .absolute_y, .absolute_x,        .absolute_x, .absolute_x, .absolute_x,
        .implied,   .indirect_x, .implied,   .indirect_x, .zero_page,   .zero_page,   .zero_page,   .zero_page,   .implied, .immediate,  .accumulator, .immediate,  .absolute,          .absolute,   .absolute,   .absolute,
        .relative,  .indirect_y, .implied,   .indirect_y, .zero_page_x, .zero_page_x, .zero_page_x, .zero_page_x, .implied, .absolute_y, .implied,     .absolute_y, .absolute_x,        .absolute_x, .absolute_x, .absolute_x,
        .implied,   .indirect_x, .implied,   .indirect_x, .zero_page,   .zero_page,   .zero_page,   .zero_page,   .implied, .immediate,  .accumulator, .immediate,  .absolute_indirect, .absolute,   .absolute,   .absolute,
        .relative,  .indirect_y, .implied,   .indirect_y, .zero_page_x, .zero_page_x, .zero_page_x, .zero_page_x, .implied, .absolute_y, .implied,     .absolute_y, .absolute_x,        .absolute_x, .absolute_x, .absolute_x,
        .immediate, .indirect_x, .immediate, .indirect_x, .zero_page,   .zero_page,   .zero_page,   .zero_page,   .implied, .immediate,  .implied,     .immediate,  .absolute,          .absolute,   .absolute,   .absolute,
        .relative,  .indirect_y, .implied,   .indirect_y, .zero_page_x, .zero_page_x, .zero_page_y, .zero_page_y, .implied, .absolute_y, .implied,     .absolute_y, .absolute_x,        .absolute_x, .absolute_y, .absolute_y,
        .immediate, .indirect_x, .immediate, .indirect_x, .zero_page,   .zero_page,   .zero_page,   .zero_page,   .implied, .immediate,  .implied,     .immediate,  .absolute,          .absolute,   .absolute,   .absolute,
        .relative,  .indirect_y, .implied,   .indirect_y, .zero_page_x, .zero_page_x, .zero_page_y, .zero_page_y, .implied, .absolute_y, .implied,     .absolute_y, .absolute_x,        .absolute_x, .absolute_y, .absolute_y,
        .immediate, .indirect_x, .immediate, .indirect_x, .zero_page,   .zero_page,   .zero_page,   .zero_page,   .implied, .immediate,  .implied,     .immediate,  .absolute,          .absolute,   .absolute,   .absolute,
        .relative,  .indirect_y, .implied,   .indirect_y, .zero_page_x, .zero_page_x, .zero_page_x, .zero_page_x, .implied, .absolute_y, .implied,     .absolute_y, .absolute_x,        .absolute_x, .absolute_x, .absolute_x,
        .immediate, .indirect_x, .immediate, .indirect_x, .zero_page,   .zero_page,   .zero_page,   .zero_page,   .implied, .immediate,  .implied,     .immediate,  .absolute,          .absolute,   .absolute,   .absolute,
        .relative,  .indirect_y, .implied,   .indirect_y, .zero_page_x, .zero_page_x, .zero_page_x, .zero_page_x, .implied, .absolute_y, .implied,     .absolute_y, .absolute_x,        .absolute_x, .absolute_x, .absolute_x,
    };

    const page_cross_table: [256]bool = .{
        false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
        true,  true,  false, false, false, false, false, false, false, true,  false, false, true,  true,  false, false,
        false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
        true,  true,  false, false, false, false, false, false, false, true,  false, false, true,  true,  false, false,
        false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
        true,  true,  false, false, false, false, false, false, false, true,  false, false, true,  true,  false, false,
        false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
        true,  true,  false, false, false, false, false, false, false, true,  false, false, true,  true,  false, false,
        false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
        true,  false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
        false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
        true,  true,  false, true,  false, false, false, false, false, true,  false, true,  true,  true,  true,  true,
        false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
        true,  true,  false, false, false, false, false, false, false, true,  false, false, true,  true,  false, false,
        false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false,
        true,  true,  false, false, false, false, false, false, false, true,  false, false, true,  true,  false, false,
    };

    // zig fmt: off
    const cycle_table: [256]u8 = .{
    //  0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
        7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6, // 0
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, // 1
        6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6, // 2
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, // 3
        6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6, // 4
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, // 5
        6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6, // 6
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, // 7
        2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4, // 8
        2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5, // 9
        2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4, // A
        2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4, // B
        2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6, // C
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, // D
        2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6, // E
        2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7, // F
    };
};

fn bumpBranchCycles(self: *Cpu, addr: Address) void {
    if (util.pageCross(addr, self.regs.pc)) self.clock += 1 else self.clock += 2;
}

fn setCompareFlags(self: *Cpu, a: u8, b: u8) void {
    const a16 = @as(i16, a);
    const b16 = @as(i16, b);
    self.regs.setFlag(.carry, a16 >= b16);
    self.regs.setFlag(.zero, a16 - b16 == 0);
    self.regs.setFlag(.negative, (a16 - b16) & 0x80 != 0);
}

pub fn read8(self: Cpu, addr: Address) u8 {
    return self.bus.read(addr);
}

pub fn read16(self: Cpu, addr: Address) u16 {
    const low_byte: u16 = self.read8(addr);
    const high_byte: u16 = self.read8(addr + 1);
    return high_byte << 8 | low_byte;
}

pub fn write8(self: Cpu, addr: Address, value: u8) void {
    self.bus.write(addr, value);
}

pub fn write16(self: Cpu, addr: Address, value: u16) void {
    const low_byte = @intCast(u8, value & 0xFF);
    const high_byte = @intCast(u8, value >> 8);
    self.bus.write(addr, low_byte);
    self.bus.write(addr + 1, high_byte);
}

fn push8(self: *Cpu, value: u8) void {
    self.bus.write(@enumToInt(Loc.stack) + self.regs.sp, value);
    self.regs.sp -= 1;
}

fn push16(self: *Cpu, value: u16) void {
    self.bus.write(@enumToInt(Loc.stack) + self.regs.sp, @truncate(u8, value >> 8));
    self.regs.sp -= 1;
    self.bus.write(@enumToInt(Loc.stack) + self.regs.sp, @truncate(u8, value));
    self.regs.sp -= 1;
}

fn pop8(self: *Cpu) u8 {
    self.regs.sp += 1;
    return self.bus.read(@enumToInt(Loc.stack) + self.regs.sp);
}

fn pop16(self: *Cpu) u16 {
    const low_byte: u16 = self.bus.read(@enumToInt(Loc.stack) + self.regs.sp + 1);
    const high_byte: u16 = self.bus.read(@enumToInt(Loc.stack) + self.regs.sp + 2);
    self.regs.sp += 2;
    return high_byte << 8 | low_byte;
}

pub fn new(bus: *Bus) *Cpu {
    return &.{
        .bus = bus,
    };
}

pub fn reset(self: *Cpu) void {
    self.regs.reset(self.*);
}

// TODO: this function is temporary, handle loading via bus
pub fn load(self: Cpu, program: [309]u8) void {
    for (program) |val, idx| {
        self.bus.ram[idx + 0x0600] = val;
    }
    self.write16(@enumToInt(Loc.reset), 0x0600);
}

pub fn step(self: *Cpu) void {
    self.interrupt = self.interrupt.handle(self);
    const opcode = self.read8(self.regs.pc);
    std.log.info("opcode number: {}", .{opcode});
    self.mode = OpcodeData.mode_table[opcode];
    const mode_result = self.mode.fetch(self.*);
    self.addr = mode_result.addr;
    self.regs.bumpPc(self.mode);
    self.clock += OpcodeData.cycle_table[opcode];
    if (mode_result.extra_cycle and OpcodeData.page_cross_table[opcode]) self.clock += 1;
    OpcodeData.instruction_table[opcode].*(self);
}

//====================
//    INSTRUCTIONS
//====================

// add with carry
fn adc(self: *Cpu) void {
    const value = self.read8(self.addr);
    const carry = @boolToInt(self.regs.getFlag(.carry));
    const acc = self.regs.acc;
    const overflow1 = @addWithOverflow(u8, self.regs.acc, value, &self.regs.acc);
    const overflow2 = @addWithOverflow(u8, self.regs.acc, carry, &self.regs.acc);
    self.regs.setFlag(.carry, overflow1 or overflow2);
    self.regs.setFlag(.zero, self.regs.acc == 0);
    self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
    self.regs.setFlag(.overflow, ~(acc ^ value) & (acc ^ self.regs.acc) & 0x80 != 0);
}

// logical and
fn And(self: *Cpu) void {
    self.regs.acc = self.regs.acc & self.read8(self.addr);
    self.regs.setFlag(.zero, self.regs.acc == 0);
    self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
}

// arthmetic shift left
fn asl(self: *Cpu) void {
    var value = if (self.mode == .accumulator) self.regs.acc else self.read8(self.addr);
    self.regs.setFlag(.carry, ((value >> 7) & 1) == 1);
    value = value << 1;
    if (self.mode == .accumulator) self.regs.acc = value else self.write8(self.addr, value);
    self.regs.setFlag(.zero, value == 0);
    self.regs.setFlag(.negative, value & 0x80 != 0);
}

// bit test
fn bit(self: *Cpu) void {
    const value = self.read8(self.addr);
    self.regs.setFlag(.overflow, ((value >> 6) & 1) == 1);
    self.regs.setFlag(.zero, value & self.regs.acc == 0);
    self.regs.setFlag(.negative, value & 0x80 != 0);
}

// branch if carry clear
fn bcc(self: *Cpu) void {
    if (!self.regs.getFlag(.carry)) {
        self.bumpBranchCycles(self.addr);
        self.regs.pc = self.addr;
    }
}

// branch if carry set
fn bcs(self: *Cpu) void {
    if (self.regs.getFlag(.carry)) {
        self.bumpBranchCycles(self.addr);
        self.regs.pc = self.addr;
    }
}

// branch if equal
fn beq(self: *Cpu) void {
    if (self.regs.getFlag(.zero)) {
        self.bumpBranchCycles(self.addr);
        self.regs.pc = self.addr;
    }
}

// branch if minus
fn bmi(self: *Cpu) void {
    if (self.regs.getFlag(.negative)) {
        self.bumpBranchCycles(self.addr);
        self.regs.pc = self.addr;
    }
}

// branch if not equal
fn bne(self: *Cpu) void {
    if (self.regs.getFlag(.zero)) {
        self.bumpBranchCycles(self.addr);
        self.regs.pc = self.addr;
    }
}

// branch if overflow clear
fn bvc(self: *Cpu) void {
    if (!self.regs.getFlag(.overflow)) {
        self.bumpBranchCycles(self.addr);
        self.regs.pc = self.addr;
    }
}

//branch if overflow set
fn bvs(self: *Cpu) void {
    if (self.regs.getFlag(.overflow)) {
        self.bumpBranchCycles(self.addr);
        self.regs.pc = self.addr;
    }
}

// branch if plus
fn bpl(self: *Cpu) void {
    if (!self.regs.getFlag(.negative)) {
        self.bumpBranchCycles(self.addr);
        self.regs.pc = self.addr;
    }
}

// force interrupt
fn brk(self: *Cpu) void {
    // TODO: is brk in fact a two byte opcode??
    // for now I assume it is and therefore increment pc by one
    // self.regs.pc += 1;

    // self.push16(self.regs.pc);
    // self.push8(self.regs.status | 0x10);
    // self.regs.setFlag(.interrupt_disable, true);
    // self.regs.pc = self.read16(@enumToInt(Loc.interrupt));
    _ = self;
    return;
}

// clear carry flag
fn clc(self: *Cpu) void {
    self.regs.status &= ~@enumToInt(Flag.carry);
}

// clear decimal mode flag
fn cld(self: *Cpu) void {
    self.regs.status &= ~@enumToInt(Flag.decimal_mode);
}

// clear interrupt disable flag
fn cli(self: *Cpu) void {
    self.regs.status &= ~@enumToInt(Flag.interrupt_disable);
}

// clear overflow flag
fn clv(self: *Cpu) void {
    self.regs.status &= ~@enumToInt(Flag.overflow);
}

// compare
fn cmp(self: *Cpu) void {
    const value = self.read8(self.addr);
    self.setCompareFlags(self.regs.acc, value);
}

// compare register x
fn cpx(self: *Cpu) void {
    const value = self.read8(self.addr);
    self.setCompareFlags(self.regs.x, value);
}

// compare register y
fn cpy(self: *Cpu) void {
    const value = self.read8(self.addr);
    self.setCompareFlags(self.regs.y, value);
}

// decrement memory
fn dec(self: *Cpu) void {
    const value = self.read8(self.addr) -% 1;
    self.write8(self.addr, value);
    self.regs.setFlag(.zero, value == 0);
    self.regs.setFlag(.negative, value & 0x80 != 0);
}

// decrement x
fn dex(self: *Cpu) void {
    self.regs.x -%= 1;
    self.regs.setFlag(.zero, self.regs.x == 0);
    self.regs.setFlag(.negative, self.regs.x & 0x80 != 0);
}

// decrement y
fn dey(self: *Cpu) void {
    self.regs.y -%= 1;
    self.regs.setFlag(.zero, self.regs.y == 0);
    self.regs.setFlag(.negative, self.regs.y & 0x80 != 0);
}

// logical exclusive or
fn eor(self: *Cpu) void {
    self.regs.acc ^= self.read8(self.addr);
    self.regs.setFlag(.zero, self.regs.acc == 0);
    self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
}

// increment memory
fn inc(self: *Cpu) void {
    const value = self.read8(self.addr) +% 1;
    self.write8(self.addr, value);
    self.regs.setFlag(.zero, value == 0);
    self.regs.setFlag(.negative, value & 0x80 != 0);
}

// increment x
fn inx(self: *Cpu) void {
    self.regs.x +%= 1;
    self.regs.setFlag(.zero, self.regs.x == 0);
    self.regs.setFlag(.negative, self.regs.x & 0x80 != 0);
}

// increment y
fn iny(self: *Cpu) void {
    self.regs.y +%= 1;
    self.regs.setFlag(.zero, self.regs.y == 0);
    self.regs.setFlag(.negative, self.regs.y & 0x80 != 0);
}

// jump
fn jmp(self: *Cpu) void {
    self.regs.pc = self.addr;
}

// jump to subroutine
fn jsr(self: *Cpu) void {
    self.push16(self.regs.pc - 1);
    self.regs.pc = self.addr;
}

// load accumulator
fn lda(self: *Cpu) void {
    self.regs.acc = self.read8(self.addr);
    self.regs.setFlag(.zero, self.regs.acc == 0);
    self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
}

// load x register
fn ldx(self: *Cpu) void {
    self.regs.x = self.read8(self.addr);
    self.regs.setFlag(.zero, self.regs.x == 0);
    self.regs.setFlag(.negative, self.regs.x & 0x80 != 0);
}

// load y register
fn ldy(self: *Cpu) void {
    self.regs.y = self.read8(self.addr);
    self.regs.setFlag(.zero, self.regs.y == 0);
    self.regs.setFlag(.negative, self.regs.y & 0x80 != 0);
}

// logical shift right
fn lsr(self: *Cpu) void {
    if (self.mode == .accumulator) {
        self.regs.setFlag(.carry, self.regs.acc & 1 == 1);
        self.regs.acc >>= 1;
        self.regs.setFlag(.zero, self.regs.acc == 0);
        self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
    } else {
        var value = self.read8(self.addr);
        self.regs.setFlag(.carry, value & 1 == 1);
        value >>= 1;
        self.write8(self.addr, value);
        self.regs.setFlag(.zero, value == 0);
        self.regs.setFlag(.negative, value & 0x80 != 0);
    }
}

// no operation
fn nop(self: *Cpu) void {
    _ = self;
}

// logical inclusive or
fn ora(self: *Cpu) void {
    self.regs.acc ^= self.read8(self.addr);
    self.regs.setFlag(.zero, self.regs.acc == 0);
    self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
}

// push accumulator
fn pha(self: *Cpu) void {
    self.push8(self.regs.acc);
}

// NOTE: This instruction pushes processor status (1 byte) onto the stack.
// Processor status consists of 6 physical flags (1 bit each),
// one of two extra bits pushed onto the stack is the B flag
// (https://wiki.nesdev.com/w/index.php/Status_flags#The_B_flag)
// which is set and the other bit is unused.
//
// push processor status
fn php(self: *Cpu) void {
    self.push8(self.regs.status | 0x10);
}

// pull accumulator
fn pla(self: *Cpu) void {
    self.regs.acc = self.pop8();
    self.regs.setFlag(.zero, self.regs.acc == 0);
    self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
}

// pull processor status
fn plp(self: *Cpu) void {
    self.regs.status = self.pop8();
    self.regs.setFlag(.break_mode, false);
    self.regs.setFlag(.unused, true);
}

// rotate left
fn rol(self: *Cpu) void {
    const carry = @boolToInt(self.regs.getFlag(.carry));
    if (self.mode == .accumulator) {
        self.regs.setFlag(.carry, (self.regs.acc & 0x80) == 1);
        self.regs.acc = (self.regs.acc << 1) | carry;
    } else {
        var value = self.read8(self.addr);
        self.regs.setFlag(.carry, (value & 0x80) == 1);
        value = (value << 1) | carry;
        self.write8(self.addr, value);
    }
}

// rotate right
fn ror(self: *Cpu) void {
    const carry = @as(u8, @boolToInt(self.regs.getFlag(.carry)));
    if (self.mode == .accumulator) {
        self.regs.setFlag(.carry, (self.regs.acc & 0x01) == 1);
        self.regs.acc = (self.regs.acc >> 1) | (carry << 7);
    } else {
        var value = self.read8(self.addr);
        self.regs.setFlag(.carry, (value & 0x01) == 1);
        value = (value >> 1) | (carry << 7);
        self.write8(self.addr, value);
    }
}

// NOTE: this instruction sets processor status from the stack
// except the break flag
//
// return from interrupt
fn rti(self: *Cpu) void {
    self.regs.status = self.pop8();
    self.regs.setFlag(.break_mode, false);
    self.regs.setFlag(.unused, true);
    self.regs.pc = self.pop16();
}

// TODO: handle overflow on addition
//
// return from subroutine
fn rts(self: *Cpu) void {
    self.regs.pc = self.pop16() + 1;
}

// subtract with carry
fn sbc(self: *Cpu) void {
    const value = self.read8(self.addr) ^ 0xFF;
    const carry = @as(u8, @boolToInt(self.regs.getFlag(.carry)));
    const acc = self.regs.acc;
    const overflow1 = @addWithOverflow(u8, self.regs.acc, value, &self.regs.acc);
    const overflow2 = @addWithOverflow(u8, self.regs.acc, carry, &self.regs.acc);
    self.regs.setFlag(.carry, overflow1 or overflow2);
    self.regs.setFlag(.zero, self.regs.acc == 0);
    self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
    self.regs.setFlag(.overflow, (acc ^ value) & (acc ^ self.regs.acc) & 0x80 != 0);
}

// set carry flag
fn sec(self: *Cpu) void {
    self.regs.setFlag(.carry, true);
}

// set decimal flag
fn sed(self: *Cpu) void {
    self.regs.setFlag(.decimal_mode, true);
}

// set interrupt disable flag
fn sei(self: *Cpu) void {
    self.regs.setFlag(.interrupt_disable, true);
}

// store accumulator in memory
fn sta(self: *Cpu) void {
    self.write8(self.addr, self.regs.acc);
}

// store register x in memory
fn stx(self: *Cpu) void {
    self.write8(self.addr, self.regs.x);
}

// store register y in memory
fn sty(self: *Cpu) void {
    self.write8(self.addr, self.regs.y);
}

// transfer accumulator to register x
fn tax(self: *Cpu) void {
    self.regs.x = self.regs.acc;
    self.regs.setFlag(.zero, self.regs.x == 0);
    self.regs.setFlag(.negative, self.regs.x & 0x80 != 0);
}

// transfer accumulator to register y
fn tay(self: *Cpu) void {
    self.regs.y = self.regs.acc;
    self.regs.setFlag(.zero, self.regs.y == 0);
    self.regs.setFlag(.negative, self.regs.y & 0x80 != 0);
}

// trasfer stack pointer to register x
fn tsx(self: *Cpu) void {
    self.regs.x = self.regs.sp;
    self.regs.setFlag(.zero, self.regs.x == 0);
    self.regs.setFlag(.negative, self.regs.x & 0x80 != 0);
}

// transfer register x to accumulator
fn txa(self: *Cpu) void {
    self.regs.acc = self.regs.x;
    self.regs.setFlag(.zero, self.regs.acc == 0);
    self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
}

// NOTE: this instruction doesn't set the zero and negative flags
//
// transfer register x to stack pointer
fn txs(self: *Cpu) void {
    self.regs.sp = self.regs.x;
}

// transfer register y to accumulator
fn tya(self: *Cpu) void {
    self.regs.acc = self.regs.y;
    self.regs.setFlag(.zero, self.regs.acc == 0);
    self.regs.setFlag(.negative, self.regs.acc & 0x80 != 0);
}

// NOTE: placeholder for unofficial opcodes
//
// not an instruction
fn nai(self: *Cpu) void {
    // TODO: handle this more gracefully
    _ = self;
    std.log.info("error: not an instruction", .{});
    unreachable;
}

// TODO: unofficial opcodes

//====================

// returns a slice of bytes containing the current state of cpu in nestest format
// https://www.qmtpro.com/~nes/misc/nestest.log
fn dumpState() []u8 {
    std.log.info("error: dumpState not yet implemented", .{});
    unreachable;
}
