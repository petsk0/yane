pub fn pageCross(a: u16, b: u16) bool {
    return a & 0xFF00 != add16(a, b) & 0xFF00;
}
pub fn add8(a: u8, b: u8) u8 {
    var result: u8 = undefined;
    _ = @addWithOverflow(u8, a, b, &result);
    return result;
}

pub fn add16(a: u16, b: u16) u16 {
    var result: u16 = undefined;
    _ = @addWithOverflow(u16, a, b, &result);
    return result;
}

pub fn sub16(a: u16, b: u16) u16 {
    var result: u16 = undefined;
    _ = @subWithOverflow(u16, a, b, &result);
    return result;
}

pub fn getBranchCycles(a: u16, b: u16) u8 {
    return if (pageCross(a, b)) 1 else 2;
}
