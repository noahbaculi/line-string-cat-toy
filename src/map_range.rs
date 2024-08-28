#[derive(Debug, PartialEq)]
pub enum MapRangeError {
    Underflow,
    Overflow,
    DivisionByZero,
}

pub fn map_range<T>(
    in_value: T,
    in_min: T,
    in_max: T,
    out_min: T,
    out_max: T,
) -> Result<T, MapRangeError>
where
    T: Copy
        + core::ops::Mul<Output = T>
        + core::ops::Add<Output = T>
        + core::ops::Div<Output = T>
        + core::ops::Sub<Output = T>
        + num_traits::CheckedSub
        + num_traits::CheckedAdd
        + num_traits::CheckedMul
        + num_traits::CheckedDiv,
{
    // ((in_value - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min
    let in_range = in_max
        .checked_sub(&in_min)
        .ok_or(MapRangeError::Underflow)?;
    let out_range = out_max
        .checked_sub(&out_min)
        .ok_or(MapRangeError::Underflow)?;
    let scaled_value = in_value
        .checked_sub(&in_min)
        .ok_or(MapRangeError::Underflow)?
        .checked_mul(&out_range)
        .ok_or(MapRangeError::Overflow)?;
    let mapped_value = scaled_value
        .checked_div(&in_range)
        .ok_or(MapRangeError::DivisionByZero)?
        .checked_add(&out_min)
        .ok_or(MapRangeError::Overflow)?;

    Ok(mapped_value)
}
