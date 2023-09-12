function output = simpleScale(input, in_lo, in_hi, out_lo, out_hi)
output = (input - in_lo) / (in_hi - in_lo) * (out_hi-out_lo) + out_lo;
end

