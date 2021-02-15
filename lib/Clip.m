function y = Clip(x, lowerValue, upperValue) % x is the input
    y = min(max(x,lowerValue),upperValue);
end