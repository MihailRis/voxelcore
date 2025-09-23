local this = {}

function this.equals(expected, fact)
    assert(fact == expected, string.format(
        "(fact == expected) assertion failed\n  Expected: %s\n  Fact:     %s",
        expected, fact
    ))
end

return this
