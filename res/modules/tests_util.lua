local util = {}

function util.create_demo_world(generator)
    app.config_packs({"base"})
    app.new_world("demo", "2019", generator or "core:default")
end

function util.check_vec3(expected, fact, name, delta)
    delta = delta or 1e-4
    for i=1,3 do
        assert(math.abs(fact[i] - expected[i]) < delta,
            name.."["..i.."] expected: "..expected[i]..", fact: "..fact[i])
    end
end

function util.check_float(expected, fact, name, delta)
    delta = delta or 1e-4
    local actual_delta = math.abs(fact - expected)
    assert(actual_delta < delta,
        name.." expected: "..expected..", fact: "..fact..
        " (delta: "..actual_delta.." > "..delta..")")
end

function util.check_int(expected, fact, name)
    assert(fact == expected,
        name.." expected: "..expected..", fact: "..fact)
end

return util
