local this = {}
local app = __vc_app

local function diff_canvas(expected, fact)
    this.equals(expected.width, fact.width, "image width")
    this.equals(expected.width, fact.width, "image height")

    local diff = Canvas({expected.width, expected.height})

    local mismatches = 0

    for y = 0, expected.height - 1 do
        for x = 0, expected.width - 1 do
            local ca = expected:at(x, y)
            local cb = fact:at(x, y)

            if ca == cb then
                diff:set(x, y, ca)
            else
                diff:set(x, y, 0xff0000ff)
                mismatches = mismatches + 1
            end
        end
    end

    return diff, mismatches
end

local TEST_ARTIFACTS_PATH = "export:test-results"

local function get_test_artifact_path(name)
    local path = TEST_ARTIFACTS_PATH
    if not file.isdir(path) then
        file.mkdirs(path)
    end
    return file.join(path, name)
end

function this.equals(expected, fact, prefix)
    local mt = getmetatable(expected)
    if mt == Canvas then
        local diff, mismatches = diff_canvas(expected, fact)
        if mismatches > 0 then
            prefix = prefix or app.script or base64.encode_urlsafe(random.bytes(6))
            local path_diff = get_test_artifact_path(
                string.format("%s.diff.png", prefix)
            )
            local path_fact = get_test_artifact_path(
                string.format("%s.fact.png", prefix)
            )
            local path_expected = get_test_artifact_path(
                string.format("%s.expected.png", prefix)
            )
            file.write_bytes(path_diff, diff:encode('png'))
            file.write_bytes(path_fact, fact:encode('png'))
            file.write_bytes(path_expected, expected:encode('png'))
            local total_pixels = expected.width * expected.height
            assert(mismatches == 0, string.format(
                "%s pixels of %s (%s%%) are different, diff saved as %s",
                mismatches,
                total_pixels,
                math.round(mismatches / total_pixels * 100, 2),
                string.escape(path_diff))
            )
        end
        return
    end
    if prefix then
        prefix = prefix .. " "
    else
        prefix = ""
    end
    assert(fact == expected, string.format(
        prefix .. "(fact == expected) assertion failed\n  Expected: %s\n  Fact:     %s",
        expected, fact
    ))
end

return this
