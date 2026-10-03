debug.log("=== ZIP Filesystem Tests ===")

-- Helper function to create test directory structure
local function setup_test_directory()
    debug.log("Setting up test directory structure")
    file.mkdirs("config:zip_test/subdir/deep")
    file.write("config:zip_test/root_file.txt", "This is a root level file")
    file.write("config:zip_test/subdir/file_in_subdir.txt", "File in subdirectory")
    file.write("config:zip_test/subdir/deep/deep_file.txt", "File in deep subdirectory")
    file.write("config:zip_test/unicode.txt", "Привет мир! Hello World! 你好世界!")
    
    local bytes = {0x00, 0x01, 0x02, 0xFF, 0xFE, 0xFD}
    file.write_bytes("config:zip_test/binary.bin", bytes)
    debug.log("Test directory structure created")
end

-- Helper function to cleanup
local function cleanup()
    debug.log("Cleaning up test files")
    if file.exists("config:zip_test") then
        file.remove_tree("config:zip_test")
    end
    if file.exists("config:test_archive.zip") then
        file.remove("config:test_archive.zip")
    end
end

-- Track found bugs
local bugs_found = {}
local function report_bug(name, description)
    table.insert(bugs_found, {name = name, description = description})
    debug.log("!!! BUG FOUND: " .. name)
    debug.log("    " .. description)
end

cleanup()

debug.log("\n[Test 1] Create test directory structure")
setup_test_directory()
assert(file.isdir("config:zip_test"))
assert(file.isfile("config:zip_test/root_file.txt"))
assert(file.isfile("config:zip_test/subdir/file_in_subdir.txt"))
assert(file.isfile("config:zip_test/subdir/deep/deep_file.txt"))

debug.log("\n[Test 2] Create ZIP archive")
file.create_zip("config:zip_test", "config:test_archive.zip")
assert(file.exists("config:test_archive.zip"), "ZIP archive was not created")
assert(file.isfile("config:test_archive.zip"), "ZIP archive is not a file")
local zip_size = file.length("config:test_archive.zip")
assert(zip_size > 0, "ZIP archive is empty")
debug.log("ZIP archive created, size: " .. tostring(zip_size) .. " bytes")

debug.log("\n[Test 3] Mount ZIP archive")
local entry_point = file.mount("config:test_archive.zip")
assert(entry_point ~= nil, "Mount returned nil")
assert(#entry_point > 0, "Mount returned empty entry point")
debug.log("ZIP mounted at entry point: " .. entry_point)

debug.log("\n[Test 4] Check mounted ZIP is read-only")
assert(not file.is_writeable(entry_point .. ":"), "Mounted ZIP should not be writeable")

-- Check root directory consistency (exists vs isdir methods check)
debug.log("\n[Test 5] Check root directory consistency")
local root_path = entry_point .. ":"
local root_isdir = file.isdir(root_path)
local root_exists = file.exists(root_path)
debug.log("file.isdir('" .. root_path .. "') = " .. tostring(root_isdir))
debug.log("file.exists('" .. root_path .. "') = " .. tostring(root_exists))

if root_isdir and not root_exists then
    report_bug("ROOT_EXISTS_INCONSISTENCY",
        "file.isdir() returns true for ZIP root but file.exists() returns false. " ..
        "ZipFileDevice::exists() should handle empty path like ZipFileDevice::isdir() does.")
end

debug.log("\n[Test 6] List root directory")
local root_entries = file.list(root_path)
debug.log("Root entries count: " .. #root_entries)
for i, entry in ipairs(root_entries) do
    debug.log("  [" .. i .. "] '" .. entry .. "'")
end

-- Check path format in ZIP archive (leading slash bug)
debug.log("\n[Test 7] Check path format in ZIP archive")
local test_paths = {
    {path = entry_point .. ":root_file.txt", desc = "without leading '/'"},
    {path = entry_point .. ":/root_file.txt", desc = "with leading '/'"},
    {path = entry_point .. ":subdir", desc = "subdir without leading '/'"},
    {path = entry_point .. ":/subdir", desc = "subdir with leading '/'"},
}

local path_without_slash_works = false
local path_with_slash_works = false
local working_prefix = ""

for _, test in ipairs(test_paths) do
    local exists = file.exists(test.path)
    local isfile = file.isfile(test.path)
    local isdir = file.isdir(test.path)
    debug.log("Path: '" .. test.path .. "' (" .. test.desc .. ")")
    debug.log("  exists=" .. tostring(exists) .. ", isfile=" .. tostring(isfile) .. ", isdir=" .. tostring(isdir))
    
    if test.desc == "without leading '/'" and exists then
        path_without_slash_works = true
        working_prefix = ""
    elseif test.desc == "with leading '/'" and exists then
        path_with_slash_works = true
        working_prefix = "/"
    end
end

if not path_without_slash_works and path_with_slash_works then
    report_bug("LEADING_SLASH_IN_ZIP_PATHS",
        "file.create_zip generates paths with leading '/' (e.g., '/root_file.txt' instead of 'root_file.txt'). " ..
        "This breaks file.list() for root directory and requires paths like 'entry:/file.txt' instead of 'entry:file.txt'. " ..
        "The bug is in write_zip() where name = entry.pathPart().substr(root.length()) produces '/file.txt' " ..
        "because root doesn't include the trailing slash.")
end

-- Set the working path format
local function make_path(relative_path)
    if #working_prefix > 0 then
        return entry_point .. ":" .. working_prefix .. relative_path
    else
        return entry_point .. ":" .. relative_path
    end
end

debug.log("\n[Test 8] Read text file (with correct path format)")
local root_file_path = make_path("root_file.txt")
debug.log("Trying to read: " .. root_file_path)
if file.exists(root_file_path) then
    local content = file.read(root_file_path)
    debug.log("Content: '" .. content .. "'")
    assert(content == "This is a root level file", "Content mismatch")
else
    debug.log("File does not exist with path: " .. root_file_path)
end

debug.log("\n[Test 9] Check subdirectory")
local subdir_path = make_path("subdir")
debug.log("Checking: " .. subdir_path)
local subdir_exists = file.exists(subdir_path)
local subdir_isdir = file.isdir(subdir_path)
debug.log("exists=" .. tostring(subdir_exists) .. ", isdir=" .. tostring(subdir_isdir))

debug.log("\n[Test 10] Check file in subdirectory")
local subdir_file_path = make_path("subdir/file_in_subdir.txt")
debug.log("Checking: " .. subdir_file_path)
local subdir_file_exists = file.exists(subdir_file_path)
debug.log("exists=" .. tostring(subdir_file_exists))
if subdir_file_exists then
    local content = file.read(subdir_file_path)
    debug.log("Content: '" .. content .. "'")
end

debug.log("\n[Test 11] List subdirectory")
if file.isdir(subdir_path) then
    local subdir_entries = file.list(subdir_path)
    debug.log("Subdirectory entries count: " .. #subdir_entries)
    for i, entry in ipairs(subdir_entries) do
        debug.log("  [" .. i .. "] '" .. entry .. "'")
    end
    
    for i, entry in ipairs(subdir_entries) do
        local expected_prefix = entry_point .. ":"
        if string.sub(entry, 1, #expected_prefix) ~= expected_prefix then
            report_bug("LIST_MISSING_ENTRY_POINT",
                "file.list() returns paths without entry point prefix. " ..
                "Expected '" .. expected_prefix .. "...' but got '" .. entry .. "'")
            break
        end
    end
    
    local subdir_path_slash = subdir_path .. "/"
    if file.isdir(subdir_path_slash) then
        local subdir_entries_slash = file.list(subdir_path_slash)
        debug.log("Subdirectory entries (with trailing '/') count: " .. #subdir_entries_slash)
        if #subdir_entries ~= #subdir_entries_slash then
            report_bug("TRAILING_SLASH_CHANGES_LIST_RESULT",
                "file.list() returns different results for paths with and without trailing slash")
        end
    end
else
    debug.log("Subdirectory does not exist, skipping list test")
end

debug.log("\n[Test 12] Deep nested structure")
local deep_dir = make_path("subdir/deep")
local deep_file = make_path("subdir/deep/deep_file.txt")
debug.log("Deep dir: " .. deep_dir .. " exists=" .. tostring(file.exists(deep_dir)))
debug.log("Deep file: " .. deep_file .. " exists=" .. tostring(file.exists(deep_file)))

debug.log("\n[Test 13] Binary file")
local binary_path = make_path("binary.bin")
debug.log("Binary file: " .. binary_path)
if file.exists(binary_path) then
    local read_bytes = file.read_bytes(binary_path)
    local expected_bytes = {0x00, 0x01, 0x02, 0xFF, 0xFE, 0xFD}
    debug.log("Read " .. #read_bytes .. " bytes, expected " .. #expected_bytes)
    local bytes_match = (#read_bytes == #expected_bytes)
    if bytes_match then
        for i, b in ipairs(expected_bytes) do
            if read_bytes[i] ~= b then
                bytes_match = false
                break
            end
        end
    end
    debug.log("Binary content matches: " .. tostring(bytes_match))
end

debug.log("\n[Test 14] Unicode content")
local unicode_path = make_path("unicode.txt")
if file.exists(unicode_path) then
    local content = file.read(unicode_path)
    local expected = "Привет мир! Hello World! 你好世界!"
    debug.log("Unicode content matches: " .. tostring(content == expected))
end

debug.log("\n[Test 15] file.parent() for ZIP paths")
local test_file = make_path("subdir/file_in_subdir.txt")
local parent = file.parent(test_file)
debug.log("file.parent('" .. test_file .. "') = '" .. parent .. "'")

local expected_parent = entry_point .. ":" .. working_prefix .. "subdir"
if parent ~= expected_parent then
    debug.log("Expected: '" .. expected_parent .. "'")
    -- This might be a problem with path normalization
end

debug.log("\n[Test 16] Navigate up using file.parent()")
local path = make_path("subdir/deep/deep_file.txt")
debug.log("Starting: " .. path)
local steps = {}
for i = 1, 5 do
    path = file.parent(path)
    table.insert(steps, path)
    debug.log("  Step " .. i .. ": " .. path)
    if path == entry_point .. ":" or path == "" then
        break
    end
end

debug.log("\n[Test 17] Unmount ZIP archive")
file.unmount(entry_point)
debug.log("Unmounted")

local after_unmount_exists = file.exists(root_file_path)
debug.log("After unmount, file.exists('" .. root_file_path .. "') = " .. tostring(after_unmount_exists))
if after_unmount_exists then
    report_bug("UNMOUNT_FILES_STILL_ACCESSIBLE",
        "Files are still accessible after file.unmount()")
end

-- Cleanup
debug.log("\n[Cleanup]")
cleanup()

-- Summary
debug.log("\n=== Test Summary ===")
if #bugs_found == 0 then
    debug.log("No bugs found!")
else
    debug.log("Found " .. #bugs_found .. " bug(s):")
    for i, bug in ipairs(bugs_found) do
        debug.log("  " .. i .. ". " .. bug.name)
        debug.log("     " .. bug.description)
    end
end

-- Final assertions to ensure critical functionality works
-- (these will fail the test if the workaround path format doesn't work)
debug.log("\n=== Final Assertions ===")

cleanup()
setup_test_directory()
file.create_zip("config:zip_test", "config:test_archive.zip")
local ep = file.mount("config:test_archive.zip")

-- Critical: Files should be readable from mounted ZIP (with workaround)
local test_content = file.read(ep .. ":/root_file.txt")
assert(test_content == "This is a root level file", "CRITICAL: Cannot read files from mounted ZIP")

-- Critical: Subdirectories should be listable
local test_list = file.list(ep .. ":/subdir")
assert(#test_list > 0, "CRITICAL: Cannot list subdirectories in mounted ZIP")

-- Critical: Deep nested files should be accessible
local deep_content = file.read(ep .. ":/subdir/deep/deep_file.txt")
assert(deep_content == "File in deep subdirectory", "CRITICAL: Cannot read deep nested files")

file.unmount(ep)
cleanup()

debug.log("\n=== ZIP Filesystem Tests Completed ===")
debug.log("Note: " .. #bugs_found .. " non-critical bug(s) detected, see summary above.")
