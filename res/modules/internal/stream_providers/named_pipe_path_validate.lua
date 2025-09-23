local forbiddenPaths = {
    "/..\\", "\\../",
    "/../", "\\..\\"
}

return function(path)
    local corrected = true

    if path:starts_with("../") or path:starts_with("..\\") then
        corrected = false
    else
        for _, forbiddenPath in ipairs(forbiddenPaths) do
            if path:find(forbiddenPath) then
                corrected = false
                break
            end
        end    
    end

    if not corrected then error "special path \"../\" is not allowed in path to named pipe" end
end