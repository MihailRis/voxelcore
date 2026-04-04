function pack.is_installed(packid)
    return file.isfile(packid..":package.json")
end

function pack.data_file(packid, name)
    file.mkdirs("world:data/"..packid)
    return "world:data/"..packid.."/"..name
end

function pack.shared_file(packid, name)
    file.mkdirs("config:"..packid)
    return "config:"..packid.."/"..name
end
