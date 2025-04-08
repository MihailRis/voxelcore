local compiler = require "core:bitwise/compiler"

local cache = { }

local cacheLength = 512

local function hashOfArgs(args)
	local str = ""

	for i = 1, #args do
		str = str..args[i]

		if i ~= #args then
			str = str..';'
		end
	end

	return str
end

local function execute(str, args, ...)
	local hasArgs = args and type(args) == 'table'

	local expArgs

	if hasArgs then
		expArgs = args
	else
		expArgs = nil
	end

	local argsHash = hasArgs and hashOfArgs(args) or ""

	local func

	if (#str + #argsHash) <= cacheLength then
		if cache[argsHash] then
			local fun = cache[argsHash][str]

			if fun then return fun end
		end

		local comp = compiler(str, expArgs)

		if not cache[argsHash] then cache[argsHash] = { } end

		cache[argsHash][str] = comp

		func = comp
	else
		func = compiler(str, expArgs)
	end

	if hasArgs or not args then
		return func(...)
	else
		return func(args, ...)
	end
end

return execute
