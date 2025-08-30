local util = require "core:bitwise/util"

local tokenizer = require "core:bitwise/tokenizer"
local parser = require "core:bitwise/parser"

local types = tokenizer.types

local operatorToFunctionName =
{
	[types.lshift] = "bit.lshift",
	[types.rshift] = "bit.rshift",
	[types.bnot] = "bit.bnot",
	[types.band] = "bit.band",
	[types.bxor] = "bit.bxor",
	[types.bor] = "bit.bor"
}

local function compile(str, args, asFunction)
	if asFunction == nil then
		asFunction = true
	end

	local tokens = parser(str)

	local stack = { }

	local ids = { }

	for _, token in ipairs(tokens) do
		if table.has(tokenizer.numTypes, token.type) or token.type == types.id then
			if token.type == tokenizer.types.binNum then
				util.stack.push(stack, tonumber(token.value, 2))
			elseif token.type == tokenizer.types.hexNum then
				util.stack.push(stack, "0x"..token.value)
			else
				util.stack.push(stack, token.value)

				if token.type == types.id then
					local add = true

					for i = 1, #ids do
						if ids[i].value == token.value then
							add = false
							break
						end
					end

					if add then table.insert(ids, token) end
				end
			end
		else
			local fun = operatorToFunctionName[token.type]

			local a = util.stack.pop(stack)
			local b = token.type ~= types.bnot and util.stack.pop(stack)

			if not b then
				util.stack.push(stack, fun..'('..a..')')
			else
				util.stack.push(stack, fun..'('..b..','..a..')')
			end
		end
	end

	local strArgs = ""

	if args then
		for _, id in ipairs(ids) do
			if not table.has(args, id.value) then
				util.throw(id.column, "undefined identifier")
			end
		end
	else
		args = { }

		table.sort(ids, function(a, b) return a.value:upper() < b.value:upper() end)

		for _, id in ipairs(ids) do
			table.insert(args, id.value)
		end
	end

	for i = 1, #args do
		local str = args[i]

		for j = 1, #str do
			local char = str:sub(j,j)

			if
				not string.find(tokenizer.idChars, char) or
				(j == 1 and string.find(tokenizer.startingDigits, char))
			then error("invalid argument name ("..i..")") end
		end

		strArgs = strArgs..str

		if i ~= #args then
			strArgs = strArgs..','
		end
	end

	local code = stack[1]

	code = "function("..strArgs..") return "..code.." end"

	if asFunction then
		return load("return "..code)()
	else
		return code
	end
end

return compile