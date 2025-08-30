local util = require "core:bitwise/util"

local tokenizer = require "core:bitwise/tokenizer"

local types = tokenizer.types

local priorities =
{
	[types.bnot] = 4,
	[types.lshift] = 3,
	[types.rshift] = 3,
	[types.band] = 2,
	[types.bxor] = 1,
	[types.bor] = 0
}

local function parse(str)
	local stack = { }
	local output = { }

	for _, token in ipairs(tokenizer.get_tokens(str)) do
		if table.has(tokenizer.numTypes, token.type) or token.type == types.id then
			table.insert(output, token)
		elseif table.has(tokenizer.operatorTypes, token.type) then
			local firstOp = util.stack.top(stack)

			if firstOp and table.has(tokenizer.operatorTypes, firstOp.type) and priorities[firstOp.type] >= priorities[token.type] then
				table.insert(output, util.stack.pop(stack))
			end

			util.stack.push(stack, token)
		elseif token.type == types.openingBracket then
			util.stack.push(stack, token)
		elseif token.type == types.closingBracket then
			while true do
				local topToken = util.stack.top(stack)

				if not topToken then util.throw(token.column, "unexpected closing bracket")
				elseif topToken.type == types.openingBracket then break end

				table.insert(output, util.stack.pop(stack))
			end

			util.stack.pop(stack)
		end
	end

	while #stack > 0 do
		local token = util.stack.pop(stack)

		if token.type == types.openingBracket then
			util.throw(token.column, "unclosed bracket")
		end

		table.insert(output, token)
	end

	return output
end

return parse