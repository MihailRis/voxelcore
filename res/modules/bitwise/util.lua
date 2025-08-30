local util = { }

util.stack = { }

function util.throw(column, msg)
	error("column "..column..": "..msg)
end

function util.stack.push(stack, value)
	table.insert(stack, value)
end

function util.stack.pop(stack)
	return table.remove(stack, #stack)
end

function util.stack.top(stack)
	return stack[#stack]
end

return util