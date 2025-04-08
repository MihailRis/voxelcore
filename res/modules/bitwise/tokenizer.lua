local util = require "core:bitwise/util"

local operators = "><|&~^()"
local digits = "0123456789ABCDEF"
local startingDigits = "0123456789"
local idChars = "qwertyuiopasdfghjklzxcvbnmQWERTYUIOPASDFGHJKLZXCVBNM0123456789_"
local spaces = " \t"

local function esc(x)
   return (x:gsub('%%', '%%%%')
            :gsub('^%^', '%%^')
            :gsub('%$$', '%%$')
            :gsub('%(', '%%(')
            :gsub('%)', '%%)')
            :gsub('%.', '%%.')
            :gsub('%[', '%%[')
            :gsub('%]', '%%]')
            :gsub('%*', '%%*')
            :gsub('%+', '%%+')
            :gsub('%-', '%%-')
            :gsub('%?', '%%?'))
end

local decNum = "DECIMAL"
local hexNum = "HEXADECIMAL"
local binNum = "BINARY"

local rshift = "RIGHT SHIFT"
local lshift = "LEFT SHIFT"
local bor = "OR"
local band = "AND"
local bxor = "XOR"
local bnot = "NOT"
local id = "ID"
local openingBracket = "OPENING BRACKET"
local closingBracket = "CLOSING BRACKET"

local opToType =
{
	['^'] = bxor,
	['|'] = bor,
	['&'] = band,
	['('] = openingBracket,
	[')'] = closingBracket,
	['~'] = bnot
}

local typeToOp =
{
	[bxor] = '^',
	[bor] = '|',
	[band] = '&',
	[openingBracket] = '(',
	[closingBracket] = ')',
	[bnot] = '~'
}

local function contains(str, char)
	return string.find(str, esc(char)) ~= nil
end

local function checkDigitSystem(column, digit, base)
	local i = digits:find(digit)

	if not i or i > base then
		util.throw(column, "the digit '"..digit.."' does not belong to the "..base.."-based number system")
	end
end

local function isEndOfNumOrId(i, set, next, len)
	if i == len or not contains(set, next) then
		if i ~= len and  not contains(operators, next) and not contains(spaces, next) then
			util.throw(i + 1, "operator or space expected")
		else
			return true
		end
	else
		return false
	end
end

local function tokenize(str)
	local tokens = { }

	local buffer = ""
	local numType, readingNum, readingId
	local numSys

	local i = 1

	while i <= #str do
		local char = str:sub(i, i)
		local upChar = string.upper(char)
		local next = str:sub(i + 1, i + 1)

		if contains(operators, char) then
			local type

			if char == '>' or char == '<' then
				if next == char then
					type = char == '>' and rshift or lshift
					i = i + 1
				else
					util.throw(i, "invalid operator")
				end
			elseif char == '~' then
				if contains(idChars, next) then
					type = bnot
				else
					util.throw(i, "number expected")
				end
			else
				type = opToType[char]
			end

			table.insert(tokens, { column = i, type = type })
		elseif ((not readingNum and contains(startingDigits, char)) or (readingNum and contains(digits, upChar))) and not readingId then
			if not readingNum then
				readingNum = true

				if char == '0' and (next == 'x' or next == 'b') then
					numType = next == 'x' and hexNum or binNum
					numSys = next == 'x' and 16 or 2

					i = i + 2

					if not str:sub(i, i) then
						util.throw(i, "unexpected end")
					end

					char = str:sub(i, i)
					upChar = string.upper(char)
		 			next = str:sub(i + 1, i + 1)
				else
					numType = decNum
					numSys = 10
				end
			end

			if readingNum then
				if numType == hexNum then
					char, next = upChar, string.upper(next)
				end

				checkDigitSystem(i, char, numSys)

				buffer = buffer..char

				if isEndOfNumOrId(i, digits, next, #str) then
					readingNum = false
					table.insert(tokens, { column = i, type = numType, value = buffer })
					buffer = ""
				end
			end
		elseif contains(idChars, char) then
			if not readingId then
				readingId = true
			end

			if readingId then
				buffer = buffer..char

				if isEndOfNumOrId(i, idChars, next, #str) then
					readingId = false
					table.insert(tokens, { column = i, type = id, value = buffer })
					buffer = ""
				end
			end
		else
			local space
			
			for j = 1, #spaces do
				if spaces:sub(j, j) == char then
					space = true
					break
				end
			end

			if not space then
				util.throw(i, "undefined token: \""..char.."\"")
			end
		end

		i = i + 1
	end

	return tokens
end

local function printTokens(tokens)
	for _, token in ipairs(tokens) do
		local str = "{ "

		for k, v in pairs(token) do
			str = str..k..' = '..v..', '
		end

		print(str:sub(1, #str - 2).." }")
	end
end

return
{
	operators = operators,
	digits = digits,
	startingDigits = startingDigits,
	idChars = idChars,
	operatorTypes = { lshift, rshift, bnot, band, bxor, bor },
	numTypes = { decNum, hexNum, binNum },
	types = {
		decNum = decNum,
		hexNum = hexNum,
		binNum = binNum,
		rshift = rshift,
		lshift = lshift,
		bor = bor,
		band = band,
		bxor = bxor,
		bnot = bnot,
		id = id,
		openingBracket = openingBracket,
		closingBracket = closingBracket
	},
	opToType = opToType,
	typeToOp = typeToOp,
	get_tokens = tokenize,
	print_tokens = printTokens
}