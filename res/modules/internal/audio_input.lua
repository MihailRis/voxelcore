local _base64_encode_urlsafe = base64.encode_urlsafe
local _random_bytes = random.bytes
local core_token = _base64_encode_urlsafe(_random_bytes(18))

local audio_input_tokens_store = {[core_token] = "core"}
audio.input = {}

local _gui_confirm = gui.confirm
local _debug_pack_by_frame = debug.get_pack_by_frame
local _audio_fetch_input = audio.__fetch_input
audio.__fetch_input = nil
local MAX_FETCH = 44100 * 4
local MAX_AMPLITUDE = 32768

local total_fetch = Bytearray()
local max_amplitude = 0.0

function audio.__reset_fetch_buffer()
    total_fetch:clear()
    max_amplitude = 0.0
end

function audio.input.get_max_amplitude()
    return max_amplitude / MAX_AMPLITUDE
end

function audio.input.fetch(token, size)
    size = size or MAX_FETCH
    if audio_input_tokens_store[token] then
        if #total_fetch >= size then
            return total_fetch:slice(1, size)
        end
        local fetched = _audio_fetch_input(size - #total_fetch)
        if not fetched then
            return
        end
        for i, sample in ipairs(I16view(fetched)) do
            max_amplitude = math.max(math.abs(sample))
        end
        total_fetch:append(fetched)
        return total_fetch:slice()
    end
    error("access denied")
end

local GRANT_PERMISSION_MSG = "Grant '%{0}' pack audio recording permission?"

function audio.input.request_open(callback)
    local token = _base64_encode_urlsafe(_random_bytes(18))
    local caller = _debug_pack_by_frame(1)
    _gui_confirm(gui.str(GRANT_PERMISSION_MSG):gsub("%%{0}", caller), function()
        audio_input_tokens_store[token] = caller
        callback(token)
        menu:reset()
    end)
end

function audio.input.__get_core_token()
    local caller = _debug_pack_by_frame(1)
    if caller == "core" then
        return core_token
    end
end
