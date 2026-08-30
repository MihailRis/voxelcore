local M = {}

-- Оценивает схожесть строк и возвращает совпадение (>= 0) и если переданы
-- параметры color_matched и color_normal, возвращает также исходную строку,
-- в которой совпадающие части обрамлены в color_matched и color_normal
---@param sample str где ищем совпадения
---@param pattern str какие совпадения ищем
---@param color_matched? str
---@param color_normal? str
---@return int match
---@return str? colored_sample
function M.fuzzy_score(sample, pattern, color_matched, color_normal)
    if pattern == "" then return 0 end

    local do_coloring = color_matched and color_normal
    local colored_sample
    if do_coloring then
        colored_sample = ""
    end

    local sample_lower = string.lower(sample)
    local pattern_lower = string.lower(pattern)

    local sample_len = #sample_lower
    local pattern_len = #pattern_lower

    if pattern_len > sample_len then return 0 end

    local pattern_idx = 1
    local consecutive = 0
    local max_consecutive = 0
    local matched_chars = 0
    local total_distance = 0
    local last_match_pos = -1

    for i = 1, sample_len do
        if pattern_idx <= pattern_len and (
                sample_lower:sub(i, i) ==
                pattern_lower:sub(pattern_idx, pattern_idx)) then
            matched_chars = matched_chars + 1
            if consecutive == 0 and do_coloring then
                colored_sample = colored_sample .. color_matched
            end
            consecutive = consecutive + 1
            if consecutive > max_consecutive then
                max_consecutive = consecutive
            end

            if last_match_pos ~= -1 then
                total_distance = total_distance + (i - last_match_pos - 1)
            end
            last_match_pos = i

            pattern_idx = pattern_idx + 1
        else
            if consecutive ~= 0 and do_coloring then
                colored_sample = colored_sample .. color_normal
            end
            consecutive = 0
        end
        if do_coloring then
            colored_sample = colored_sample .. sample:sub(i, i)
        end
    end
    if consecutive ~= 0 and do_coloring then
        colored_sample = colored_sample .. color_normal
    end

    if pattern_idx <= pattern_len then
        return 0
    end

    local coverage = matched_chars / pattern_len * 20

    local consecutive_bonus = max_consecutive * 10

    local start_bonus = 0
    if sample_lower:sub(1, 1) == pattern_lower:sub(1, 1) then
        start_bonus = 15
    end

    local prefix_bonus = 0
    if sample_lower:find(pattern_lower, 1, true) == 1 then
        prefix_bonus = 25
    end

    local distance_penalty = total_distance * 2

    local score = coverage + consecutive_bonus + start_bonus + prefix_bonus - distance_penalty

    if sample_lower:find(pattern_lower, 1, true) then
        score = score + 30
    end

    return math.max(0, score), colored_sample
end

return M
