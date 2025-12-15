local PriorityQueue = {}
PriorityQueue.__index = PriorityQueue

function PriorityQueue:new()
    return setmetatable({ heap = {}, index_map = {} }, self)
end

-- Adds item to the queue. Item must have an "id" field.
function PriorityQueue:push(item, priority)
    local node = { item = item, priority = priority }
    table.insert(self.heap, node)
    local index = #self.heap

    if not item.id then
      error("PriorityQueue:push - item must have an 'id' field")
    end

    self.index_map[item.id] = index
    self:_siftUp(index)
end

function PriorityQueue:pop()
    if #self.heap == 0 then return nil end
    local top = self.heap[1]
    self.index_map[top.item.id] = nil

    local last = table.remove(self.heap)
    if #self.heap > 0 then
        self.heap[1] = last
        self.index_map[last.item.id] = 1
        self:_siftDown(1)
    end

    return top.item, top.priority
end

function PriorityQueue:peek()
    if #self.heap == 0 then return nil end
    local top = self.heap[1]
    return top.item, top.priority
end

PriorityQueue.front = PriorityQueue.peek

function PriorityQueue:is_empty()
    return #self.heap == 0
end

function PriorityQueue:size()
    return #self.heap
end

function PriorityQueue:clear()
    self.heap = {}
end

-- Prints queue contents for debugging purposes
function PriorityQueue:dump()
    print("PriorityQueue dump:")
    for i, node in ipairs(self.heap) do
        print(i, node.item, node.priority)
    end
end

function PriorityQueue:update_priority(item_id, new_priority)
    local index = self.index_map[item_id]
    if not index then return false end

    local node = self.heap[index]
    local old_priority = node.priority
    node.priority = new_priority

    if new_priority < old_priority then
        self:_siftUp(index)
    elseif new_priority > old_priority then
        self:_siftDown(index)
    end
    return true
end

function PriorityQueue:update_front_priority(new_priority)
    if #self.heap == 0 then return end
    local old_priority = self.heap[1].priority
    self.heap[1].priority = new_priority

    if new_priority > old_priority then
        self:_siftDown(1)
    end
end

function PriorityQueue:get_by_id(item_id)
    return self.heap[self.index_map[item_id]]
end

function PriorityQueue:exists(item_id)
    return self.index_map[item_id] ~= nil
end

function PriorityQueue:remove_by_id(item_id)
    local index = self.index_map[item_id]
    if not index then return false end

    self.index_map[item_id] = nil
    local last = table.remove(self.heap)

    if index == #self.heap + 1 then
        return true
    end

    self.heap[index] = last
    self.index_map[last.item.id] = index

    local new_priority = last.priority
    self:_siftDown(index)
    self:_siftUp(index) -- maybe an if/else and a single function call would be better instead of both siftUp & siftDown, but not sure

    return true
end


-- internal methods
function PriorityQueue:_swap(i, j)
    local heap = self.heap
    heap[i], heap[j] = heap[j], heap[i]
    self.index_map[heap[i].item.id] = i
    self.index_map[heap[j].item.id] = j
end

function PriorityQueue:_siftUp(index)
    while index > 1 do
        local parent = math.floor(index / 2)
        if self.heap[parent].priority <= self.heap[index].priority then break end
        self:_swap(parent, index)
        index = parent
    end
end

function PriorityQueue:_siftDown(index)
    local size = #self.heap
    while true do
        local left = 2 * index
        local right = left + 1
        local smallest = index

        if left <= size and self.heap[left].priority < self.heap[smallest].priority then
            smallest = left
        end
        if right <= size and self.heap[right].priority < self.heap[smallest].priority then
            smallest = right
        end
        if smallest == index then break end

        self:_swap(smallest, index)
        index = smallest
    end
end

return PriorityQueue
