-- Standard components OOP wrappers (__index tables of metatables)

local Transform = {__index={
    get_pos=function(self) return __transform.get_pos(self.eid) end,
    set_pos=function(self, v) return __transform.set_pos(self.eid, v) end,
    get_size=function(self) return __transform.get_size(self.eid) end,
    set_size=function(self, v) return __transform.set_size(self.eid, v) end,
    get_rot=function(self) return __transform.get_rot(self.eid) end,
    set_rot=function(self, m) return __transform.set_rot(self.eid, m) end,
}}

local function new_Transform(eid)
    return setmetatable({eid=eid}, Transform)
end

local Rigidbody = {__index={
    is_enabled=function(self) return __rigidbody.is_enabled(self.eid) end,
    set_enabled=function(self, b) return __rigidbody.set_enabled(self.eid, b) end,
    get_vel=function(self) return __rigidbody.get_vel(self.eid) end,
    set_vel=function(self, v) return __rigidbody.set_vel(self.eid, v) end,
    get_size=function(self) return __rigidbody.get_size(self.eid) end,
    set_size=function(self, v) return __rigidbody.set_size(self.eid, v) end,
    get_gravity_scale=function(self) return __rigidbody.get_gravity_scale(self.eid) end,
    set_gravity_scale=function(self, s) return __rigidbody.set_gravity_scale(self.eid, s) end,
    get_linear_damping=function(self) return __rigidbody.get_linear_damping(self.eid) end,
    set_linear_damping=function(self, f) return __rigidbody.set_linear_damping(self.eid, f) end,
    is_vdamping=function(self) return __rigidbody.is_vdamping(self.eid) end,
    get_vdamping=function(self) return __rigidbody.get_vdamping(self.eid) end,
    set_vdamping=function(self, b) return __rigidbody.set_vdamping(self.eid, b) end,
    is_grounded=function(self) return __rigidbody.is_grounded(self.eid) end,
    is_crouching=function(self) return __rigidbody.is_crouching(self.eid) end,
    set_crouching=function(self, b) return __rigidbody.set_crouching(self.eid, b) end,
    get_body_type=function(self) return __rigidbody.get_body_type(self.eid) end,
    set_body_type=function(self, s) return __rigidbody.set_body_type(self.eid, s) end,
}}

local function new_Rigidbody(eid)
    return setmetatable({eid=eid}, Rigidbody)
end

local Skeleton = {__index={
    get_model=function(self, i) return __skeleton.get_model(self.eid, i) end,
    set_model=function(self, i, s) return __skeleton.set_model(self.eid, i, s) end,
    get_matrix=function(self, i) return __skeleton.get_matrix(self.eid, i) end,
    set_matrix=function(self, i, m) return __skeleton.set_matrix(self.eid, i, m) end,
    get_texture=function(self, s) return __skeleton.get_texture(self.eid, s) end,
    set_texture=function(self, s, s2) return __skeleton.set_texture(self.eid, s, s2) end,
    index=function(self, s) return __skeleton.index(self.eid, s) end,
    is_visible=function(self, i) return __skeleton.is_visible(self.eid, i) end,
    set_visible=function(self, i, b) return __skeleton.set_visible(self.eid, i, b) end,
    get_color=function(self) return __skeleton.get_color(self.eid) end,
    set_color=function(self, color) return __skeleton.set_color(self.eid, color) end,
    set_interpolated=function(self, b) return __skeleton.set_interpolated(self.eid, b) end,
}}

local function new_Skeleton(eid)
    return setmetatable({eid=eid}, Skeleton)
end

-- Entity class

local Entity = {__index={
    despawn=function(self) return entities.despawn(self.eid) end,
    get_skeleton=function(self) return entities.get_skeleton(self.eid) end,
    set_skeleton=function(self, s) return entities.set_skeleton(self.eid, s) end,
    get_component=function(self, name) return self.components[name] end,
    require_component=function(self, name)
        local component = self.components[name]
        if not component then
            error(("entity has no required component '%s'"):format(name))
        end
        return component
    end,
    has_component=function(self, name) return self.components[name] ~= nil end,
    get_uid=function(self) return self.eid end,
    def_index=function(self) return entities.get_def(self.eid) end,
    def_name=function(self) return entities.def_name(entities.get_def(self.eid)) end,
    get_player=function(self) return entities.get_player(self.eid) end,
    set_enabled=function(self, name, flag)
        local comp = self.components[name] 
        if comp then
            if flag then
                if comp.__disabled and comp.on_enable then
                    comp.on_enable()
                end
                comp.__disabled = nil
            else
                if not comp.__disabled and comp.on_disable then
                    comp.on_disable()
                end
                comp.__disabled = true
            end
        end
    end,
}}

local entities = {}

return {
    new_Entity = function(eid)
        local entity = setmetatable({eid=eid}, Entity)
        entity.transform = new_Transform(eid)
        entity.rigidbody = new_Rigidbody(eid)
        entity.skeleton = new_Skeleton(eid)
        entity.components = {}
        entities[eid] = entity;
        return entity
    end,
    get_Entity = function(eid)
        return entities[eid]
    end,
    remove_Entity = function(eid)
        local entity = entities[eid]
        if entity then
            entity.components = nil
            entities[eid] = nil;
        end
    end,
    update = function(tps, parts, part)
        for uid, entity in pairs(entities) do
            if uid % parts ~= part then
                goto continue
            end
            for _, component in pairs(entity.components) do
                local callback = component.on_update
                if not component.__disabled and callback then
                    local result, err = pcall(callback, tps)
                    if err then
                        debug.error(err)
                    end
                end
            end
            ::continue::
        end
    end,
    physics_update = function(delta)
        for uid, entity in pairs(entities) do
            for _, component in pairs(entity.components) do
                local callback = component.on_physics_update
                if not component.__disabled and callback then
                    local result, err = pcall(callback, delta)
                    if err then
                        debug.error(err)
                    end
                end
            end
        end
    end,
    render = function(delta)
        for _,entity in pairs(entities) do
            for _, component in pairs(entity.components) do
                local callback = component.on_render
                if not component.__disabled and callback then
                    local result, err = pcall(callback, delta)
                    if err then
                        debug.error(err)
                    end
                end
            end
        end
    end,
    get_all = function(uids)
        if uids == nil then
            return entities
        else
            local values = {}
            for _, uid in ipairs(uids) do
                values[uid] = entities[uid]
            end
            return values
        end
    end,
    __reset = function()
        entities = {}
    end
}
