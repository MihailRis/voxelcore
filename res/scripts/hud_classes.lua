local Text3D = {__index={
    hide=function(self) return gfx.text3d.hide(self.id) end,
    get_pos=function(self) return gfx.text3d.get_pos(self.id) end,
    set_pos=function(self, v) return gfx.text3d.set_pos(self.id, v) end,
    get_axis_x=function(self) return gfx.text3d.get_axis_x(self.id) end,
    set_axis_x=function(self, v) return gfx.text3d.set_axis_x(self.id, v) end,
    get_axis_y=function(self) return gfx.text3d.get_axis_y(self.id) end,
    set_axis_y=function(self, v) return gfx.text3d.set_axis_y(self.id, v) end,
    set_rotation=function(self, m) return gfx.text3d.set_rotation(self.id, m) end,
    get_text=function(self) return gfx.text3d.get_text(self.id) end,
    set_text=function(self, s) return gfx.text3d.set_text(self.id, s) end,
    update_settings=function(self, t) return gfx.text3d.update_settings(self.id, t) end,
}}

local Skeleton = {__index={
    index=function(self, s) return gfx.skeletons.index(self.name, s) end,
    get_model=function(self, i) return gfx.skeletons.get_model(self.name, i) end,
    set_model=function(self, i, s) return gfx.skeletons.set_model(self.name, i, s) end,
    get_matrix=function(self, i) return gfx.skeletons.get_matrix(self.name, i) end,
    set_matrix=function(self, i, m) return gfx.skeletons.set_matrix(self.name, i, m) end,
    get_texture=function(self, i) return gfx.skeletons.get_texture(self.name, i) end,
    set_texture=function(self, i, s) return gfx.skeletons.set_texture(self.name, i, s) end,
    is_visible=function(self, i) return gfx.skeletons.is_visible(self.name, i) end,
    set_visible=function(self, i, b) return gfx.skeletons.set_visible(self.name, i, b) end,
    get_color=function(self, i) return gfx.skeletons.get_color(self.name, i) end,
    set_color=function(self, i, c) return gfx.skeletons.set_color(self.name, i, c) end,
}}

gfx.text3d.new = function(pos, text, preset, extension)
    local id = gfx.text3d.show(pos, text, preset, extension)
    return setmetatable({id=id}, Text3D)
end

gfx.skeletons = __skeleton
gfx.skeletons.get = function(name)
    if gfx.skeletons.exists(name) then
        return setmetatable({name=name}, Skeleton)
    end
end
