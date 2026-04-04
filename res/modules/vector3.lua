local vector3 = 
{
    __type = "vec3",
    __call = function(vector3, x, y, z) 
            return vector3:new(x, y, z) end
}

vector3.__index = vector3





-- vec3 construct
function vector3:new(x, y, z)
    if type(x) ~= "number" or type(y) ~= "number" or type(z) ~= "number" then
        print("\n(( TypeError : new vec3))\nType arg vec3(x, y, z) = " .. "(" .. type(x) .. ", " .. type(y) .. ", " .. type(z) .. ")")
        error("Invalid input argument. Expected a vector obj. [ vec3(scalar, scalar, scalar) ]\n\n")
    end
    local obj = {
      x = x or 0,
      y = y or 0,
      z = z or 0
    }
    setmetatable(obj, self)
    return obj
  end

-- str vec
function vector3:strvec3()
    return "(" .. self.x .. ", " .. self.y .. ", " .. self.z .. ")"
end
  
-- dot product
-- @param {vec3}
-- @return {number} -> float || integer
-- @usage:
--     local v1 = vec3(10, 15)
--     local v2 = vec3(15, 10, 1)
--     print(v1:dot(v2)) -- Output: 301
function vector3:dot(v)
    if type(v) == "number" then
        print("\n(( TypeError : dot ))\nType arg dot(vector3), your arg: " .. "dot(" .. type(v) .. ")")
        error("Invalid input argument. Expected a vector3 obj.\n")
    end
    return self.x * v.x + self.y * v.y + self.z * v.z
end
  
-- cross product
function vector3:cross(v)
    if type(v) == "number" then
        print("\n(( TypeError : cross ))\nType arg cross(vector3), your arg: " .. "cross(" .. type(v) .. ")")
        error("Invalid input argument. Expected a vector3 obj.\n")
    end
    return vector3:new(
      self.y * v.z - self.z * v.y,
      self.z * v.x - self.x * v.z,
      self.x * v.y - self.y * v.x
    )
end
  
-- round vector components
function vector3:round(decimals)
    decimals = decimals or 0
    self.x = math.floor(self.x * 10^decimals + 0.5) / 10^decimals
    self.y = math.floor(self.y * 10^decimals + 0.5) / 10^decimals
    self.z = math.floor(self.z * 10^decimals + 0.5) / 10^decimals
    return self
end
  

-- check if two vectors are parallel
-- @param {vec3}
-- @return {bool}
-- @usage:
--      local v1 = vec3(10, 15, 1)
--      local v2 = vec3(10, 15, 1)
--      print(v1:isParallel(v2)) -- Output: true
function vector3:isParallel(val)
    if type(val) ~= "table" or type(val) == "number" then
        print("\n(( TypeError : isParallel ))\nType arg isParallel(vector3)")
        error("Invalid input argument. Expected a vector3 object.\n")
    end
    return self:cross(val):len() < 1e-6
end

-- magnitude (length) vec
-- @return {number}
function vector3:len()
    return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)
end

-- project vec3
-- @param vector {vec3} - The direction vector
-- @return {vec3} - The projected vector3
function vector3:proj(vector)
    if type(vector) == "number" then
        print("\n(( TypeError : proj(vec3) ))\nType arg proj(vector3)")
        error("Invalid input argument. Expected a vector3 object\n")
    end
    local dot_product = self:dot(vector)
    return vector:new(dot_product * (vector.x / vector:len()^2), dot_product * (vector.y / vector:len()^2), dot_product * (vector.z / vector:len()^2))
end

-- normalize vector
-- @return {number}
-- @usage:
--      local v1 = vec3(10, 15, 1)
--      print(v1:norm()) -- Output: 18.1
function vector3:norm()
    local length_vectors = self:len()
    return vector3:new(self.x / length_vectors, self.y / length_vectors, self.z / length_vectors)
end


-- linear interpolation (lerp) for vector3
-- @param b {vec3} - The target vector3 object
-- @param t {number} - (0 <= t <= 1) the interpolation factor
-- @return {vec3}
function vector3:lerp(b, t)
    if type(b) ~= "table" then
        print("\n(( TypeError : lerp(vec3) ))\nType arg lerp(vec3, number)")
        error("Invalid input argument. Expected a vector3 object\n")
    end
    if type(t) ~= "number" or t < 0 or t > 1 then
        error("Invalid input argument. Expected a number between 0 and 1 .. (0 <= t <= 1)\n")
    end
    return vector3:new(self.x + t * (b.x - self.x), self.y + t * (b.y - self.y), self.z + t * (b.z - self.z));
end

-- distance to line vec3
-- @param point1 {vec3} - The first point on the line
-- @param point2 {vec3} - The second point on the line
-- @return {number} - The shortest distance from the point to the line
function vector3:dist2line(point1, point2)
    if type(point1) ~= "table" or type(point2) ~= "table" then
        print("\n(( TypeError : dist2line(vec3) ))\nType arg dist2line(vector3 , vector3)")
        error("Invalid input argument. Expected two vector3 objects\n")
    end
    local line_vector = point2 - point1
    local point_to_line = self - point1
    local projection = point_to_line - line_vector:proj(point_to_line)
    return projection:len()
end


-- angle between vec3
-- @param vector {vec3} - The other vector3
-- @return {number} -> rad .. angle between 2 vectors
function vector3:abtw(vector)
    if type(vector) == "number" then
        print("\n(( TypeError : abtw(vec3) ))\nType arg abtw(vector3)")
        error("Invalid input argument. Expected a vector3 object\n")
    end
    local dot_prod = self:dot(vector)
    local len_prod = self:len() * vector:len()
    if len_prod == 0 then
        return 0
    end
    return math.acos(dot_prod / len_prod)
end


-- rot vec3
-- @param angle {number} -> float || int
-- @param axis {string}  .. if nil => rot axis z
-- @param convert2deg {bool} .. if true => deg convert to rad
-- @return {vec3}
function vector3:rot(angle, axis, convert2deg)
    if convert2deg == true then
        angle = math.rad(angle)
    end

    if type(axis) == "string" then
        local cx, sx = math.cos(angle), math.sin(angle)
        if axis == "x" then
            local x, y, z = self.x, self.y, self.z
            self.y = cx * y - sx * z
            self.z = sx * y + cx * z
        elseif axis == "y" then
            local x, y, z = self.x, self.y, self.z
            self.x = cx * x + sx * z
            self.z = -sx * x + cx * z
        elseif axis == "z" then
            local x, y, z = self.x, self.y, self.z
            self.x = cx * x - sx * y
            self.y = sx * x + cx * y
        end
    elseif axis == nil then
        local cx, sx, cy, sy = math.cos(angle), math.sin(angle), math.cos(angle), math.sin(angle)
        local x, y, z = self.x, self.y, self.z
        self.x = cx * x - sx * y
        self.y = sx * x + cx * y
        self.z = cy * z - sy * self:len()
        local len = self:len()
        if len ~= 0 then
            self.z = self.z / len
        end
    end
    return self:round(15)
end


-- add vec  
function vector3.__add(value_1, value_2)
    if type(value_1) == "number" then
        if value_1 == 0 then
            return value_2
        end
        return vector3:new(value_2.x + value_1, value_2.y + value_1, value_2.z + value_1)
    else
        if type(value_2) == "number" then
            if value_2 == 0 then
                return value_1
            end
            return vector3:new(value_1.x + value_2, value_1.y + value_2, value_1.z + value_2)
        else
            if value_1.z ~= nil and value_2.z ~= nil then
                return vector3:new(value_1.x + value_2.x, value_1.y + value_2.y, value_1.z + value_2.z)
            else
                return vector3:new(value_1.x + value_2.x, value_1.y + value_2.y)
            end
        end
    end
end





function vector3.__sub(value_1, value_2)
    if type(value_1) == "number" then
        if value_1 == 0 then
            return value_2
        end
        return vector3:new(value_2.x - value_1, value_2.y - value_1, value_2.z - value_1)
    else
        if type(value_2) == "number" then
            if value_2 == 0 then
                return value_1
            end
            return vector3:new(value_1.x - value_2, value_1.y - value_2, value_1.z - value_2)
        else
            if value_1.z ~= nil and value_2.z ~= nil then
                return vector3:new(value_1.x - value_2.x, value_1.y - value_2.y, value_1.z - value_2.z)
            else
                return vector3:new(value_1.x - value_2.x, value_1.y - value_2.y)
            end
        end
    end
end





-- mul vec
function vector3.__mul(value_1, value_2)
    if type(value_1) == "number" then
        if value_1 == 0 then
            return value_2
        end
        return vector3:new(value_2.x * value_1, value_2.y * value_1, value_2.z * value_1)
    else
        if type(value_2) == "number" then
            if value_2 == 0 then
                return value_1
            end
            return vector3:new(value_1.x * value_2, value_1.y * value_2, value_1.z * value_2)
        else
            if value_1.z ~= nil and value_2.z ~= nil then
                return vector3:new(value_1.x * value_2.x, value_1.y * value_2.y, value_1.z * value_2.z)
            else
                return vector3:new(value_1.x * value_2.x, value_1.y * value_2.y)
            end
        end
    end
end

function vector3.__pow(value_1, value_2)
    if type(value_1) == "number" then
        if value_1 == 0 then
            return value_2
        end
        return vector3:new(value_2.x ^ value_1, value_2.y ^ value_1, value_2.z ^ value_1)
    else
        if type(value_2) == "number" then
            if value_2 == 0 then
                return value_1
            end
            return vector3:new(value_1.x ^ value_2, value_1.y ^ value_2, value_1.z ^ value_2)
        else
            if value_1.z ~= nil and value_2.z ~= nil then
                return vector3:new(value_1.x ^ value_2.x, value_1.y ^ value_2.y, value_1.z ^ value_2.z)
            else
                return vector3:new(value_1.x ^ value_2.x, value_1.y ^ value_2.y)
            end
        end
    end
end



-- div vec
function vector3.__div(value_1, value_2)
    if type(value_1) == "number" then
        if value_1 == 0 then
            return value_2
        end
        return vector3:new(value_2.x / value_1, value_2.y / value_1, value_2.z / value_1)
    else
        if type(value_2) == "number" then
            if value_2 == 0 then
                return value_1
            end
            return vector3:new(value_1.x / value_2, value_1.y / value_2, value_1.z / value_2)
        else
            if value_1.z ~= nil and value_2.z ~= nil then
                return vector3:new(value_1.x / value_2.x, value_1.y / value_2.y, value_1.z / value_2.z)
            else
                return vector3:new(value_1.x / value_2.x, value_1.y / value_2.y)
            end
        end
    end
end




-- eq vec
function vector3.__eq(a, b)
    return a.x == b.x and a.y == b.y and a.z == b.z
end



function vector3.__tostring(vcrt3)
    return vcrt3:strvec3()
end

return setmetatable(vector3, vector3)

