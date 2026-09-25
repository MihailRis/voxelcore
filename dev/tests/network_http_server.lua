local function to_str(v)
    if type(v) == 'string' then
        return v
    end
    return Bytearray_as_string(v)
end

do
    local server = network.http_open(network.find_free_port(), function(request)
        if request.path == "/hello" then
            return {
                status = 200,
                headers = {["Content-Type"] = "text/plain"},
                body = "Hello, " .. request.query
            }
        end
        return {status = 404, body = "Not Found"}
    end)

    local port = server:get_port()
    local done = false
    local body

    network.get("http://127.0.0.1:" .. port .. "/hello?world", function(s)
        body = s
        done = true
    end, function(code, s)
        body = s
        done = true
        print("error", code, s)
    end)

    app.sleep_until(function() return done end, nil, 5)

    asserts.equals("Hello, world", body)
    server:close()
end

do
    local server = network.http_open(network.find_free_port(), function(request)
        local data = request:json()
        return network.http_json({sum = data.a + data.b})
    end)

    local port = server:get_port()
    local done = false
    local body

    network.post(
        "http://127.0.0.1:" .. port .. "/",
        json.tostring({a = 2, b = 3}),
        function(s)
            body = s
            done = true
        end,
        function(code, s)
            body = s
            done = true
            print("error", code, s)
        end
    )

    app.sleep_until(function() return done end, nil, 5)

    asserts.equals(5, json.parse(to_str(body)).sum)
    server:close()
end

do
    local router = network.http_router()
    router:get("/users/:id", function(request, id)
        return network.http_json({id = id})
    end)

    local server = network.http_open(network.find_free_port(), router)
    local port = server:get_port()
    local done = false
    local body

    network.get("http://127.0.0.1:" .. port .. "/users/42", function(s)
        body = s
        done = true
    end, function(code, s)
        body = s
        done = true
        print("error", code, s)
    end)

    app.sleep_until(function() return done end, nil, 5)

    asserts.equals("42", json.parse(body).id)
    server:close()
end
