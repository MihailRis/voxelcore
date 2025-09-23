for i=1,3 do
    print(string.format("iteration %s", i + 1))
    local text = ""
    local complete = false

    for j=1,100 do
        text = text .. math.random(0, 9)
    end

    local server = network.tcp_open(7645, function (client)
        print("client connected")
        start_coroutine(function()
            print("client-listener started")
            local received_text = ""
            while client:is_alive() and #received_text < #text do
                local received = client:recv(512)
                if received then
                    received_text = received_text .. utf8.tostring(received)
                    print(string.format("received %s byte(s) from client", #received))
                end
                coroutine.yield()
            end
            asserts.equals (text, received_text)
            complete = true
        end, "client-listener")
    end)

    network.tcp_connect("localhost", 7645, function (socket)
        print("connected to server")
        start_coroutine(function()
            print("data-sender started")
            local ptr = 1
            while ptr <= #text do
                local n = math.random(1, 20)
                socket:send(string.sub(text, ptr, ptr + n - 1))
                print(string.format("sent %s byte(s) to server", n))
                ptr = ptr + n
            end
            socket:close()
        end, "data-sender")
    end)

    app.sleep_until(function () return complete end, nil, 5)
    server:close()
end
