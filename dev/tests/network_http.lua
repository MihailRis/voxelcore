local response_received = false

network.get("https://api.github.com/repos/MihailRis/VoxelEngine-Cpp/releases/latest", function (s)
    print(json.parse(s).name)
    response_received = true
end, function (code)
    print("repond with code", code)
    response_received = true
end)

app.sleep_until(function () return response_received end, nil, 10)
