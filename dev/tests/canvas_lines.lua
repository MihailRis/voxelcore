local W, H = 128, 128

local canvas = Canvas({W, H})
canvas:clear(0xff202020)

local WHITE = 0xffffffff
local RED  = 0xffff4040
local GREEN = 0xff40ff40
local BLUE = 0xff4080ff
local YELLOW = 0xffffff40

-- border
canvas:line(0, 0, W - 1, 0, WHITE)
canvas:line(W - 1, 0, W - 1, H - 1, WHITE)
canvas:line(W - 1, H - 1, 0, H - 1, WHITE)
canvas:line(0, H - 1, 0, 0, WHITE)

-- horizontal
canvas:line(-40, 16, W + 40, 16, RED)
canvas:line(-40, H / 2, W + 40, H / 2, RED)
canvas:line(-40, H - 17, W + 40, H - 17, RED)

-- vertical
canvas:line(16, -40, 16, H + 40, GREEN)
canvas:line(W / 2, -40, W / 2, H + 40, GREEN)
canvas:line(W - 17, -40, W - 17, H + 40, GREEN)

-- diagonals
canvas:line(-40, -40, W + 40, H + 40, BLUE)
canvas:line(-40, H + 40, W + 40, -40, BLUE)

canvas:line(W / 2, -40, W + 40, H / 2, BLUE)
canvas:line(-40, H / 2, W / 2, H + 40, BLUE)

-- outer
canvas:line(-30, -30, 20, 20, YELLOW)
canvas:line(W - 20, H - 20, W + 30, H + 30, YELLOW)

canvas:line(W + 30, -30, W - 20, 20, YELLOW)
canvas:line(-30, H + 30, 20, H - 20, YELLOW)

-- completely outside
canvas:line(-50, -20, -10, H + 20, 0xffff00ff)
canvas:line(W + 10, -20, W + 50, H + 20, 0xffff00ff)
canvas:line(-20, -20, W + 20, -10, 0xffff00ff)
canvas:line(-20, H + 10, W + 20, H + 30, 0xffff00ff)

local expected = Canvas.decode(
    file.read_bytes("script:attachments/line_clipping_test.png"), "png"
)
asserts.equals(expected, canvas)
