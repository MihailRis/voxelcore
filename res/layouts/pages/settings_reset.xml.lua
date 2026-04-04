function reset(category)
	if category == "aud" then
		reset_audio()
	elseif category == "dsp" then
		reset_display()
	elseif category == "gfx" then
		reset_graphics()
	elseif category == "ctl" then
		reset_control()
	end
end

function reset_setting(name)
	app.set_setting(name, app.get_setting_info(name).def)
end

function reset_audio()
	reset_setting("audio.volume-master")
    reset_setting("audio.volume-regular")
    reset_setting("audio.volume-ui")
    reset_setting("audio.volume-ambient")
    reset_setting("audio.volume-music")
end

function reset_display()
	reset_setting("camera.fov")
    reset_setting("display.framerate")
    reset_setting("display.fullscreen")
	reset_setting("camera.shaking")
	reset_setting("camera.inertia")
	reset_setting("camera.fov-effects")
end

function reset_graphics()
	reset_setting("chunks.load-distance")
    reset_setting("chunks.load-speed")
    reset_setting("graphics.fog-curve")
    reset_setting("graphics.gamma")
    reset_setting("graphics.backlight")
end

function reset_control()
	input.reset_bindings()
end
