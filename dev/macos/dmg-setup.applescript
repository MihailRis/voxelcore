on run argv
    set volumeName to item 1 of argv
    tell application "Finder"
        set targetDisk to missing value
        set candidates to every disk whose name is volumeName
        repeat with i from 1 to count of candidates
            set d to item i of candidates
            set diskPath to POSIX path of (d as alias)
            try
                do shell script "test -w " & quoted form of diskPath
                set targetDisk to d
                exit repeat
            end try
        end repeat
        if targetDisk is missing value then error "writable disk not found: " & volumeName

        tell targetDisk
            open
            set win to container window
            set current view of win to icon view
            set toolbar visible of win to false
            set statusbar visible of win to false
            set the bounds of win to {200, 120, 800, 548}
            set opts to the icon view options of win
            set arrangement of opts to not arranged
            set icon size of opts to 128
            set text size of opts to 13
            set background picture of opts to file ".background:background.tiff"
            set position of item "VoxelCore.app" of win to {180, 200}
            set position of item "Applications" of win to {420, 200}
            update without registering applications
            delay 2
            close
        end tell
    end tell
end run
