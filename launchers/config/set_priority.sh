renice -n -15 -p `pgrep visual_frontend`
renice -n 20 -p `pgrep show_frame_node`
renice -n -10 -p `pgrep smart_smoother`
renice -n -16 -p `pgrep image_proc`
renice -n -14 -p `pgrep scan_to_match`
