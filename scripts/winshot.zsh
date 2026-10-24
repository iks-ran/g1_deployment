# usage: winshot.zsh <out.png>  — grab the RViz window on DISPLAY=:0
id=$(wmctrl -l | grep -i "rviz" | tail -1 | awk '{print $1}')
[[ -z $id ]] && { echo "no rviz window"; exit 1; }
wmctrl -i -a $id; sleep 1
eval $(xwininfo -id $id | awk '/Absolute upper-left X/{print "X="$4} /Absolute upper-left Y/{print "Y="$4} /Width/{print "W="$2} /Height/{print "H="$2}')
ffmpeg -loglevel error -y -f x11grab -video_size ${W}x${H} -i :0.0+$X,$Y -frames:v 1 $1 && echo saved $1
