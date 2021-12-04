#!/bin/bash
echo "-------------------------------------------------------------"
echo "TU Many Bots configuration"
echo "-------------------------------------------------------------"
echo "tme_ROBOT_ENV: [simple_corridor, maze, maze_clutter, maze_clutter_limited]"
echo "tme_start_num: [2 - 5]"
echo "tme_expl_method: [nearest, greedy, minPos, combined]"
echo "tme_stop_time: number in seconds the simluation should run"
echo "-------------------------------------------------------------"
if [ -z ${ROBOT_ENV+x} ]
then
  world=v2_maze
else
  world=$ROBOT_ENV
fi

if [ -z ${tme_expl_method+x} ]
then
  expl_method=combined
else
  expl_method=$tme_expl_method
fi

if [ -z ${tme_stop_time+x} ]
then
  stop_time=500
else
  stop_time=$tme_stop_time
fi

if [ -z ${tme_start_num+x} ]
then
  start_num=4
  export tme_start_num=$start_num
else
  start_num=$tme_start_num
  export tme_start_num=$start_num
fi

if [[ $tme_start_num ]]
then
  start_two=True
  if [ $tme_start_num -ge 3 ]
  then
    start_three=True
    if [ $tme_start_num -ge 4 ]
    then
      start_four=True
      if [ $tme_start_num -ge 5 ]
      then
        start_five=True
      fi
    else
      start_four=False
      start_five=False
    fi
  else
    start_three=False
    start_four=False
    start_five=False
  fi
else 
  start_two=True
  start_three=False
  start_four=False
  start_five=False
fi

export tme_ROBOT_ENV=$world
export tme_ROBOT=rto-1
export tme_ROBOT_BLIND=rto-blind
export tme_start_two=$start_two
export tme_start_three=$start_three
export tme_start_four=$start_four
export tme_start_five=$start_five
export tme_expl_method=$expl_method
export tme_stop_time=$stop_time
export tme_start_num=$start_num

echo "selected world is: $world"
echo "starting num of robots $start_num: [2: $start_two, 3: $start_three, 4: $start_four, 5: $start_five] "
echo "chosen method is: $expl_method"
echo "chosen stop time is: $stop_time"
echo "-------------------------------------------------------------"

if [ $world="simple_corridor" ]
then
  export tme_start_robot1_x="0.0"
  export tme_start_robot1_y="0.0"
  export tme_start_robot1_z="0.0"
  export tme_start_robot1_yaw="0"

  export tme_start_robot2_x="1.0"
  export tme_start_robot2_y="0.0"
  export tme_start_robot2_z="0.0"
  export tme_start_robot2_yaw="0"

  export tme_start_robot3_x="-1.0"
  export tme_start_robot3_y="1.0"
  export tme_start_robot3_z="0.0"
  export tme_start_robot3_yaw="0.0"

  export tme_start_robot4_x="-1.0"
  export tme_start_robot4_y="-1.0"
  export tme_start_robot4_z="0.0"
  export tme_start_robot4_yaw="0.0"

  export tme_start_robot5_x="1.0"
  export tme_start_robot5_y="1.0"
  export tme_start_robot5_z="0.0"
  export tme_start_robot5_yaw="0.0"


fi
if [ $world="maze" ]
then
  export tme_start_robot1_x="1.0"
  export tme_start_robot1_y="0.0"
  export tme_start_robot1_z="0.0"
  export tme_start_robot1_yaw="0.0"

  export tme_start_robot2_x="-1.0"
  export tme_start_robot2_y="0.0"
  export tme_start_robot2_z="0.0"
  export tme_start_robot2_yaw="0.0"

  export tme_start_robot3_x="-1.0"
  export tme_start_robot3_y="1.0"
  export tme_start_robot3_z="0.0"
  export tme_start_robot3_yaw="0.0"

  export tme_start_robot4_x="-1.0"
  export tme_start_robot4_y="-1.0"
  export tme_start_robot4_z="0.0"
  export tme_start_robot4_yaw="0.0"

  export tme_start_robot5_x="1.0"
  export tme_start_robot5_y="1.0"
  export tme_start_robot5_z="0.0"
  export tme_start_robot5_yaw="0.0"
fi
if [ $world="maze_simple" ]
then
  export tme_start_robot1_x="1.0"
  export tme_start_robot1_y="0.0"
  export tme_start_robot1_z="0.0"
  export tme_start_robot1_yaw="0.0"

  export tme_start_robot2_x="-1.0"
  export tme_start_robot2_y="0.0"
  export tme_start_robot2_z="0.0"
  export tme_start_robot2_yaw="0.0"

  export tme_start_robot3_x="-1.0"
  export tme_start_robot3_y="1.0"
  export tme_start_robot3_z="0.0"
  export tme_start_robot3_yaw="0.0"

  export tme_start_robot4_x="-1.0"
  export tme_start_robot4_y="-1.0"
  export tme_start_robot4_z="0.0"
  export tme_start_robot4_yaw="0.0"

  export tme_start_robot5_x="1.0"
  export tme_start_robot5_y="1.0"
  export tme_start_robot5_z="0.0"
  export tme_start_robot5_yaw="0.0"
fi
if [ $world="maze_simple_2" ]
then
  export tme_start_robot1_x="1.0"
  export tme_start_robot1_y="0.0"
  export tme_start_robot1_z="0.0"
  export tme_start_robot1_yaw="0.0"

  export tme_start_robot2_x="-1.0"
  export tme_start_robot2_y="0.0"
  export tme_start_robot2_z="0.0"
  export tme_start_robot2_yaw="0.0"

  export tme_start_robot3_x="-1.0"
  export tme_start_robot3_y="1.0"
  export tme_start_robot3_z="0.0"
  export tme_start_robot3_yaw="0.0"

  export tme_start_robot4_x="-1.0"
  export tme_start_robot4_y="-1.0"
  export tme_start_robot4_z="0.0"
  export tme_start_robot4_yaw="0.0"

  export tme_start_robot5_x="1.0"
  export tme_start_robot5_y="1.0"
  export tme_start_robot5_z="0.0"
  export tme_start_robot5_yaw="0.0"
fi
if [ $world="maze_clutter" ]
then
  export tme_start_robot1_x="1.0"
  export tme_start_robot1_y="0.0"
  export tme_start_robot1_z="0.0"
  export tme_start_robot1_yaw="0.0"

  export tme_start_robot2_x="-1.0"
  export tme_start_robot2_y="0.0"
  export tme_start_robot2_z="0.0"
  export tme_start_robot2_yaw="0.0"

  export tme_start_robot3_x="-1.0"
  export tme_start_robot3_y="1.0"
  export tme_start_robot3_z="0.0"
  export tme_start_robot3_yaw="0.0"

  export tme_start_robot4_x="-1.0"
  export tme_start_robot4_y="-1.0"
  export tme_start_robot4_z="0.0"
  export tme_start_robot4_yaw="0.0"

  export tme_start_robot5_x="1.0"
  export tme_start_robot5_y="1.0"
  export tme_start_robot5_z="0.0"
  export tme_start_robot5_yaw="0.0"
fi
if [ $world="maze_clutter_limited" ]
then
  export tme_start_robot1_x="1.0"
  export tme_start_robot1_y="0.0"
  export tme_start_robot1_z="0.0"
  export tme_start_robot1_yaw="0.0"

  export tme_start_robot2_x="-1.0"
  export tme_start_robot2_y="0.0"
  export tme_start_robot2_z="0.0"
  export tme_start_robot2_yaw="0.0"

  export tme_start_robot3_x="-1.0"
  export tme_start_robot3_y="1.0"
  export tme_start_robot3_z="0.0"
  export tme_start_robot3_yaw="0.0"

  export tme_start_robot4_x="-1.0"
  export tme_start_robot4_y="-1.0"
  export tme_start_robot4_z="0.0"
  export tme_start_robot4_yaw="0.0"

  export tme_start_robot5_x="1.0"
  export tme_start_robot5_y="1.0"
  export tme_start_robot5_z="0.0"
  export tme_start_robot5_yaw="0.0"
fi