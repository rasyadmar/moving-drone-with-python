step =1
waypoint_list = set([1,3])
while True:
    if step > 3:
        step =1
    if step == 1:
        if step in waypoint_list:
            print(step)
            print(len(waypoint_list))
            waypoint_list.remove(1)
            step = step + 1
        else:
            step = step + 1
    elif step == 2:
        if step in waypoint_list:
            print(step)
            step = step + 1
        else:
            step = step + 1
    elif step == 3:
        if step in waypoint_list:
            print(step)
            step = step + 1
        else:
            step = step + 1