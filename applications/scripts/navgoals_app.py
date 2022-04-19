#! /usr/bin/env python3

import rospy
import robot_api

def wait_for_time():                                                                          
    """Wait for simulated time to begin.
    """                                                                                       
    while rospy.Time().now().to_sec() == 0:                                                   
        pass

def main():
    rospy.init_node('navgoals_app')
    wait_for_time()

    print('Welcome to the map annotator!')
    show_commands()

    nav_goal = robot_api.NavGoal()

    txt = ''
    while True:
        txt = input('> ').split(' ')
        if len(txt) < 1: continue
        command = txt[0].lower()
        if len(txt) == 1:
            if command == 'list':
                print('Poses:')
                for loc in nav_goal.get_locations():
                    print(f'\t{loc}')
            elif command == 'help':
                show_commands()
            elif command == 'quit':
                break
        elif len(txt) == 2:
            if command == 'save':
                nav_goal.save_current_pose(txt[1])
                print(f'Saved {txt[1]}')
            elif command == 'delete':
                if nav_goal.delete(txt[1]):
                    print(f'{txt[1]} deleted')
                else:
                    print(f'No such pose {txt[1]}')
            elif command == 'goto':
                if nav_goal.goto(txt[1]):
                    print(f'Going to {txt[1]}')
                else:
                    print(f'No such pose {txt[1]}')
        else:
            print('Couldn\'t recognize command')
    print('all done')


def show_commands():
    print('Commands:')
    print('\tlist: List saved poses.')
    print('\tsave <name>: Save the robot\'s current pose as <name>. Overwrites if <name> already exists.')
    print('\tdelete <name>: Delete the pose given by <name>.')
    print('\tgoto <name>: Sends the robot to the pose given by <name>.')
    print('\thelp: Show this list of commands')
  
  
  
  
  


if __name__ == '__main__':
  main()
