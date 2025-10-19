from __future__ import annotations

from typing import List

from src.models import CarPose, Cone, Path2D


class PathPlanning:
    """Student-implemented path planner.

    You are given the car pose and an array of detected cones, each cone with (x, y, color)
    where color is 0 for yellow (right side) and 1 for blue (left side). The goal is to
    generate a sequence of path points that the car should follow.

    Implement ONLY the generatePath function.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        """Return a list of path points (x, y) in world frame.

        Requirements and notes:
        - Cones: color==0 (yellow) are on the RIGHT of the track; color==1 (blue) are on the LEFT.
        - You may be given 2, 1, or 0 cones on each side.
        - Use the car pose (x, y, yaw) to seed your path direction if needed.
        - Return a drivable path that stays between left (blue) and right (yellow) cones.
        - The returned path will be visualized by PathTester.

        The path can contain as many points as you like, but it should be between 5-10 meters,
        with a step size <= 0.5. Units are meters.

        Replace the placeholder implementation below with your algorithm.
        """

        # Default: produce a short straight-ahead path from the current pose.
        # delete/replace this with your own algorithm.
        

        #split the cones to 2 lists
        b_cones = [cone for cone in self.cones if cone.color == 1]
        y_cones = [cone for cone in self.cones if cone.color == 0]

        #this sorts the list of cones to give proirity 
        #to the top most yellow cone and to the least bottom blue cone 
        y_cones.sort(key=lambda cone: cone.x)
        b_cones.sort(key=lambda cone: cone.x)

        if y_cones[0].x < self.car_pose.x or b_cones[0] < self.car_pose.x:
            y_cones.sort(key=lambda cone: cone.y)
            b_cones.sort(key=lambda cone: cone.y, reverse=True)
        else:
            y_cones.sort(key=lambda cone: cone.y, reverse=True)
            b_cones.sort(key=lambda cone: cone.y)

        print("blue cones: ") 
        print(b_cones)
        print("yellow cones: ") 
        print(y_cones)
        

        for y_cone in y_cones:
            seen = 0
            for b_cone in b_cones:
                if y_cone.x == b_cone.x:
                    seen+=1
            if seen == 0:
                if self.car_pose.x > y_cone.x:
                    b_cones.append(Cone(y_cone.x, y_cone.y-2, color=1))
                else:
                    b_cones.append(Cone(y_cone.x, y_cone.y+2, color=1))

        
        for b_cone in b_cones:
            seen = 0
            for y_cone in y_cones:
                if b_cone.x == y_cone.x:
                    seen+=1
            if seen == 0:
                if self.car_pose.x > b_cone.x:
                    y_cones.append(Cone(b_cone.x, b_cone.y+2, color=0))
                else:
                    y_cones.append(Cone(b_cone.x, b_cone.y-2, color=0))
                    

        
        if (self.car_pose.x > y_cones[0].x) or (self.car_pose.x > b_cones[0].x):
            y_cones.sort(key=lambda cone: (cone.x, -cone.y), reverse=True)
            b_cones.sort(key=lambda cone: (cone.x, cone.y), reverse=True)
            
            sign = -1
        else:
            y_cones.sort(key=lambda cone: (cone.x, -cone.y))
            b_cones.sort(key=lambda cone: (cone.x, cone.y))

            sign = 1

        print("blue cones: ") 
        print(b_cones)
        print("yellow cones: ") 
        print(y_cones)


        waypoints : Path2D = []
        waypoints.append((self.car_pose.x, self.car_pose.y))
        i_b = 0
        i_y = 0
        for i in range(0, min(len(b_cones), len(y_cones))):
            if y_cones[i_y].x == b_cones[i_b].x:
                waypoints.append((y_cones[i_y].x, (y_cones[i_y].y + b_cones[i_b].y) / 2))
                i_b+=1
                i_y+=1
            elif sign * y_cones[i_y].x < sign * b_cones[i_b].x:
                i_y+=1
            else:
                i_b+=1
                
    
        if (self.car_pose.x > y_cones[0].x) or (self.car_pose.x > b_cones[0].x):
            waypoints.insert(1, (waypoints[1][0]+0.5, waypoints[1][1]))
        else:
            waypoints.insert(1, (waypoints[1][0]-0.5, waypoints[1][1]))

        print("waypoints: ")
        print(waypoints)





        import math

        path : Path2D = []
        step = 0.2

        current_x = self.car_pose.x
        current_y = self.car_pose.y

        car_at_wp = (self.car_pose.x, self.car_pose.y)

        for j in range(1, len(waypoints)):

            #print(f"waypoint[{j}]")

            dx_goal = waypoints[j][0] - current_x
            dy_goal = waypoints[j][1] - current_y

            theta_goal = math.atan2(dy_goal, dx_goal)

            dist = math.hypot(dx_goal, dy_goal)
            num_steps = round(dist/step)

            for i in range(1, num_steps+1):

                dx = math.cos(theta_goal) * step * i
                dy = math.sin(theta_goal) * step * i

                current_x = car_at_wp[0] + dx
                current_y = car_at_wp[1] + dy

                #print("current x:")
                #print(current_x)
                #print("current y")
                #print(current_y)

                path.append((current_x, current_y))    
            
            car_at_wp = (current_x, current_y)

        return path
