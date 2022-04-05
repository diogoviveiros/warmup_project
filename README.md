# Introduction to Robotics: Warmup Lab (Diogo Viveiros)


# Driving in a  Square

Making the robot drive in a square seemed like a relatively simple task to me. Of course, a square is made up of 4 equal lines, so all I had to do was make the robot go straight for a period of time, turn 90 degrees, then go straight for that period of time, and just do that until it gets back to its original position. 

## Code Explanation

In order to accomplish the motions that I described in the previous paragraph, I had to implement three essential Twist messages in my run function. The first one was forward_twist, which simply moved the robot in the x direction. The second message was turn_twist, which made the robot turn at a rate of pi/2 radians per second, or 90 degrees in one second. The final message, stop_twist, was also important in order to ensure that the robot would not still be moving forwards when turning, or vice-versa. I would make the robot move forwards for 3 seconds, then stop for a second, then turn for 1.1 seconds. I did 1.1 instead of just 1 second in order to account for any frictions and other real-world effects that would make the turning not exactly perfect. Then, I put all of this code in a loop that repeated four times, meaning the robot would always make a perfect square path and would stop approximately where it began. 

## GIF

![Going Around in Squares GIF](DrivingSquare.gif)

## Challenges

Coming from an Arduino development background, it was a little challenging at first to change some of my preconceptions about how some functions in ROS worked, primarily sleep. In the Arduino IDE, sleep (or delay) makes the device completely pause for that period of time. However, in ROS, it actually has the opposite effect, where sleep actually makes the function that we previously ran keep going until that sleep period is over and the new function is then executed. This is a little shift from what I am used to, but I quickly got the hang of it! Really, the main challenge of this project was getting Ubuntu, ROS, and all of the networking required properly installed and working, but this was a fruitful exercise in order to get everything to finally function properly on my machine. 

## Future Work

Had I had more time to improve the behavior a little bit, I would have modified the delay after turning. Clearly, adding 1.1 seconds of turn was a little bit too much as by the time that the robot had returned to its origin, it was slightly more tilted than it was originally. With more time, I would have been able to find a slightly better sweet spoot in the 1 to 1.1 second range such that the robot could always have a much more consistent path that takes into account friction anad other real world barriers. I would also like to try and implement this same behavior but by using odometry, as I would like to start experimenting with the Turtlebot's motion sensing capabilities and seeing the difference in results from the two methodologies.

## Takeaways

 

 - Twists are powerful message broadcasters. By just changing the linear path and the rate of turn, we can already tell the Turtlebot to do very interesting movements that are very versatile. However, we also have to take into account the nosie that the real world generates which can be somewhat unpredictable, and requires some experimentation and trial-and-error in order to narrow down.
 - By combining for-loops and the rospy.sleep() commands, we can have very basic, yet powerful tools that allow for constant repetition of basic motions in order for us to accomplish what we want. Despite me only have three Twists and doing those motions three times, the sleep command allows me to change the angle of the turn and how long I want the robot to go straight, while the loops allow me to control precisly how many times I want the robot to repeat these tasks. Combined, this means I can make a bigger or a smaller square, or a path that's even a different shape. With very few lines of code, I can make quite large changes to the robot's movement behavior

# Person Follower

Coming soon!

# Wall Follower


Coming soon!
