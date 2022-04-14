
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

Making a robot follow a person was a more challenging task. Utilizing the sensors was a little bit more tricky, and therefore I had to learn new things in the process as well. 

## Code Explanation

While in drive in a square I had three specific functions that did the same thing, for person follower I had to make my code a little more maleable. What I mean by this is that the robot now has to adapt to the exact position where the closest moving object (the person) is. I accomplished this by analyzing all the 360 distance values that the LIDAR sensor outputs, and finding the minimum value from those. With that, I have the angle in which the robot should rotate towards, and it can therefore spin in that direction. If the robot sees that the person is directly in front of it and at a small distance of half a meter, then the robot will stop. If the person increases their distance, the robot will start moving again to keep following them. 


## GIF

![Following Person GIF](DrivingSquare.gif)

## Challenges

Compared to Drive in a Square, I had to change how I viewed how to control my Turtlebot in order for it to work properly. The biggest challenge was to try and make my movements not be "lagged". I eventually figured out that htis was happening because I was using rospy.sleep() like in Drive in a Square. However, with the constant adjustments that the Turtlebot would have to make in order to follow a person accurately, using sleep would create too much of a lag as the queue of objects would continuously increase and the Turtlebot would never be reacting properly to what it was scanning. I therefore had to find a way to control the Turtlebot without using sleep, which was initially challenging, but I eventually got the hang of!

## Future Work

In the future, I would like to make the robot a little bit more reactive to me turning, as right now there is some lag in me moving and the Turtlebot actually following me. I also think it would be really cool to use the camera and OpenCV to potentially add some kind of person tracking, where the robot only follows a specific person! It would add a sense of camaraderie between the robot and its user. 

## Takeaways

 - Delays are sometimes not the answer, as they can severely hamper the robot's ability to synchronize its movements with its sensing capabilities. This is particularly true with something like the LIDAR sensor which is updating at a fairly high frequency.
 - If-Conditions can be powerful in controlling logic for the Turtlebot. 


# Wall Follower


The wall follower was an extremely complex project that I quite honestly struggled with for a while, until I implemented the delta error which allowed the robot to always be at a 90 degree angle in comparison to the wall. This allowed the robot to have proportional control and keep a constant distance to the wall. 

## Code Explanation

For the wall follower, I used similar code to that of the person follower, where I am checking all of the LIDAR's distance values and finding the minimum one. If the robot has no wall right in front of it, it will move forwards.  Else, if it's angle to the minimum distance to the wall is not 90 degrees, it will rotate until it is. This allows the robot to alwas have a proportional distance to the wall while also being able to spin and follow the wall properly. 


## GIF

![Following Wall GIF](DrivingSquare.gif)

## Challenges

It took me quite a while to realize how I should be using the error. Then, it took me a little bit longer to adjust the error values properly and optimize the number of if-conditions that I needed in order for the robot to work properly (it turned out to be zero, I was stuck in thinking that I actually needed to utilize if conditions in order to add the logic that I needed when I really didn't have to). This was a lot of trial and error, and did take quite a long time. I got it to work eventually though. 

## Future Work

I think in the future, I would like the Turtlebot to move in both clockwise and anti-clockwise directiosn across the wall. This would allow us much more control and flexibility with how we control the robot!

## Takeaways

 - Error deltas are a good and powerful way to allow us to control the robot and maintain proportionality, partiuclarly when taking into account things such as proper spin control. 
 - Small changes to conditions, such as distance or even if-parameters, can have a very large change in how the Turtlebot reacts. 
 - If-conditions are not always the solution. Sometimes having a simpler loop can actually be better!

