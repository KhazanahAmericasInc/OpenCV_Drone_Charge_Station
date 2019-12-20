## Description  
An intelligent landing pad that can detect a drone flying overhead and take control to autonomously land the drone and recharge it. In this case, I am using an inexpensive indoor drone flown manually by a person before the autonomous landing procedure takes over control. In a commercial application, the drone would use other sensors to fly autonomous missions before the landing. I am focusing on just the landing portion of the problem because autonomous flight is already common and not as challenging.  

## Video  
YouTube Demo Video: https://www.youtube.com/watch?v=vQnh8SQZ9hY  

## Hardware
- Eachine E010 quadcopter 
- Arduino Uno 
- 2.4GHz nRF24L01+ 
- PlayStation Eye webcam  
- Flysky i6 transmitter 
- Flysky PPM compatible receiver  
- 12V power supply  
- 12V to 5V voltage converter  


## Credit  
- Rotation matrix to euler angles math: https://www.learnopencv.com/rotation-matrix-to-euler-angles/   
- Inspiration and PID loop framework from @partomatl: https://github.com/partomatl   
- Signal filtering from @partomatl : https://github.com/partomatl   
- Transmitter protocol from goebish on RCgroups / github: https://github.com/goebish/nrf24_multipro  
- Serial communication modification of transmitter protocol from @perrytsao:  https://github.com/perrytsao  
- The PPM library @Nikkilae: https://github.com/Nikkilae/PPM-reader  



