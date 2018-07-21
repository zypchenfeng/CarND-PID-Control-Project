# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program
#### Effect of the P, I, D component
1. P is the proportional component to the cte. In my first test with I and D implemented, we can see the steering has no response when the car gradually off the track, which means the proportional isn't working. It meets my expectation. Detail video please see ./video/No_P.mp4, or https://youtu.be/yt-vYeT1By8

2.  I is the integrated error component. It is representing the systematic bias if running for a while. It is not very sensitive to the cte, because if the cte is switching between positive and negative, the add up with be cancelled. Detail video please see ./video/No_I.mp4, or https://youtu.be/48b2pi1tdMs

3. D is the proportional to the derivative of cte, so if we only have P, the response will only depends on cte, and easy to overshot. The result meets my expectation. Detail video please see ./video/No_D.mp4, or https://youtu.be/iVhX2Ec_osI

4. The project result with P/I/D optimized by twiddling can be seen at ./video/Optimized_PID.mp4, or https://youtu.be/BurLVjHbbbw

5. The twiddling screenshot can be found at ./video/PID_Twiddling.mp4, or https://youtu.be/hC8PsUy_sNI
