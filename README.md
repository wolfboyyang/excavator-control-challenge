Control Excavator with Inverse Kinematics
==========================

[excavator simulator in browser](https://rwth-crmasters-sose22.gitlab.io/course-material/excavator-simulator/simple.html)

[excavator simulator repo](https://gitlab.com/rwth-crmasters-sose22/course-material/excavator-simulator)
## Inverse Kinematics
- [Miro Board](https://miro.com/app/board/uXjVOtSx7M4=/?share_link_id=75027058547)
- Joints
![](/img/excavator_joints.webp)
- Calulate q0 in XY plane
```
As p3.y is not 0, (0.02), to simplify the solution, let move p3.y to p2.y as mq2 is make p3 rotate around y axis, 

With this operation, j2/j3/j4 are in the same line in global xy plane

angle_j0_j2_j0_base_0 is fixed for every given q1

rotate j0_j2_0 by -aj0 to j0_j4_0 is direction of j0_j4 when q0=0

when j0_j4_0 X j0_j4 < 0, q0 = -acos(j0_j4*j0_j4_0/(a*c))
when j0_j4_0 X j0_j4 > 0, q0 = acos(j0_j4*j0_j4_0/(a*c))
```
![](/img/xy_plane.webp)
![](/img/ik_q0.webp)
- Calulate q2/q3 in j2/j3/j4 arm plane
```
Law of Cosines
c^2 = a^2 + b^2 - 2 * a * b * cos(a3)

Law of Sines
c/sin(a3) = a/sin(a4) = b/sin(a2)

mimic q2 = q2 * multiplier + offset

for IK
a2_0 + mq2 = a2

a3_0 + mq3 = a3
```
![](/img/arm_plane.webp)
![](/img/ik_q2_q3.webp)
