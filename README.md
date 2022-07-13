Control Excavator with Inverse Kinematics
==========================

[excavator simulator in browser](https://rwth-crmasters-sose22.gitlab.io/course-material/excavator-simulator/simple.html)

[excavator simulator repo](https://gitlab.com/rwth-crmasters-sose22/course-material/excavator-simulator)
## Dashboard Screenshot
![](/img/dashboard.webp)

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

## Code for Inverse Kinematics Node
On Setup Tab:
- add mathjs as math
- add clamp as clamp
- change output to 5
```js
const rot_z =  [[0, -1, 0],
                [1,  0, 0],
                [0,  0, 1]]

const p0 = [0.68, -0.63, 0.38]
const p10 = [0.3, -0.31, 0.08]
const p11 = [-0.142, 0.107, 0.02]
const p1 = math.chain(rot_z).multiply(p11).add(p10).done();
// add p3.y to p2.y to make j2,j3,j4 in line when projected to global xy plane
const p2 = [-0.17, -0.08 + 0.02, 0.09] // origin p2 [-0.17, -0.08, 0.09]
const p3 = [-1.4, 0.02 - 0.02, 1.06] // origin p3 [-1.4, 0.02, 1.06]
const p4 = [0.05, 0, -1.14]

const mq2_range = [-0.4517, 0.46378]
const mq3_range = [-0.217, 1.910777]


tx = flow.get('x')
ty = flow.get('y')
tz = flow.get('z')
j4 = [tx, ty, tz]

function length_of_vector(v) {
    if (math.count(v) == 2)
        return math.distance(v, [0, 0])
    else 
        return math.distance(v, [0, 0, 0])
}
function angle_of_vector(v1, v2){
    return math.evaluate(`acos(${math.multiply(v1,v2)}/(${length_of_vector(v1)}*${length_of_vector(v2)}))`)
}
function rotate_axis_z(angle) {
    return [[math.cos(angle), -math.sin(angle), 0],
            [math.sin(angle),  math.cos(angle), 0],
            [              0,                0, 1]]
}
function rotate_axis_y(angle) {
    return [[ math.cos(angle), 0, math.sin(angle)],
            [            0, 1,            0],
            [-math.sin(angle), 0, math.cos(angle)]]
}
q1 = flow.get('q1')
if(!q1) q1 = 0

// base_link <- Kabine
j0 = p0

// caluclate q0 in xy plane
// regardless q0, aj2 and distance of j2 to j0 is fixed 
// aj2 is the angle between -x axis at j2 with j2_j0

// q1 - Kabine <- Bagger_Verbindung_Arm
//             <- Arm1_zu_BaggerVerbindungArm2
// in j0 local space (no effect by q0)
mq1 = rotate_axis_z(q1)
m1_j0 = math.chain(rot_z).multiply(mq1).done()
j0_j2_j0 = math.chain(m1_j0).multiply(p2).add(p1).done()
j0_j2_j0_xy = math.subset(j0_j2_j0, math.index([0,1]))

axis_j4_j2_j0_xy = math.chain(m1_j0).multiply([1,0,0])
        .subset(math.index([0,1])).done()
aj2 = angle_of_vector(j0_j2_j0_xy, axis_j4_j2_j0_xy)
a = length_of_vector(j0_j2_j0_xy)

// back to global word (from base_link)
j0_j4_xy = math.chain(j4).subtract(j0)
        .subset(math.index([0,1])).done()
c = length_of_vector(j0_j4_xy)
aj4 = math.evaluate(`asin(sin(${aj2})*${a}/${c})`)
aj0 = math.evaluate(`pi - ${aj2} -${aj4}`)

// when q0 = 0
j0_j4_0_xy = math.chain(rot_z).multiply(rotate_axis_z(-aj0))
        .multiply(j0_j2_j0)
        .subset(math.index([0,1])).done()
sign = math.sign(math.det([j0_j4_0_xy, j0_j4_xy]))
q0 = sign* angle_of_vector(j0_j4_0_xy, j0_j4_xy)

// q0 is done
// update j1,j2

// q1 - Kabine <- Bagger_Verbindung_Arm
//             <- Arm1_zu_BaggerVerbindungArm2
mq0 = rotate_axis_z(q0)
m0 = math.chain(rot_z).multiply(mq0).done()
j1 = math.chain(m0).multiply(p1).add(j0).done()

mq1 = rotate_axis_z(q1)
m1 = math.chain(m0).multiply(rot_z).multiply(mq1).done()
j2 = math.chain(m1).multiply(p2).add(j1).done()

// calculate q2, q3 in j2,j4, axis z plane
// mimic q2 - Arm1_zu_BaggerVerbindungArm2 <- Arm1
multiplier = -4.5774
offset = -0.4517

// when mq2 = 0
j2_p3_0 = math.chain(m1).multiply(p3).done()
j2_j4 = math.subtract(j4, j2)
a2_0 = angle_of_vector(j2_j4,j2_p3_0)

// for q2
// Law of Cosines
// c^2 = a^2 + b^2 - 2 * a * b * cos(a3)
a = length_of_vector(p3)
b = length_of_vector(p4)
c = math.distance(j4, j2)

a3 = math.evaluate(`acos((${a}^2+${b}^2-${c}^2)/(2*${a}*${b}))`)
// Law of Sines
// c/sin(a3) = a/sin(a4) = b/sin(a2)
a2 = math.evaluate(`asin(sin(${a3})*${b}/${c})`)

j2_j4 = math.subtract(j4, j2)
// in local space of j2
// j2_p3_v_j2 || mmq2 * p3
m1_trans = math.transpose(m1)
j2_p3_v_j2 = math.chain(rotate_axis_y(a2)).multiply(m1_trans)
        .multiply(j2_j4).done()
sign = math.sign(math.chain(p3).cross(j2_p3_v_j2).multiply([0,1,0]).done())
mq2 = sign * angle_of_vector(p3, j2_p3_v_j2)

mq2 = clamp(mq2, mq2_range[0], mq2_range[1])
// For forward kinematic, q2 = q2 * multiplier + offset
// For inverse kinematic
q2 = (mq2 - offset) / multiplier

mmq2 = rotate_axis_y(mq2)
m2 = math.chain(m1).multiply(mmq2).done();

j3 = math.chain(m2).multiply(p3).add(j2).done()

// mimic q3 - Arm1 <- Arm2
multiplier = -5.1897
offset = -0.217

// calculate q3
// when mq3 = 0
j3_j4 = math.subtract(j4, j3)
// in local space of j4
// j3_p4_v_j3 || mmq3 * p4
m2_trans = math.transpose(m2)
j3_p4_v_j3 = math.multiply(m2_trans, j3_j4)
sign = math.sign(math.chain(p4).cross(j3_p4_v_j3).multiply([0,1,0]).done())
mq3 = sign * angle_of_vector(p4, j3_p4_v_j3)

mq3 = clamp(mq3, mq3_range[0], mq3_range[1])
// For forward kinematic, mq3 = q3 * multiplier + offset
// For inverse kinematic

q3 = (mq3 - offset) / multiplier

mmq3 = rotate_axis_y(mq3)
m3 = math.chain(m2).multiply(mmq3).done();

j4 = math.chain(m3).multiply(p4).add(j3).done()
// Arm2 <- Loeffel
// target is j4 where Loeffel attached 

dp = j4

flow.set('dx', dp[0])
flow.set('dy', dp[1])
flow.set('dz', dp[2])

outputMsgs = []
outputMsgs.push({payload: q0})
outputMsgs.push({payload: q1})
outputMsgs.push({payload: q2})
outputMsgs.push({payload: q3})

return outputMsgs;
```
