# Data Input

Basic Problem Configuration

## Vehicle Momement of Inertia

```object
pro_parameter_value (0,1,0)
70 0 0
0 70 0
0 0 60
```

```object
pro_parameter_value (0,2,0)
70 0 0
0 70 0
0 0 60
```

```object
pro_parameter_value (0,3,0)
70 0 0
0 70 0
0 0 60
```

## External Torque

```object
pro_parameter_value (1,1,0)
0
0
0
```

```object
pro_parameter_value (1,2,0)
0
0
0
```

```object
pro_parameter_value (1,3,0)
0
0
0
```

## Attitude Initial Value

The attitude input is given by the inertial attitude of each vehicle.
All the relative and inverse relations are computed from that set automatically.

**Important details:**

- inertial attitudes (from body to inertial frame)

$\mathbf{R}^{\mathrm{I}}_{1}$

```python
pro_atd_value (0,1)
numpy.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
```

$\mathbf{R}^{\mathrm{I}}_{2}$

```object
pro_atd_value (0,2)
1 0 0
0 1 0
0 0 1
```

$\mathbf{R}^{\mathrm{I}}_{3}$

```object
pro_atd_value (0,3)
1 0 0
0 1 0
0 0 1
```

***

## Angular Velocity Initial Value

The angular velocity is inputed as a vector per vehicle. Each gives the variation of the attitude of the corresponding vehicle relative to the inertial frame.

**Important details:**

- assumed units are [N.m/s]
- in the body frame

$\mathbf{\omega}_{1}$

```object
pro_anv_value (1,0)
0.1
0.1
0.1
```

$\mathbf{\omega}_{2}$

```object
pro_anv_value (2,0)
0.1
0.1
0.1
```

$\mathbf{\omega}_{3}$

```object
pro_anv_value (3,0)
0.1
0.1
0.1
```

***

## References Initial Value

References are given in the inertial frame. The ground truth of the measurements are computed using the attitudes that of the formation.
The vectors need not be unit vectors in this file, but in practice as soon as they are entered into the program they are normalized into unit vectors.

**Important details:**

- can be non unitary (program normalizes them)
- measurments are determined by the attitude
- in the inertial frame

$\mathbf{i}_{1}$

```object
pro_ref_value (1,0)
1
0
0
```

- $\mathbf{i}_{2}$

```object
pro_ref_value (2,0)
0
1
0
```

- $\mathbf{i}_{3}$

```object
pro_ref_value (3,0)
0
1
0
```

***

## Line of Sight Initial Values

One vector per relation between two vehicles is given in the inertial frame. The measurements' ground truth is computed automatically.

**Important details:**

- given in the inertial frame
- one vector per edge of communication graph

$\mathbf{i}_{1/2}$

```object
pro_ilos_value (1,2)
0
1
1
```

$\mathbf{i}_{1/3}$

```object
pro_ilos_value (1,3)
0
0
1
```

## Sensor Characteristics

This section contains the characteristics of the formation sensors.

### Standard Deviation

#### Angular Velocity Sensors

$\mathbf{\omega}_{1}$

```object
sen_anv_sdev (1,0)
4*10**-5
```

$\mathbf{\omega}_{2}$

```object
sen_anv_sdev (2,0)
4*10**-5
```

$\mathbf{\omega}_{3}$

```object
sen_anv_sdev (3,0)
4*10**-5
```

#### Line-of-Sight Sensors

$\mathbf{d}_{1/2}$

```object
sen_los_sdev (1,2)
17*10**-6
```

$\mathbf{d}_{2/1}$

```object
sen_los_sdev (2,1)
17*10**-6
```

$\mathbf{d}_{1/3}$

```object
sen_los_sdev (1,3)
17*10**-6
```

$\mathbf{d}_{3/1}$

```object
sen_los_sdev (3,1)
17*10**-6
```

#### Reference Sensors

$\mathbf{d}_{1}$

```object
sen_ref_sdev (1,1)
17*10**-6
```

$\mathbf{d}_{2}$

```object
sen_ref_sdev (2,2)
17*10**-6
```

$\mathbf{d}_{3}$

```object
sen_ref_sdev (3,3)
17*10**-6
```

***

## Observer characteristics and initial values

### Initial Attitude Estimate

$\mathbf{R}^{\mathrm{I}}_{1}$

```python
obs_atd_value (0,1)
rot_mtx(numpy.pi/2, numpy.array([[0],[1],[0]]))
```

$\mathbf{R}^{\mathrm{I}}_{2}$

```python
obs_atd_value (0,2)
rot_mtx(3*numpy.pi/4, numpy.array([[0],[1],[0]]))
```

$\mathbf{R}^{\mathrm{I}}_{3}$

```python
obs_atd_value (0,3)
numpy.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
```

### Initial Angular Velocity Estimate

**Important details:**

- assumed units are [N.m/s]
- in the body frame

$\mathbf{\omega}_{1}$

```object
obs_anv_value (1,0)
0
0
0
```

$\mathbf{\omega}_{2}$

```object
obs_anv_value (2,0)
0
0
0
```

$\mathbf{\omega}_{3}$

```object
obs_anv_value (3,0)
0.1
0.1
0.1
```

### Potential terms weight

```object
obs_wpotential_value (0)
1 1 1 1
1 1 1 1
1 1 1 1
1 1 1 1
```

### Kinetic terms weight

```object
obs_wkinetic_value (0)
0 0 0 0
0 1 0 0
0 0 1 0
0 0 0 1
```

### Observer Damping

```object
obs_damping_value (1,1)
1 0 0
0 1 0
0 0 1
```

```object
obs_damping_value (2,2)
1 0 0
0 1 0
0 0 1
```

```object
obs_damping_value (3,3)
1 0 0
0 1 0
0 0 1
```

## Maneuvers

```python
pro_ilos_maneuver (1,2)
[
    [
        0,
        100,
        1*numpy.pi/2,
        (numpy.array([[0],[1],[0]]) + numpy.array([[0],[1/2**0.5],[1/2**0.5]])) / numpy.linalg.norm(numpy.array([[0],[1],[0]]) + numpy.array([[0],[1/2**0.5],[1/2**0.5]]))
    ],
    [
        100,
        200,
        0*numpy.pi,
        (numpy.array([[0],[0],[1]]) + numpy.array([[0],[1],[0]])) / numpy.linalg.norm(numpy.array([[0],[0],[1]]) + numpy.array([[0],[1],[0]]))
    ],
]
```

```python
pro_ilos_maneuver (1,3)
[[0, 1, 0, [[0],[0],[1]]]]
```

## Data Keys

This section of the file establishes the names given to the data fields inside the program. If you do not  use the default values, change this section appropriately.

### Problem Keys

```python
kanv
'mykey'
```
