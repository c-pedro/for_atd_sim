# Data Input

Basic Problem Configuration

## Vehicle Momement of Inertia

```object
pro_moi_value (0,1)
70 0 0
0 70 0
0 0 60
```

```object
pro_moi_value (0,2)
70 0 0
0 70 0
0 0 60
```

```object
pro_moi_value (0,3)
70 0 0
0 70 0
0 0 60
```

## External Torque

object
torque_function_initial (0,1)
0.01
0
0


object
torque_function_initial (0,2)
0
0.01
0


object
torque_function_initial (0,3)
0
0
0.01



```python
torque_function
'torque_sinusoid'#'torque_static'
```

```python 
torque_function_gain (0,1)
numpy.pi/180
```

```python 
torque_function_gain (0,2)
numpy.pi/180
```

```python 
torque_function_gain (0,3)
numpy.pi/180
```

```python 
torque_function_frequency (0,1)
1
```

```python 
torque_function_frequency (0,2)
1
```

```python
torque_function_frequency (0,3)
1
```

```python 
torque_function_phase (0,1)
numpy.array([[0]])
```

```python 
torque_function_phase (0,2)
numpy.array([[0]])
```

```python 
torque_function_phase (0,3)
numpy.array([[0]])
```



## Attitude Initial Value

The attitude input is given by the inertial attitude of each vehicle.
All the relative and inverse relations are computed from that set automatically.

**Important details:**

- inertial attitudes (from body to inertial frame)

$\mathbf{R}^{\mathrm{I}}_{1}$

```object
pro_atd_value (0,1)
1 0 0
0 1 0
0 0 1
```
<!-- numpy.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) -->

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
0
0
0
```

$\mathbf{\omega}_{2}$

```object
pro_anv_value (2,0)
0
0
0
```

$\mathbf{\omega}_{3}$

```object
pro_anv_value (3,0)
0
0
0
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
1
0
```

- $\mathbf{i}_{2}$

```object
pro_ref_value (2,0)
-1
1
0
```

- $\mathbf{i}_{3}$

```object
pro_ref_value (3,0)
0
0
1
```

***

## Line of Sight Initial Values

One vector per relation between two vehicles is given in the inertial frame. The measurements' ground truth is computed automatically.

**Important details:**

_ given in the inertial frame
_ one vector per edge of communication graph

$\mathbf{i}_{1/2}$

```object
pro_ilos_value (1,2)
1
0
0
```

$\mathbf{i}_{1/3}$

```object
pro_ilos_value (1,3)
1
0
1
```

## Sensor Characteristics

This section contains the characteristics of the formation sensors.

### Standard Deviation

#### Angular Velocity Sensors

$\mathbf{\omega}_{1}$

```python
sen_anv_sdev (1,0)
5*10**-6
```

$\mathbf{\omega}_{2}$

```python
sen_anv_sdev (2,0)
5*10**-6
```

$\mathbf{\omega}_{3}$

```python
sen_anv_sdev (3,0)
5*10**-6
```

#### Line_of_Sight Sensors

$\mathbf{d}_{1/2}$

```python
sen_los_sdev (1,2)
80*10**-6
```

$\mathbf{d}_{2/1}$

```python
sen_los_sdev (2,1)
80*10**-6
```

$\mathbf{d}_{1/3}$

```python
sen_los_sdev (1,3)
80*10**-6
```

$\mathbf{d}_{3/1}$

```python
sen_los_sdev (3,1)
80*10**-6
```

#### Reference Sensors

$\mathbf{d}_{1}$

```python
sen_ref_sdev (1,1)
80*10**-6
```

$\mathbf{d}_{2}$

```python
sen_ref_sdev (2,2)
80*10**-6
```

$\mathbf{d}_{3}$

```python
sen_ref_sdev (3,3)
80*10**-6
```

***

## Observer characteristics and initial values

### Initial Attitude Estimate

$\mathbf{R}^{\mathrm{I}}_{1}$

```python
obs_atd_value (0,1)
rot_mtx(numpy.pi/2*1, numpy.array([[0],[1],[0]]))
```

$\mathbf{R}^{\mathrm{I}}_{2}$

```python
obs_atd_value (0,2)
rot_mtx(3*numpy.pi/4*1, numpy.array([[0],[1],[0]]))
```

$\mathbf{R}^{\mathrm{I}}_{3}$

```python
obs_atd_value (0,3)
rot_mtx(numpy.pi/2*1, numpy.array([[0],[1],[0]]))
```
<!-- numpy.eye(3) -->




### Initial Angular Velocity Estimate

**Important details:**

- assumed units are [N.m/s]
- in the body frame

$\mathbf{\omega}_{1}$

```object
obs_anv_value (1,0)
1
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
0
0
0
```

### Potential terms weight

```python
obs_wpotential_value (1,1)
[
    (0, [[0.6]]),
]
```

```python
obs_wpotential_value (2,2)
[
    (0, [[0.6]]),
]
```

```python
obs_wpotential_value (3,3)
[
    (0, [[0.6]]),
]
```

```python
obs_wpotential_value (1,2)
[
    (0, [[0.6]]),
]
```

```python
obs_wpotential_value (2,1)
[
    (0, [[0.6]]),
]
```

```python
obs_wpotential_value (1,3)
[
    (0, [[0.6]]),
]
```

```python
obs_wpotential_value (3,1)
[
    (0, [[0.6]]),
]
```


### Kinetic terms weight

```python
obs_wkinetic_value (1,0)
[
    (0, [[1.5]]),
]
```

```python
obs_wkinetic_value (2,0)
[
    (0, [[1.5]]),
]
```

```python
obs_wkinetic_value (3,0)
[
    (0, [[1.5]]),
]
```



### Observer Damping

```python
obs_damping_value (1,0)
[
    (0, [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]),
]
```

```python
obs_damping_value (2,0)
[
    (0, [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]),
]
```

```python
obs_damping_value (3,0)
[
    (0, [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]),
]
```
    <!-- (70, [[1.5, 0, 0], [0, 1.5, 0], [0, 0, 1.5]]), -->


## Maneuvers

```python
pro_maneuver (1,2)
[
    [
        0,
        100,
        -numpy.pi*0,
        numpy.array([[0],[0],[1]]),
    ],    
]
```

```python
pro_maneuver (1,3)
[
    [
        0,
        100,
        numpy.pi/100,
        numpy.array([[0],[0],[1]]),
    ],
]
```


```python
pro_maneuver (3,3)
[
    [0, 100, 0, [[0],[0],[1]]],
    [100, 110, 0, [[0],[0],[1]]],
]
```
