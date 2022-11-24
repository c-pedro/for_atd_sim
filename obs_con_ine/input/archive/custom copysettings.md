# Simulation Settings

## Input

```python
sett-data.file-path
'input\\input.md'
```

## Simulation

```object
sett-time.initial-value
0
```

```object
sett-time.final-value
40
```

```object
sett-time.step-value
0.5
```

## Output

### Simulation results

#### Path

```python
sett-results.file-path
'output'
```

#### Keep old data

```python
sett-results.clear-option
True
```

#### Settings Name

```python
sett-settings.filename-option
'sim-settings'
```

```python
sett-results.filename-option
'sim'
```

## Output Commands

```python
sett-command.default-option
[
    (
        'plot',
        {
            'data' : [['phi_observer_value', (1,0)]],
            'figsize' : (6,6)
        }
    ),
    (
        'plot',
        {
            'data' : [
                ['anv_problem_value', (2,0)],
                ['anv_observer_value', (2,0)]
                ],
            'figsize' : (6,6)
        }
    ),
    (
        'transform',
        {
            'type' : 'error_magnitude',
            'data' : [
                ['anv_problem_value', (2,0)],
                ['anv_observer_value', (2,0)]
                ],
            'var_key' : 'test_value'
        }
    ),
    (
        'plot',
        {
            'data' : [['test_value', (2,0)]],
            'dataType' : 'float'
        }
    )
]
```
