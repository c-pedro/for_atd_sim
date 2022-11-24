# Formation Attitude Simulator (for_atd_sim)

These simulations consider the three vehicle constrained formation where two of the vehicles cannot measure their relative direction.
Code used to obtain the simulations for the attitude observers and deterministic solutions in different papers.

## Observers Available
- `obs_con_ine` Constraint based attiude observer;
- `obs_atd_ine` Reconstruction based attiude observer;
- `obs_detatd_inerel` Deterministic Attitude Solution;


## How to run simulations

0. Assumptions
    - Python 3 is installed 
    - Python location is in the system path
    - terminal in `for_atd_sim`

1. Install requirements  
```bash pre_script.sh```

2. Edit scripts in the `scripts` directory   

    - increase `n_trials` to improve significance 
        - LARGE TIME
        - LARGE DATA
    
3. Run scripts in the `scripts` directory  

    - Convergence Analysis (with time)  
    ```python3 scripts/taes_script_mc_time.py```

    - Convergence Analysis (with error)  
    ```python3 scripts/taes_script_mc_conv.py```

    - Performance Analysis  
    ```python3 scripts/taes_script_mc_performance.py```

    - Special Cases Analysis  
    ```python3 scripts/taes_script_special.py```

4. Analyze results in output folder of each subpackage

    - Convergence Analysis (with time)  
    data: ```obs_con_ine/output/s00200```

    - Convergence Analysis (with error)  
    data (first): 
    ```obs_con_ine/output/s00001```  
    data (last):
    ```obs_con_ine/output/s00040```  
    plot_1:
    ```obs_con_ine/output/convergence-error-anl.png```  
    plot_2:
    ```obs_con_ine/output/convergence-percentage-anl.png```

    - Performance Analysis  
    data (Reconstruction based): 
    ```obs_atd_ine/output/s00100```  
    data (Constraint based): 
    ```obs_con_ine/output/s00100```  
    data (Deterministic): 
    ```obs_detatd_inerel/output/s00100```  
    plot_1:
    ```obs_con_ine/output/performance-anl.png```  

    - Special Cases Analysis  
    data (Degenerate 1): 
    ```obs_con_ine/output/s00300```  
    data (Degenerate 2): 
    ```obs_con_ine/output/s00301```  
    data (Ambiguous):
    ```obs_con_ine/output/s000302```  
    plot (Degenerate 1):   
    ```obs_con_ine/output/s000300/figs/s00300_g00000_m00000-anl-obs_atderr_angle_value.png```  
    plot (Degenerate 2):   
    ```obs_con_ine/output/s000301/figs/s00301_g00000_m00000-anl-obs_atderr_angle_value.png```  
    plot (Ambiguous):   
    ```obs_con_ine/output/s000302/figs/s00302_g00000_m00000-anl-obs_atderr_angle_value.png```