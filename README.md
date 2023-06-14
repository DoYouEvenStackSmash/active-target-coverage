# Active Object Tracking and Target Coverage

## Interactive (mouse driven) demos
These are some fun interactive tests to exhibit the tracker behaviors.  The higher the sample rate, the more accurate the tracker will be.
* Active tracking with an agent following user mouse at sample rate 1/20.  
 ```./tests/interactive-sensing-agent.py 20```

* Static tracking with an agent predicting location of user mouse at sample rate 1/20:  
 ```./tests/static-interactive-sensing-agent.py 20```

## Repeatable dynamic examples
These are some programs for running repeatable examples, which are useful for tuning parameters.  
* Active tracking with an agent following a target traveling along specified track at every time step. Useful for sanity checking.
```./tests/tracking-sensing-agent-test.py point-fields/loop.json```

* Active tracking with an agent following a target traveling along a specified track, with a user specified sample rate.  
```./tests/active-tracking-agent.py 20```

## Agent and/or target examples
Some examples for just rendering agent observations or target behaviors.  
* Agentless environment with a single target following its predefined path. Useful for debugging the generated tracks.  `LALT` to translate, `LSHIFT` to rotate.   
 ```./tests/target-travel-test.py point-fields/loop.json```

* Agent with a target track, showing visibility vs detections. Useful for debugging the agent coordinate system.  
 ```./detecting-sensing-agent-test.py point-fields/grid.json```

## Generating a new track
A convenience program for generating some predefined tracks. Not clean code, but it rewards a skilled user.   
* Generate a path with two loops which looks a bit like a bow tie, with a noise factor of 10.  
```./point-fields/path-generator.py LOOP 10```
* Generate a cubic spline which resembles a sine wave with noise factor of 0.  
 ```./point-fields/path-generator.py LERP 0```


