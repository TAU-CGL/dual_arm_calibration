
# calibration process - NOT TESTED (yet)

## handclasp - uri and ayal connected with the connector.

## 1. calibration.py
```bash
python3 uri_calibration/src/calibration.py
```

performs the following:
1. reach_grid() - forms a grid with allowed positions for handclasp
2. mount() - uri mounts the connector on the gripper
3. approach() - ayal approaches to uri, based on prior knowledge of their relative base position
4. connect() - ayal enters the connector
5. sample_loop() - for N_cycle
    - move() - both moves to a new location
    - align() - ayal alignes in the connector
    - sample() - we sample the calibration
6. solve() - (naive) mean over all samples
7. unmount() - 
    - ayal takes the connector off uri's gripper.
    - ayal hands the connector to uri.
    - uri puts the connector back in place.
8. verify() - pnp of something.

uses also:
1. jitter.py


## 2. train_controller.py
```bash
python uri_calibration/src/train_controller.py
```

- train_lqr_6x6 - force(6) -> delta_pose(6)
- train_lqr_15x3 - {force_uri(6), force_ayal(6), distance(1), manipulabilities(2)} -> delta_orientation(6)

##3. rmp_controller.py, robot_fsm.py