Source: https://github.com/MichaelGrupp/evo/wiki/Metrics (wiki repo commit 8cb784f3546eece74639b1d3dc09e78677159b72, raw from https://raw.githubusercontent.com/wiki/MichaelGrupp/evo/Metrics.md)
Accessed: 2026-09-28

These built-in command line apps let you evaluate estimated trajectories against a reference (ground truth):

* `evo_ape`
* `evo_rpe`

Conceptually, the command syntax is as follows:
```
command format reference-trajectory estimated-trajectory [options] 
```

where `format` indicates one of the supported trajectory formats - see [here](https://github.com/MichaelGrupp/evo/wiki/Formats) for more infos on the different formats.

In case of a ROS bagfile, the syntax is slightly different:
```
command bag2 bagfile-path reference-topic estimated-topic [options] 
```
TF topics are also supported. For all details see [here](https://github.com/MichaelGrupp/evo/wiki/Formats#bag--bag2---ros-bagfile).

For more details on the command line interfaces, use the `--help` flag - e.g. `evo_rpe euroc --help`. 

## Tutorials
A good start to get a feeling for the concepts behind the apps is the interactive [Jupyter notebook](http://jupyter.readthedocs.io/en/latest/install.html) `metrics_tutorial.ipynb` that you can find [here](https://github.com/MichaelGrupp/evo/tree/master/notebooks). 

Very simple Bash-script demos for the command line apps that show examples for different use cases are also provided in the repository, see [here](https://github.com/MichaelGrupp/evo/tree/master/test/demos).

## Alignment

You can use Umeyama alignment as a pre-processing step:
* `--align` or `-a` = SE(3) Umeyama alignment (rotation, translation)
* `--align --correct_scale` or `-as` = Sim(3) Umeyama alignment (rotation, translation, scale)
* `--correct_scale` or `-s` = scale alignment

Scale or Sim(3) alignment is usually required for monocular SLAM, where you usually have random scale. SE(3) alignment is useful for the absolute pose error (`evo_ape`) if you want to measure the shape similarity of trajectories as best as possible. The [alignment_demo.py](https://github.com/MichaelGrupp/evo/blob/master/examples/alignment_demo.py) script shows the different types of alignment with an example trajectory (as shown in the [`evo_traj` documentation](https://github.com/MichaelGrupp/evo/wiki/evo_traj#geometric-alignment)).

**New since v1.5.0:** 
A simple origin alignment that can be useful for drift/loop closure evaluation is available with `--align_origin`. This is not based on the Umeyama algorithm.

## Projection

3D trajectories can be projected to 2D before errors are calculated. This works [the same as in evo_traj](https://github.com/MichaelGrupp/evo/wiki/evo_traj#projection) via `--project_to_plane {xy, xz, yz}`, see the section there for details.

## Downsampling and Filtering

Data can be downsampled and motion-filtered. This works [the same as in evo_traj](https://github.com/MichaelGrupp/evo/wiki/evo_traj#downsampling-and-filtering), see the section there for details.

## `evo_ape`

Absolute pose error, often used as absolute trajectory error. Corresponding poses are directly compared between estimate and reference given a pose relation. Then, statistics for the whole trajectory are calculated. This is useful to test the global consistency of a trajectory.

Example:

```
evo_ape tum reference.txt estimate.txt --align
```

Append `--help` to the command to see all available options.

## `evo_rpe`

Instead of a direct comparison of absolute poses, the relative pose error compares motions ("pose deltas"). This metric gives insights about the local accuracy, i.e. the drift. For example, the translational or rotational drift per meter can be evaluated:

```
evo_rpe tum reference.txt estimate.txt --pose_relation angle_deg --delta 1 --delta_unit m
```

Relative pose pairs are consecutive unless `--all_pairs` is specified. Then, overlapping relative pose pairs are used.

Append `--help` to the command to see all available options.