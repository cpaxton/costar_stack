# Librarian

Provide simple interface for managing files used by CoSTAR programs.

## File System

- **Types:** high level categories; use these to group definitions of different items/variables used in a CoSTAR workspace.
- **Items:** items are represented as files, and can be saved/loaded as ROS parameters.

## Using Librarian

### Launch Files

The `librarian_bringup` package contains launch files for Librarian. As such, you can launch the librarian core easily with:

```
roslaunch librarian_bringup core.launch
```

To change the location of the folder Librarian stores and reads files from, simply change the `librarian_root` parameter:

```
roslaunch librarian_bringup core.launch librarian_root:=~/.costar/
```

### Running Librarian

Start `librarian_core` with the directory you want to use for saving/loading files.

### Provided Services

## Troubleshooting

### Contact

Librarian is maintained by Chris Paxton (cpaxton3@jhu.edu)
