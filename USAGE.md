# Usage Documentation
## Usage

### Basic Workflow

1. **Load Point Cloud Sequence**:
   ```
   pclLoaderCmd -mainPath "path/to/sequence/frame_####.ply" -name "myLoader";
   ```

2. **Create Raycaster** (optional):
   ```
   pclRaycasterCmd -name "myRaycaster" -pclLoader "myLoader" -selCamera "camera1";
   ```

3. **Create Cluster** (optional):
   ```
   pclClusteringCmd -name "myCluster" -pclLoader "myLoader";
   ```

4. **Create Sequencer** (optional):
   ```
   pclSequencerCmd -name "mySequencer" -pclLoader "myLoader";
   ```
------------------------------------------------------------------------

### Command Reference

#### `pclLoaderCmd`
Main command for creating the loader node.

**Flags**:
- `-mainPath` / `-mP`: Path to point cloud file or sequence
- `-name` / `-n`: Name for the loader node
- `-pclRaycaster` / `-pr`: Create and connect raycaster
- `-pclCluster` / `-pc`: Create and connect cluster
- `-pclSequencer` / `-ps`: Create and connect sequencer
- `-selCamera` / `-sc`: Camera for raycaster (if creating raycaster)

**Example**:
```
pclLoaderCmd -mainPath "C:/data/points/frame_####.ply" -name "loader1" -pclRaycaster -selCamera "persp";
```

#### `pclRaycasterCmd`
Creates a raycaster node connected to a loader.

**Flags**:
- `-name` / `-n`: Name for the raycaster node
- `-pclLoader` / `-pl`: Name of the pclLoader node to connect to

#### `pclClusteringCmd`
Creates a cluster node connected to a loader.

**Flags**:
- `-name` / `-n`: Name for the cluster node
- `-pclLoader` / `-pl`: Name of the pclLoader node to connect to

#### `pclSequencerCmd`
Creates a sequencer node connected to a loader.

**Flags**:
- `-name` / `-n`: Name for the sequencer node
- `-pclLoader` / `-pl`: Name of the pclLoader node to connect to
   
------------------------------------------------------------------------

### Node Attributes

#### pclLoader Node
- **mainPath**: Path to point cloud file or sequence
- **loadFile**: Enable/disable file loading (0=off, 1=on)
- **loadStartFrame** / **loadEndFrame**: Frame range to load
- **skipFilesRows**: Downsampling stride (0 = no downsampling)
- **containerOnOff**: Container filtering mode (0=off, 2=inside, 3=outside)
- **inContainerMatrix**: Transform matrix for container positioning
- **timeOffset**: Current frame offset for timeline playback
- **displayPoints**: Show/hide points in viewport
- **pointsSize**: Point size for viewport display

#### pclRaycaster Node
- **inTime**: Time input (connect to time1.outTime)
- **cameraMatrix**: Camera to use for raycasting
- **raycastOnOff**: Enable raycasting (0=off, 1=raycast, 2=frustum)
- **raycastInputX** / **raycastInputY**: Screen coordinates for raycast
- **displayType**: Display mode (0=all, 1=raycast hits, 2=outside raycast)
- **minRadius**: Minimum radius for raycast hit detection

#### pclCluster Node
- **action**: Enable clustering (0=off, 1=on)
- **recursionDepth**: Octree subdivision depth
- **minimumPointCount**: Minimum points per cluster
- **clustersGrouping**: Enable cluster grouping
- **createClusterPath**: Track cluster trajectories
- **createCurvs**: Generate NURBS curves from trajectories
 
------------------------------------------------------------------------ 
### Path Patterns

The loader supports several path patterns:
(Examples work on both *.ply and *.pcl files)
1. **Single File**:
   ```
   C:/data/points/frame.ply
   ```

2. **Numbered Sequence** (using `#`):
   ```
   C:/data/points/frame_####.ply
   ```
   Matches: `frame_0001.ply`, `frame_0002.ply`, etc.

3. **Folder with Multiple Files**:
   ```
   C:/data/points/folder/
   ```
   Loads all `.ply` and `.pcd` files in the folder

4. **Sequence Folder**:
   ```
   C:/data/points/frame_####/points.ply
   ```
   Matches numbered folders containing point files