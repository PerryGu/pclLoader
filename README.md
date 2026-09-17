# PCL Loader Plugin Suite for Maya

A comprehensive Maya plugin suite for loading, processing, and
visualizing point cloud data (PCL/PLY files) with advanced features
including container filtering, raycasting, clustering, and particle
system integration.

## TL;DR

-   **Point Cloud Loading**: Load single files or sequences of PCL/PLY
    point cloud data.
-   **Timeline Playback**: Real-time point cloud updates as you scrub
    through the timeline.
-   **Container Filtering**: Interactive bounding box container to
    filter points (inside/outside).
-   **Raycasting**: Mark and color points visible in a specific camera
    view.
-   **Clustering**: Automatic cluster detection with trajectory tracking
    and curve generation.
-   **Particle System Integration**: Display point data to Maya's
    nParticle system.
-   **Multithreaded Loading**: Parallel file loading prevents Maya UI
    freezing.

------------------------------------------------------------------------

## Demo

## 🎬 Video Showcase

<table>
  <tr>
    <td align="center"><strong>pclLoader</strong></td>
    <td align="center"><strong>pclCluster</strong></td>
    <td align="center"><strong>pclRaycaster</strong></td>
  </tr>
  <tr>
    <td align="center">
      <a href="https://www.youtube.com/watch?v=hNdi2yLKMfU">
        <img src="media/load_point_cloud_sequences.png" width="300px">
      </a>
    </td>
    <td align="center">
      <a href="https://www.youtube.com/watch?v=tLRw0yPXXZ8">
        <img src="media/display_particles_type.png" width="300px">
      </a>
    </td>
    <td align="center">
      <a href="https://www.youtube.com/watch?v=ksgL64EMrjg">
        <img src="media/raycaster_from_cam_center.png" width="300px">
      </a>
    </td>
  </tr>
</table>





---

## 🖼 Image Gallery

pclLoader
<table> <tr> <td><img src="media/clustering_step1.png" height="220px"></td> <td><img src="media/clustering_step2.png" height="220px"></td> <td><img src="media/clustering_step3.png" height="220px"></td> </tr> </table>

pclCluster
<table> <tr> <td><img src="media/clustering_step4.png" height="220px"></td> <td><img src="media/display_particles_type.png" height="220px"></td> <td><img src="media/load_point_cloud_sequences.png" height="220px"></td> </tr> </table>

pclRaycaster
<table> <tr> <td><img src="media/raycaster_from_cam_center.png" height="220px"></td> <td><img src="media/raycaster_from_cam_pixel_pos.png" height="220px"></td> <td><img src="media/only_points_inside_container.png" height="220px"></td> </tr> </table>

------------------------------------------------------------------------


## Overview

This plugin suite provides a high-performance solution for working with point-cloud sequences inside Maya. 
The project was inspired by advanced data-processing workflows I developed during my tenure as a CG Engineer at Intel,
where handling large-scale datasets and real-time playback was critical.

Rebuilt entirely from scratch as a personal project, this suite implements those core concepts into a generalized, modular environment. 
It supports **large-scale PLY/PCD datasets, frame-by-frame timeline playback, and interactive filtering**, 
while enabling advanced features like camera-based 2D to 3D raycasting and spatial clustering trajectories within Maya's Dependency Graph.

The system follows a modular architecture: every feature is implemented
as an independent Maya node. Each node focuses on a single
responsibility while receiving its input from the central pclLoader node
via Maya's dependency graph, keeping the workflow efficient, stable, and
easy to extend.

This modular design allows each component to evolve independently and
makes the suite maintainable and scalable.

------------------------------------------------------------------------

# Node Breakdown (High-Level Responsibilities)

## pclLoader - The Central Data-Source Node

The main node that loads and caches point cloud sequences.

### Features:

-   Multithreaded file loading for performance (prevents UI freezing and
    dramatically speeds up heavy file loading).
-   Row skipping point downsampling via `skipRows` parameter
    (downsamples extremely heavy clouds and accelerates loading).
-   Supports single files, folders, and numbered sequences (with `#`
    pattern).
-   Container-based point filtering (show inside/outside).
-   Real-time frame updates based on timeline.

------------------------------------------------------------------------

## pclSequencer - Point-Cloud Display (nParticles)

Visualizes point-cloud frames in Maya.

### Features:

-   Receives positions/colors from pclLoader.
-   Connects to nParticle emitter and nucleus.
-   Enables smooth real-time playback using Maya's GPU nParticles
    system.
-   Save point data to PCD/PLY files.

------------------------------------------------------------------------

## pclRaycaster - Camera-Based Raycasting

Marks points visible in a connected camera.

### Features:

-   Raycast from 2D screen coordinates to 3D point cloud (shoots rays
    through pixel coordinates from any selected Maya camera).
-   Finds the closest hit within a configurable radius and returns the
    closest hit point.
-   Highlights hit points for visual inspection or labeling.

------------------------------------------------------------------------

## pclCluster - Player Separation & Trajectory Extraction

Performs spatial clustering on point clouds.

### Features:

-   Octree-based spatial subdivision (splits clouds into distinct
    clusters/players).
-   Cluster center calculation.
-   Trajectory tracking across frames.
-   Automatic NURBS curve (paths) generation from cluster trajectories.

------------------------------------------------------------------------

## Architecture

The plugin suite follows a modular architecture:

- **Core Library** (`core/`): Maya-independent modules
  - `pclFrameCache`: File loading and caching
  - `pclGeometryUtils`: Geometric utilities (bounding boxes, filtering)

- **Maya Adapters** (`pclLoader/`, `pclRaycaster/`, etc.): Maya-specific nodes
  - Convert between Maya types and core library types
  - Handle Maya dependency graph connections
  - Viewport drawing and visualization

- **Shared Utilities** (`pclUtilities`, `pclCommandHelpers`): Reusable Maya-aware functions

### Data Flow

```
pclLoader (loads files, filters by container)
    ↓ (outPointsPosition, outPointsColor)
    ├→ pclRaycaster (raycasting/frustum culling)
    ├→ pclCluster (spatial clustering)
    └→ pclSequencer (particle system export)
```

## Performance

- **Multithreaded Loading**: Automatically uses multiple CPU cores for parallel file loading
- **Frame Caching**: Loaded frames are cached to avoid redundant I/O
- **Optimized Arrays**: Pre-allocated arrays and cached length calls for performance
- **Connection-Based**: Nodes communicate via Maya's dependency graph (efficient)


# Architecture Diagram
### Data Flow

```mermaid
graph TD
    %% Nodes definition
    CMD[PclSequencerCmd<br/>'System Setup']
    LOADER[pclLoader<br/>'The Central Hub & Logic']
    
    RAY[pclRaycaster<br/>'Camera 2D to 3D']
    CLUS[pclCluster<br/>'Octree Logic']
    SEQ[pclSequencer<br/>'Particle Map']
    
    CAM[Camera<br/>'Hit Points']
    BBOX[BoundingBoxes<br/>'Segments & Path']
    PART[mayaNParticles]

    %% Connections
    CMD -->|1. Creates & Connects| LOADER
    
    LOADER -->|2. OutPoints| RAY
    LOADER -->|2. OutPoints| CLUS
    LOADER -->|2. OutPoints| SEQ
    
    RAY --> CAM
    CLUS --> BBOX
    SEQ --> PART

    %% Styling with subtle dark-theme colors
    style CMD fill:#2D3748,stroke:#63B3ED,stroke-width:2px,color:#FFFFFF
    style LOADER fill:#2D3748,stroke:#63B3ED,stroke-width:2px,color:#FFFFFF
    
    style RAY fill:#253237,stroke:#68D391,stroke-width:1px,color:#E2E8F0
    style CLUS fill:#253237,stroke:#68D391,stroke-width:1px,color:#E2E8F0
    style SEQ fill:#253237,stroke:#68D391,stroke-width:1px,color:#E2E8F0
    
    style CAM fill:#1A202C,stroke:#A0AEC0,stroke-width:1px,stroke-dasharray: 5 5,color:#CBD5E0
    style BBOX fill:#1A202C,stroke:#A0AEC0,stroke-width:1px,stroke-dasharray: 5 5,color:#CBD5E0
    style PART fill:#1A202C,stroke:#A0AEC0,stroke-width:1px,stroke-dasharray: 5 5,color:#CBD5E0
```


--------------------------------------------------------------------------------------

## Project Structure

```markdown
📁 pcl_loader/
│
├── 📁 src/
│   │
│   ├── 📁 pclLoader/
│   │   ├── 📁 core/
│   │   │   ├── 📄 pclFrameCache.cpp
│   │   │   ├── 📄 pclFrameCache.h
│   │   │   ├── 📄 pclGeometryUtils.cpp
│   │   │   ├── 📄 pclGeometryUtils.h
│   │   │
│   │   ├── 📄 pclLoader.cpp
│   │   ├── 📄 pclLoader.h
│   │   ├── 📄 pclLoaderCmd.cpp
│   │   ├── 📄 pclLoaderCmd.h
│   │   ├── 📄 pclCommandHelpers.cpp
│   │   ├── 📄 pclCommandHelpers.h
│   │   ├── 📄 pclContainer.cpp
│   │   ├── 📄 pclContainer.h
│   │   ├── 📄 pclUtilities.cpp
│   │   ├── 📄 pclUtilities.h
│   │   └── 📄 pluginMain.cpp
│   │
│   ├── 📁 pclSequencer/
│   │   ├── 📄 pclSequencer.cpp
│   │   ├── 📄 pclSequencer.h
│   │   ├── 📄 pclSequencerCmd.cpp
│   │   ├── 📄 pclSequencerCmd.h
│   │
│   ├── 📁 pclRaycaster/
│   │   ├── 📄 pclRaycaster.cpp
│   │   ├── 📄 pclRaycaster.h
│   │   ├── 📄 pclRaycasterCmd.cpp
│   │   ├── 📄 pclRaycasterCmd.h
│   │
│   ├── 📁 pclClustering/
│   │   ├── 📄 pclCluster.cpp
│   │   ├── 📄 pclCluster.h
│   │   ├── 📄 pclClusteringCmd.cpp
│   │   ├── 📄 pclClusteringCmd.h
│   │
│   └── 📁 (additional helper files if needed)
│
├── 📁 tests/
│   ├── 📄 README.md
│   ├── 📄 testRunner.cpp
│   ├── 📄 pclUtilitiesTests.cpp
│   │
│   ├── 📁 core/
│   │   ├── 📄 pclFrameCacheTests.cpp
│   │   ├── 📄 pclGeometryUtilsTests.cpp
│
├── 📁 media/
│   ├── 🖼 clustering_step1.png
│   ├── 🖼 clustering_step2.png
│   ├── 🖼 clustering_step3.png
│   ├── 🖼 clustering_step4.png
│   ├── 🖼 display_particles_type.png
│   ├── 🖼 load_point_cloud_sequences.png
│   ├── 🖼 only_points_inside_container.png
│   ├── 🖼 raycaster_from_cam_center.png
│   ├── 🖼 raycaster_from_cam_pixel_pos.png
│
├── 📄 README.md
├── 📄 USAGE.md
└── 📄 .gitignore

```

------------------------------------------------------------------------

## Installation

1. Copy the compiled `.mll` file to your Maya plugin directory:
   ```
   [Maya Install Path]/bin/plug-ins/
   ```
2. Load the plugin in Maya:
   - **Maya 2020+**: Windows → Settings/Preferences → Plug-in Manager
   - Or use: `loadPlugin pclLoader.mll;`

------------------------------------------------------------------------
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
For full usage examples and command reference, see **USAGE.md**.
------------------------------------------------------------------------


For full usage documentation, see [docs/USAGE.md](USAGE.md)

## 🛠 Technical Specifications & Build Requirements

### Environment
- **Maya SDK**: 2020+ (Targeted for performance and stability).
- **Compiler**: Visual Studio 2017 / 2019 (v141/v142 toolsets).
- **Language Standard**: C++17 (Utilizing modern STL and threading).

### External Libraries
- **Open3D 0.12+**: High-performance point-cloud I/O (PLY/PCD).
- **Eigen3 3.3.7+**: Optimized linear algebra for 3D math.
- **Boost 1.70+**: Filesystem and string utility support.

------------------------------------------------------------------------


# Building

1.  Open the Visual Studio solution file\.
2. **Configure Settings**:
    - Set Maya SDK, Open3D, Boost, and Eigen3 include/library paths.
    - Ensure **C++17 standard** is enabled in Project Properties.
3. **Build**: Compile in **Release** mode for **x64**.
4. **Deploy**: Copy the `.mll` file to your Maya plug-ins directory.

------------------------------------------------------------------------

## Troubleshooting

### Plugin Won't Load
- Verify Maya version compatibility (2020+).
- Check that all dependencies (Open3D, Boost, Eigen) are properly linked.
- Ensure Visual Studio runtime libraries are available.

### Files Won't Load
- Verify file paths use forward slashes or escaped backslashes.
- Check that files have `.ply` or `.pcd` extensions.
- Ensure Open3D can read the file format.

### Performance Issues
- Reduce `skipFilesRows` for faster loading (but less points).
- Adjust `recursionDepth` in cluster node for faster clustering.
- Use container filtering to reduce point count.

------------------------------------------------------------------------

## ⚖️ Status & Project Origin
**Legacy Portfolio Project (v1.0.1)** This suite was inspired by high-performance data-processing workflows I developed at Intel. It was rebuilt entirely from scratch as a personal project to demonstrate experience with **C++17, Maya API, and multithreading**. 

- **Compatibility**: Maya 2020+ | Windows | VS 2017-2019.
- **Maintenance**: Not actively maintained; kept for technical demonstration.

---

## See also

- [scatteringNode](https://github.com/PerryGu/scatteringNode) — Maya C++ plugin for fast surface and volume point-cloud scatter (first-and-last-hit). The production volume-fill path.
- [scatteringNet](https://github.com/PerryGu/scatteringNet) — occupancy network for the same volume-fill question (CAD train, inspect on shapes not in the catalog). Not a replacement for a ray test.

---

## Contact & Author
**Perry Guy** - Technical Artist & CG Engineer  
*Extensive experience in 3D Graphics, Tool Development, and Performance Optimization.*

📧 **Email**: [perryguy2@gmail.com](mailto:perryguy2@gmail.com)

**YouTube Channel**: [@ThePerryGuy](https://www.youtube.com/@ThePerryGuy)


## License
This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.
