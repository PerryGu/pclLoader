#ifndef PCLFRAMECACHE_H
#define PCLFRAMECACHE_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <mutex>
#include <thread>
#include <atomic>

class pclFrameCache
{
public:
	/// Loaded frame data in native (Maya-free) format.
	struct FrameData
	{
		std::vector<Eigen::Vector3d> positions;
		std::vector<Eigen::Vector3d> colors;
		std::vector<std::string> sourceFiles;
	};

	void clear();

	/// Load a sequence from the given path. Returns false if no valid frames found.
	/// @param mainPath Path to the point cloud sequence or file.
	/// @param skipRows Stride for downsampling points (0 = no downsampling).
	/// @param maxThreads Maximum number of threads to use for parallel loading (0 = auto-detect).
	bool loadSequence(const std::string& mainPath, int skipRows, unsigned int maxThreads = 0);

	/// Cached frame list.
	const std::vector<FrameData>& frames() const { return mFrames; }
	unsigned int startFrame() const { return mStartFrame; }
	unsigned int endFrame() const { return mEndFrame; }

	/// Highest point count across all frames (for particle padding).
	unsigned int highestPointCount() const { return mHighestPointCount; }

private:
	/// Load a single file as a single frame.
	bool gatherSingleFile(const std::string& filePath);
	/// Determine whether path points to a sequence or a simple folder.
	bool gatherFolder(const std::string& folderPath, int skipRows, unsigned int maxThreads = 0);
	/// Handle paths with '#' pattern.
	bool gatherSequenceFolder(const std::string& folderPath, int skipRows, unsigned int maxThreads = 0);
	/// Handle simple folder (no '#').
	bool gatherSingleFrameFolder(const std::string& folderPath, int skipRows);
	/// Load the given files into a frame and track metadata.
	bool loadFrameFiles(const std::vector<std::string>& frameFiles, int frameIndex, int skipRows);
	/// Load a single file via Open3D and append positions/colors.
	bool loadPointCloudFile(const std::string& filePath, FrameData& frame, int skipRows);

	/// Thread-safe frame loading helper.
	/// Loads a single frame and appends it to the cache with proper synchronization.
	struct FrameLoadTask
	{
		std::vector<std::string> frameFiles;
		int frameIndex;
		int skipRows;
	};

	/// Loads frames in parallel using worker threads.
	bool loadFramesParallel(const std::vector<FrameLoadTask>& tasks, unsigned int maxThreads);

	/// Thread-safe helper to append a loaded frame to the cache.
	void appendFrame(const FrameData& frame, int frameIndex);

	std::vector<int> mFrameNumbers;
	std::vector<FrameData> mFrames;
	unsigned int mStartFrame = 0;
	unsigned int mEndFrame = 0;
	unsigned int mHighestPointCount = 0;

	/// Mutex for protecting shared data structures during parallel loading.
	mutable std::mutex mCacheMutex;
};

#endif // PCLFRAMECACHE_H

