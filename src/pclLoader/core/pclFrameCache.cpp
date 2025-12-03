#include "pclFrameCache.h"

#include <Open3D/Open3D.h>
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <algorithm>
#include <regex>
#include <thread>
#include <atomic>

namespace
{
// Returns true when the provided extension is supported by Open3D.
bool hasPointCloudExtension(const std::string& ext)
{
	return ext == ".ply" || ext == ".pcd";
}

// Normalizes path separators so downstream parsing is predictable.
std::string normalizePath(const std::string& path)
{
	std::string result = path;
	boost::replace_all(result, "\\", "/");
	return result;
}
}

namespace
{
// Holds prefix/suffix metadata extracted from a token that contains '#'.
struct SequencePattern
{
	std::string prefix;
	std::string suffix;
	size_t padding = 0;
	bool hasHashes = false;
};

// Breaks a pattern token into prefix, suffix, and padding length.
SequencePattern parsePattern(const std::string& token)
{
	SequencePattern pattern;
	auto first = token.find('#');
	if (first == std::string::npos)
	{
		pattern.prefix = token;
		return pattern;
	}

	auto last = token.find_last_of('#');
	pattern.prefix = token.substr(0, first);
	if (last + 1 < token.size())
	{
		pattern.suffix = token.substr(last + 1);
	}

	pattern.padding = last - first + 1;
	pattern.hasHashes = true;
	return pattern;
}

// Stores the decomposed pieces of the user supplied path.
struct SequencePathParts
{
	boost::filesystem::path basePath;
	boost::filesystem::path trailingPath;
	std::string patternToken;
	bool hasPattern = false;
};

// Splits the normalized path into base/pattern/trailing sections.
SequencePathParts dissectSequencePath(const std::string& normalizedPath)
{
	namespace bfs = boost::filesystem;
	SequencePathParts parts;
	const bfs::path full(normalizedPath);

	bfs::path before = full.root_path();
	const bfs::path relative = full.relative_path();
	for (const auto& component : relative)
	{
		const std::string name = component.string();
		if (!parts.hasPattern && name.find('#') != std::string::npos)
		{
			parts.patternToken = name;
			parts.hasPattern = true;
			continue;
		}

		if (!parts.hasPattern)
		{
			before /= component;
		}
		else
		{
			parts.trailingPath /= component;
		}
	}

	if (!parts.hasPattern)
	{
		parts.basePath = full;
	}
	else
	{
		parts.basePath = before.empty() ? bfs::path(".") : before;
	}

	return parts;
}

// Expands a matched directory entry into the concrete files for a frame.
bool collectFrameFiles(const boost::filesystem::path& entryPath,
	const boost::filesystem::path& trailingPath,
	std::vector<std::string>& outFiles)
{
	namespace bfs = boost::filesystem;

	const bfs::path target = trailingPath.empty() ? entryPath : entryPath / trailingPath;
	if (!bfs::exists(target))
	{
		return false;
	}

	if (bfs::is_regular_file(target))
	{
		if (!hasPointCloudExtension(target.extension().string()))
		{
			return false;
		}

		outFiles.push_back(target.string());
		return true;
	}

	if (!bfs::is_directory(target))
	{
		return false;
	}

	for (bfs::directory_iterator itr(target); itr != bfs::directory_iterator(); ++itr)
	{
		const bfs::path child = (*itr);
		if (!bfs::is_regular_file(child))
		{
			continue;
		}

		if (!hasPointCloudExtension(child.extension().string()))
		{
			continue;
		}

		outFiles.push_back(child.string());
	}

	std::sort(outFiles.begin(), outFiles.end());
	return !outFiles.empty();
}
}

// Resets cached frames and metadata.
void pclFrameCache::clear()
{
	mFrames.clear();
	mFrameNumbers.clear();
	mStartFrame = 0;
	mEndFrame = 0;
	mHighestPointCount = 0;
}

// Public entry point used by nodes to populate the cache.
bool pclFrameCache::loadSequence(const std::string& mainPath, int skipRows, unsigned int maxThreads)
{
	clear();
	if (mainPath.empty())
	{
		return false;
	}

	std::string normalizedPath = normalizePath(mainPath);
	boost::replace_all(normalizedPath, "*", "#");

	namespace bfs = boost::filesystem;
	if (bfs::is_regular_file(normalizedPath))
	{
		return gatherSingleFile(normalizedPath);
	}

	// Dispatch based on whether the path points to a file, sequence, or folder.
	// Pass maxThreads for parallel loading support.
	if (!gatherFolder(normalizedPath, skipRows, maxThreads))
	{
		return false;
	}

	// Update frame metadata so Maya nodes can query basic statistics quickly.
	if (!mFrames.empty())
	{
		mStartFrame = static_cast<unsigned int>(mFrameNumbers.front());
		mEndFrame = static_cast<unsigned int>(mFrameNumbers.back());
		for (const auto& frame : mFrames)
		{
			mHighestPointCount = std::max(mHighestPointCount, static_cast<unsigned int>(frame.positions.size()));
		}
	}

	return true;
}

// Treats the supplied path as a single frame made of one file.
bool pclFrameCache::gatherSingleFile(const std::string& filePath)
{
	return loadFrameFiles({ filePath }, 0, 0);
}

// Determines whether a folder contains a numbered sequence or a simple set.
bool pclFrameCache::gatherFolder(const std::string& folderPath, int skipRows, unsigned int maxThreads)
{
	namespace bfs = boost::filesystem;

	if (!bfs::exists(folderPath))
	{
		return false;
	}

	const bool hasSequencePattern = folderPath.find('#') != std::string::npos;
	if (hasSequencePattern)
	{
		return gatherSequenceFolder(folderPath, skipRows, maxThreads);
	}

	return gatherSingleFrameFolder(folderPath, skipRows);
}

// Resolves sequence patterns into frame file lists.
bool pclFrameCache::gatherSequenceFolder(const std::string& folderPath, int skipRows, unsigned int maxThreads)
{
	namespace bfs = boost::filesystem;
	const SequencePathParts parts = dissectSequencePath(folderPath);
	// Bail out if the user path does not actually contain a pattern token.
	if (!parts.hasPattern)
	{
		return false;
	}

	const SequencePattern pattern = parsePattern(parts.patternToken);
	// Guard against malformed patterns that lack hashes or padding.
	if (!pattern.hasHashes || pattern.padding == 0)
	{
		return false;
	}

	// Ensure the base directory exists before we begin scanning.
	if (!bfs::exists(parts.basePath) || !bfs::is_directory(parts.basePath))
	{
		return false;
	}

	const std::string regexPattern = "^" + pattern.prefix + "(\\d{" + std::to_string(pattern.padding) + "})" + pattern.suffix + "$";
	const std::regex matchRegex(regexPattern);

	std::vector<std::pair<int, std::vector<std::string>>> frames;

	// Iterate every entry in the base directory and match it against the pattern.
	for (bfs::directory_iterator itr(parts.basePath); itr != bfs::directory_iterator(); ++itr)
	{
		const bfs::path entry = (*itr);
		const std::string name = entry.filename().string();
		std::smatch match;
		// Skip entries whose names do not match the expected digit padding.
		if (!std::regex_match(name, match, matchRegex))
		{
			continue;
		}

		std::vector<std::string> frameFiles;
		// Collect the files that make up this frame (single file or nested folder).
		if (!collectFrameFiles(entry, parts.trailingPath, frameFiles))
		{
			continue;
		}

		const int frameNumber = match.size() > 1 ? std::stoi(match[1].str()) : static_cast<int>(frameFiles.size());
		frames.emplace_back(frameNumber, frameFiles);
	}

	if (frames.empty())
	{
		return false;
	}

	// Sort by the numerical frame index so frame numbering is stable.
	std::sort(frames.begin(), frames.end(), [](const auto& lhs, const auto& rhs) {
		return lhs.first < rhs.first;
		});

	// Prepare frame loading tasks for parallel execution.
	std::vector<FrameLoadTask> tasks;
	tasks.reserve(frames.size());
	for (const auto& frame : frames)
	{
		FrameLoadTask task;
		task.frameFiles = frame.second;
		task.frameIndex = frame.first;
		task.skipRows = skipRows;
		tasks.push_back(task);
	}

	// Load frames in parallel if we have multiple frames and threading is enabled.
	if (tasks.size() > 1 && maxThreads > 0)
	{
		return loadFramesParallel(tasks, maxThreads);
	}

	// Fall back to sequential loading for single frames or when threading is disabled.
	bool ok = true;
	for (const auto& task : tasks)
	{
		ok &= loadFrameFiles(task.frameFiles, task.frameIndex, task.skipRows);
	}
	return ok;
}

// Loads every point cloud contained directly inside a folder.
bool pclFrameCache::gatherSingleFrameFolder(const std::string& folderPath, int skipRows)
{
	namespace bfs = boost::filesystem;
	// Without the folder there is nothing to load.
	if (!bfs::exists(folderPath))
	{
		return false;
	}

	std::vector<std::string> frameFiles;
	// Inspect every file in the folder and keep only valid point cloud extensions.
	for (bfs::directory_iterator itr(folderPath); itr != bfs::directory_iterator(); ++itr)
	{
		const bfs::path file = (*itr);
		if (bfs::is_regular_file(file))
		{
			const std::string ext = file.extension().string();
			if (hasPointCloudExtension(ext))
			{
				frameFiles.push_back(file.string());
			}
		}
	}

	return loadFrameFiles(frameFiles, 0, skipRows);
}

// Converts a list of files into cached frame data and tracks frame numbers.
bool pclFrameCache::loadFrameFiles(const std::vector<std::string>& frameFiles, int frameIndex, int skipRows)
{
	if (frameFiles.empty())
	{
		return false;
	}

	FrameData frame;
	frame.sourceFiles = frameFiles;
	for (const auto& file : frameFiles)
	{
		// If any contributing file fails we abort the entire frame to keep data consistent.
		if (!loadPointCloudFile(file, frame, skipRows))
		{
			return false;
		}
	}

	mFrames.push_back(frame);
	mFrameNumbers.push_back(frameIndex);
	return true;
}

// Uses Open3D to stream a point cloud file into the cache-friendly format.
bool pclFrameCache::loadPointCloudFile(const std::string& filePath, FrameData& frame, int skipRows)
{
	using namespace open3d;
	auto cloud = std::make_shared<geometry::PointCloud>();
	// Reading through Open3D ensures consistent parsing between .ply and .pcd inputs.
	if (!io::ReadPointCloud(filePath, *cloud))
	{
		return false;
	}

	const int stride = std::max(1, skipRows + 1);
	// Down-sample by stepping across the points array according to skipRows.
	for (size_t i = 0; i < cloud->points_.size(); i += static_cast<size_t>(stride))
	{
		const auto& pt = cloud->points_[i];
		frame.positions.emplace_back(pt.x(), pt.y(), pt.z());

		Eigen::Vector3d color(0.5, 0.5, 0.5);
		if (cloud->HasColors())
		{
			const auto& c = cloud->colors_[i];
			color = Eigen::Vector3d(c.x(), c.y(), c.z());
		}
		frame.colors.emplace_back(color);
	}

	return true;
}

// Thread-safe helper to append a loaded frame to the cache.
void pclFrameCache::appendFrame(const FrameData& frame, int frameIndex)
{
	std::lock_guard<std::mutex> lock(mCacheMutex);
	mFrames.push_back(frame);
	mFrameNumbers.push_back(frameIndex);
}

// Loads frames in parallel using worker threads.
bool pclFrameCache::loadFramesParallel(const std::vector<FrameLoadTask>& tasks, unsigned int maxThreads)
{
	if (tasks.empty())
	{
		return false;
	}

	// Determine optimal thread count (don't exceed hardware concurrency or task count).
	// std::thread::hardware_concurrency() returns 0 if not available (C++11, VS2013+).
	const unsigned int hardwareThreads = std::thread::hardware_concurrency();
	const unsigned int numThreads = (maxThreads > 0) 
		? std::min(maxThreads, std::min((hardwareThreads > 0 ? hardwareThreads : 4u), static_cast<unsigned int>(tasks.size())))
		: std::min((hardwareThreads > 0 ? hardwareThreads : 4u), static_cast<unsigned int>(tasks.size()));

	const unsigned int actualThreads = (numThreads == 0) ? 1 : numThreads;

	// Pre-allocate result storage for thread-safe frame insertion.
	// We'll use a vector of pairs to store (frameIndex, frameData) and sort later.
	std::vector<std::pair<int, FrameData>> loadedFrames;
	loadedFrames.reserve(tasks.size());
	std::mutex resultMutex;
	std::atomic<bool> hasError(false);

	// Worker function that loads a single frame.
	auto loadFrameWorker = [&](size_t startIndex, size_t endIndex) {
		for (size_t i = startIndex; i < endIndex && !hasError.load(); ++i)
		{
			const FrameLoadTask& task = tasks[i];
			FrameData frame;
			frame.sourceFiles = task.frameFiles;

			// Load all files for this frame.
			bool frameOk = true;
			for (const auto& file : task.frameFiles)
			{
				if (!loadPointCloudFile(file, frame, task.skipRows))
				{
					frameOk = false;
					break;
				}
			}

			if (!frameOk)
			{
				hasError.store(true);
				continue;
			}

			// Thread-safely append the loaded frame.
			{
				std::lock_guard<std::mutex> lock(resultMutex);
				loadedFrames.emplace_back(task.frameIndex, std::move(frame));
			}
		}
	};

	// Distribute tasks across worker threads.
	const size_t tasksPerThread = (tasks.size() + actualThreads - 1) / actualThreads;
	std::vector<std::thread> workers;

	for (unsigned int t = 0; t < actualThreads; ++t)
	{
		const size_t startIndex = t * tasksPerThread;
		const size_t endIndex = std::min(startIndex + tasksPerThread, tasks.size());
		
		if (startIndex < tasks.size())
		{
			workers.emplace_back(loadFrameWorker, startIndex, endIndex);
		}
	}

	// Wait for all worker threads to complete.
	for (auto& worker : workers)
	{
		if (worker.joinable())
		{
			worker.join();
		}
	}

	// Check for errors.
	if (hasError.load() || loadedFrames.empty())
	{
		return false;
	}

	// Sort loaded frames by frame index to maintain correct order.
	std::sort(loadedFrames.begin(), loadedFrames.end(), 
		[](const auto& lhs, const auto& rhs) {
			return lhs.first < rhs.first;
		});

	// Append sorted frames to cache (already protected by mutex in appendFrame).
	for (const auto& pair : loadedFrames)
	{
		appendFrame(pair.second, pair.first);
	}

	return true;
}


