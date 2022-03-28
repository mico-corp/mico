#ifndef FILEBUFFER_HPP
#define FILEBUFFER_HPP

#include <dv-sdk/data/cvector.hpp>
#include <dvsal/utils/FileDataTable.hpp>

#include <map>
#include <vector>

namespace dv {

enum class StatusPacket { Read, notRead };

struct PacketBuffer {
	FileDataDefinition packet;
	StatusPacket status;
	bool cached;
};

inline bool operator==(const PacketBuffer &lhs, const PacketBuffer &rhs) {
	return (lhs.packet == rhs.packet && lhs.status == rhs.status && lhs.cached == rhs.cached);
}

class FileBuffer {
public:
	std::vector<PacketBuffer> packetsDat;
	std::vector<PacketBuffer> packetsDatRange;

	std::vector<char> &getDataPtrCache(PacketBuffer const &packet) {
		return mapPtr[packet.packet.ByteOffset];
	}

	std::size_t getDataSizeCache(PacketBuffer const &packet) {
		return mapSize[packet.packet.ByteOffset];
	}

	// add new dataPtr and dataSize to the Cache
	void addToCache(const PacketBuffer &packetToAdd, const std::vector<char> &dataptr, const size_t dataSize);

	// remove dataPtr and dataSize relative to packetToRemove from Cache
	void removeFromCache(PacketBuffer &packetToRemove);

	void clearCache();

	// update packetsDatRange in the time range
	void updatePacketsTimeRange(
		const std::int64_t startTimestamp, const std::int64_t endTimestamp, const std::int32_t id);

	// get packet in Range : packetsDatRange
	std::vector<PacketBuffer> getInRange();

	// update cache removing not needed packet from dataptr and datasize
	void updateCache();

	FileBuffer();
	FileBuffer(dv::cvector<FileDataDefinition> &Table);

private:
	std::map<std::int64_t, std::vector<char>> mapPtr; // pair of FileDataDefinitionT::ByteOffset and dataPtr
	std::map<std::int64_t, std::size_t> mapSize;      // pair of FileDataDefinitionT::ByteOffset and dataSize
	std::int64_t currentStartTime;
	std::int64_t currentStopTime;
};

} // namespace dv
#endif // FILEBUFFER_H
