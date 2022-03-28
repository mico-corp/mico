

#include "dvsal/utils/filebuffer.hpp"

using namespace dv;

FileBuffer::FileBuffer() : packetsDat(0), packetsDatRange(0) {
}

void FileBuffer::addToCache(const PacketBuffer &packetToAdd, const std::vector<char> &dataptr, const size_t dataSize) {
	//    assert(packetToAdd. != 0);
	assert(!dataptr.empty());
	assert(dataSize != 0);

	mapPtr.insert({packetToAdd.packet.ByteOffset, dataptr});
	mapSize.insert({packetToAdd.packet.ByteOffset, dataSize});

	auto added = std::find(packetsDat.cbegin(), packetsDat.cend(), packetToAdd);
	packetsDat[static_cast<std::size_t>(added - packetsDat.cbegin())].cached = true;
}

void FileBuffer::removeFromCache(PacketBuffer &packetToRemove) {
	assert(packetToRemove.packet.ByteOffset != 0);
	assert(mapPtr.find(packetToRemove.packet.ByteOffset) != mapPtr.cend());

	mapPtr.erase(packetToRemove.packet.ByteOffset);
	mapSize.erase(packetToRemove.packet.ByteOffset);
	packetToRemove.status = StatusPacket::notRead;
}

void FileBuffer::updatePacketsTimeRange(
	const std::int64_t startTimestamp, const std::int64_t endTimestamp, const std::int32_t id) {
	currentStartTime = startTimestamp;
	currentStopTime  = endTimestamp;
	packetsDatRange.clear();

	for (auto &packdat : packetsDat) {
		if (packdat.packet.TimestampStart <= currentStopTime && packdat.packet.TimestampEnd >= currentStartTime
			&& packdat.packet.PacketInfo.StreamID() == id) {
			packetsDatRange.push_back(packdat);
			packdat.status = StatusPacket::Read;
		}
	}

	updateCache(); // remove packet not in the timeRange
}

void FileBuffer::updateCache() {
	for (auto &pack : packetsDat) {
		if (pack.status == StatusPacket::Read) {
			if (std::find(packetsDatRange.cbegin(), packetsDatRange.cend(), pack) == packetsDatRange.cend()) {
				if (mapPtr.find(pack.packet.ByteOffset) != mapPtr.cend()) {
					if (pack.packet.TimestampEnd < currentStartTime || pack.packet.TimestampStart > currentStopTime) {
						removeFromCache(pack);
						pack.cached = false;
					}
				}
			}
		}
	}
}

FileBuffer::FileBuffer(dv::cvector<FileDataDefinition> &Table) : packetsDat(0), packetsDatRange(0) {
	for (auto elem : Table) {
		PacketBuffer newPack;
		newPack.packet = elem;
		newPack.status = StatusPacket::notRead;
		newPack.cached = false;
		packetsDat.push_back(newPack);
	}
}

std::vector<PacketBuffer> FileBuffer::getInRange() {
	return packetsDatRange;
}
