#pragma once
#include <cstdint>
#include <string>
#include <vector>

// Public API of LASPlugin, applications include only this header.
// Usage:
//	using PluginInterface = LASPluginInterface;
//	PluginInterface* Plugin = PLUGIN_MANAGER.GetPlugin<PluginInterface>(UNIQUE_ID.FromString(PluginInterface::PluginID));
//	if (Plugin != nullptr && Plugin->GetInterfaceVersion() == PluginInterface::InterfaceVersion)
//		Plugin->...;
class LASPluginInterface
{
public:
	// Same as "ID" in LASPlugin.json.
	static constexpr const char* PluginID = "c2bceb24-3336-4c25-9951-56ef3bdb9b19";

	// Increase when anything in this file changes.
	static constexpr int InterfaceVersion = 1;

	struct VariableLengthRecord
	{
		std::string UserID;
		uint16_t RecordID = 0;
		std::string Description;
		std::vector<uint8_t> Data;
	};

	struct FileHeader
	{
		uint8_t VersionMajor = 0;
		uint8_t VersionMinor = 0;
		uint8_t PointDataFormat = 0;
		uint16_t PointDataRecordLength = 0;
		uint64_t PointCount = 0;
		std::string SystemIdentifier;
		std::string GeneratingSoftware;
		uint16_t FileCreationDay = 0;
		uint16_t FileCreationYear = 0;
		uint16_t FileSourceID = 0;
		uint16_t GlobalEncoding = 0;
		double ScaleFactor[3] = { 0.0, 0.0, 0.0 };
		double Offset[3] = { 0.0, 0.0, 0.0 };
		double Min[3] = { 0.0, 0.0, 0.0 };
		double Max[3] = { 0.0, 0.0, 0.0 };
		// Coordinate system of the file is usually stored here.
		std::vector<VariableLengthRecord> VariableLengthRecords;
	};

	struct PointAttributes
	{
		uint16_t Intensity = 0;
		uint8_t ReturnNumber = 0;
		uint8_t NumberOfReturns = 0;
		uint8_t Classification = 0;
		bool bSynthetic = false;
		bool bKeyPoint = false;
		bool bWithheld = false;
		bool bOverlap = false;
		uint8_t ScannerChannel = 0;
		bool bScanDirection = false;
		bool bEdgeOfFlightLine = false;
		// In degrees.
		float ScanAngle = 0.0f;
		uint8_t UserData = 0;
		uint16_t PointSourceID = 0;
		double GPSTime = 0.0;
		uint16_t NearInfrared = 0;
	};

	struct FileData
	{
		std::string FilePath;
		bool bCompressed = false;
		FileHeader Header;

		// X, Y and Z of every point with scale and offset of the file applied, empty after import because positions are in the point cloud.
		std::vector<double> Positions;
		// R, G and B of every point as stored in the file, empty if point format has no colors or after import.
		std::vector<uint16_t> Colors;
		// Empty if attributes were not requested, otherwise one for every point, in the same order as points of the point cloud.
		std::vector<PointAttributes> Attributes;
		uint32_t ExtraBytesPerPoint = 0;
		// ExtraBytesPerPoint bytes of every point, empty if attributes were not requested.
		std::vector<uint8_t> ExtraBytes;

		// Set only by import, ID of the created point cloud.
		std::string PointCloudID;
		// Set only by import, point cloud positions are file positions minus this shift.
		double AppliedShift[3] = { 0.0, 0.0, 0.0 };
	};

	// Order of virtual functions is part of the interface, new functions should be added only at the end.
	virtual ~LASPluginInterface() = default;
	virtual int GetInterfaceVersion() const { return InterfaceVersion; }

	// Reads file without creating point cloud, works also without graphics.
	// Returned data is owned by plugin and is valid until next read or import, nullptr if file could not be read.
	virtual const FileData* ReadFile(const std::string& FilePath, bool bReadAttributes = false) = 0;
	// Data of the last read or imported file, nullptr if there is none or if it could not be read.
	virtual const FileData* GetLastLoadedData() const = 0;

	// Attributes of imported point clouds cost memory, so they are not kept by default.
	virtual void SetKeepAttributesOnImport(bool NewValue) = 0;
	virtual bool IsKeepingAttributesOnImport() const = 0;

	// Positions are X, Y and Z of every point, Colors are R, G and B of every point or empty. ScaleFactor is precision of saved positions.
	virtual bool SaveFile(const std::string& FilePath, const std::vector<double>& Positions, const std::vector<uint16_t>& Colors, bool bCompressed, double ScaleFactor = 0.001) = 0;
};