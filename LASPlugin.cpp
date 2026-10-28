#include "LASPlugin.h"
#include <cfloat>
#include <cmath>
#include <cstring>

// laszip_api.h adds "using namespace std;" to the global namespace, after that "byte" in the Windows rpcndr.h is ambiguous with std::byte, so rpcndr.h (through unknwn.h) is included first.
#include <unknwn.h>
#include "ThirdParty/laszip/laszip_api.h"

// DO NOT CHANGE THIS LINE.
// The plugin manager uses it to create and destroy the plugin instance.
REGISTER_PLUGIN(LASPlugin)

bool LASPlugin::OnLoad()
{
#ifdef _DEBUG
	const std::string ConfigurationFolder = "Debug/";
#else
	const std::string ConfigurationFolder = "Release/";
#endif

	// laszip_load_dll loads LASzip64.dll by name, so it gets the one that is loaded here from plugin folder.
	// In games DLLs of plugins are next to executable, where it is found by name.
	LASzipDLL = LoadLibraryA((GetPluginDirectory() + ConfigurationFolder + "LASzip64.dll").c_str());
	if (laszip_load_dll() != 0)
	{
		// Plugin stays loaded, its functions fail while LASzip is not loaded.
		LOG.Add("LASPlugin::OnLoad: Loading LASzip DLL failed, LAS and LAZ files are not supported", "LAS_PLUGIN", FE_LOG_ERROR);
		return true;
	}
	bLASzipLoaded = true;

	// Point clouds are created on GPU.
	if (APPLICATION.GetGraphicsMode() == FEGraphicsMode::None)
		return true;

	FEResourceImporter Importer;
	Importer.Description = "LAS point cloud";
	Importer.Extensions = { "las", "laz" };
	Importer.Import = [this](const std::string& FilePath, FEScene* Scene) { return ImportPointCloud(FilePath); };
	ImporterID = RESOURCE_MANAGER.RegisterResourceImporter(Importer);

	FEResourceExporter Exporter;
	Exporter.ObjectType = FE_POINT_CLOUD;
	Exporter.Description = "LAS";
	Exporter.Extensions = { "las" };
	Exporter.Export = [this](FEObject* Object, const std::string& FilePath) { return ExportPointCloud(Object, FilePath, false); };
	LASExporterID = RESOURCE_MANAGER.RegisterResourceExporter(Exporter);

	Exporter.Description = "LAZ";
	Exporter.Extensions = { "laz" };
	Exporter.Export = [this](FEObject* Object, const std::string& FilePath) { return ExportPointCloud(Object, FilePath, true); };
	LAZExporterID = RESOURCE_MANAGER.RegisterResourceExporter(Exporter);
	return true;
}

void LASPlugin::OnUnload()
{
	RESOURCE_MANAGER.UnregisterResourceImporter(ImporterID);
	RESOURCE_MANAGER.UnregisterResourceExporter(LASExporterID);
	RESOURCE_MANAGER.UnregisterResourceExporter(LAZExporterID);
	LastLoadedData.reset();

	if (bLASzipLoaded)
		laszip_unload_dll();
	bLASzipLoaded = false;

	if (LASzipDLL != nullptr)
		FreeLibrary(LASzipDLL);
	LASzipDLL = nullptr;
}

void LASPlugin::OnUpdate(double DeltaTime)
{

}

static void CloseAndDestroyReader(laszip_POINTER LaszipReader, bool bCloseReader)
{
	if (bCloseReader && laszip_close_reader(LaszipReader))
		LOG.Add("CloseAndDestroyReader: closing laszip reader failed", "LAS_PLUGIN", FE_LOG_ERROR);

	if (laszip_destroy(LaszipReader))
		LOG.Add("CloseAndDestroyReader: destroying laszip reader failed", "LAS_PLUGIN", FE_LOG_ERROR);
}

// Text fields of LAS header have fixed size and are not always null terminated.
template<size_t Size>
static std::string FixedSizeText(const laszip_CHAR(&Text)[Size])
{
	return std::string(Text, strnlen(Text, Size));
}

static void ConvertHeader(const laszip_header& Source, LASPluginInterface::FileHeader& Destination)
{
	Destination.VersionMajor = Source.version_major;
	Destination.VersionMinor = Source.version_minor;
	Destination.PointDataFormat = Source.point_data_format;
	Destination.PointDataRecordLength = Source.point_data_record_length;
	Destination.PointCount = Source.number_of_point_records ? Source.number_of_point_records : Source.extended_number_of_point_records;
	Destination.SystemIdentifier = FixedSizeText(Source.system_identifier);
	Destination.GeneratingSoftware = FixedSizeText(Source.generating_software);
	Destination.FileCreationDay = Source.file_creation_day;
	Destination.FileCreationYear = Source.file_creation_year;
	Destination.FileSourceID = Source.file_source_ID;
	Destination.GlobalEncoding = Source.global_encoding;

	const double ScaleFactor[3] = { Source.x_scale_factor, Source.y_scale_factor, Source.z_scale_factor };
	const double Offset[3] = { Source.x_offset, Source.y_offset, Source.z_offset };
	const double Min[3] = { Source.min_x, Source.min_y, Source.min_z };
	const double Max[3] = { Source.max_x, Source.max_y, Source.max_z };
	for (int i = 0; i < 3; i++)
	{
		Destination.ScaleFactor[i] = ScaleFactor[i];
		Destination.Offset[i] = Offset[i];
		Destination.Min[i] = Min[i];
		Destination.Max[i] = Max[i];
	}

	for (laszip_U32 i = 0; i < Source.number_of_variable_length_records && Source.vlrs != nullptr; i++)
	{
		const laszip_vlr_struct& SourceRecord = Source.vlrs[i];
		LASPluginInterface::VariableLengthRecord Record;
		Record.UserID = FixedSizeText(SourceRecord.user_id);
		Record.RecordID = SourceRecord.record_id;
		Record.Description = FixedSizeText(SourceRecord.description);
		if (SourceRecord.data != nullptr)
			Record.Data.assign(SourceRecord.data, SourceRecord.data + SourceRecord.record_length_after_header);

		Destination.VariableLengthRecords.push_back(Record);
	}
}

static LASPluginInterface::PointAttributes ConvertPointAttributes(const laszip_point& Point, bool bExtendedPointFormat)
{
	LASPluginInterface::PointAttributes Result;
	Result.Intensity = Point.intensity;
	Result.bScanDirection = Point.scan_direction_flag != 0;
	Result.bEdgeOfFlightLine = Point.edge_of_flight_line != 0;
	Result.UserData = Point.user_data;
	Result.PointSourceID = Point.point_source_ID;
	Result.GPSTime = Point.gps_time;
	Result.NearInfrared = Point.rgb[3];

	// Point formats 6-10 of LAS 1.4 store these values in extended fields.
	if (bExtendedPointFormat)
	{
		Result.ReturnNumber = Point.extended_return_number;
		Result.NumberOfReturns = Point.extended_number_of_returns;
		Result.Classification = Point.extended_classification;
		Result.bSynthetic = (Point.extended_classification_flags & 1) != 0;
		Result.bKeyPoint = (Point.extended_classification_flags & 2) != 0;
		Result.bWithheld = (Point.extended_classification_flags & 4) != 0;
		Result.bOverlap = (Point.extended_classification_flags & 8) != 0;
		Result.ScannerChannel = Point.extended_scanner_channel;
		// Extended scan angle is stored in 0.006 degree steps.
		Result.ScanAngle = static_cast<float>(Point.extended_scan_angle * 0.006);
	}
	else
	{
		Result.ReturnNumber = Point.return_number;
		Result.NumberOfReturns = Point.number_of_returns;
		Result.Classification = Point.classification;
		Result.bSynthetic = Point.synthetic_flag != 0;
		Result.bKeyPoint = Point.keypoint_flag != 0;
		Result.bWithheld = Point.withheld_flag != 0;
		Result.ScanAngle = static_cast<float>(Point.scan_angle_rank);
	}

	return Result;
}

bool LASPlugin::ReadLASFile(const std::string& FilePath, bool bReadAttributes, FileData& OutData, std::vector<FEPointCloudVertexDouble>* OutVertices)
{
	if (FilePath.empty())
	{
		LOG.Add("LASPlugin::ReadLASFile: FilePath is empty", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	if (!FILE_SYSTEM.DoesFileExist(FilePath))
	{
		LOG.Add("LASPlugin::ReadLASFile: File does not exist", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	if (!bLASzipLoaded)
	{
		LOG.Add("LASPlugin::ReadLASFile: LASzip DLL is not loaded", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	laszip_POINTER LaszipReader;
	laszip_I32 Error = laszip_create(&LaszipReader);
	if (Error)
	{
		LOG.Add("LASPlugin::ReadLASFile: Creating laszip reader failed with error: " + std::to_string(Error), "LAS_PLUGIN", FE_LOG_ERROR);
		return false;
	}

	laszip_BOOL bIsCompressed = 0;
	bool bIsLASOrLAZFile = !laszip_open_reader(LaszipReader, FilePath.c_str(), &bIsCompressed);
	if (!bIsLASOrLAZFile)
	{
		LOG.Add("LASPlugin::ReadLASFile: File is not a valid LAS/LAZ file", "LAS_PLUGIN", FE_LOG_ERROR);
		CloseAndDestroyReader(LaszipReader, false);
		return false;
	}

	laszip_header* FileHeader;
	if (laszip_get_header_pointer(LaszipReader, &FileHeader))
	{
		LOG.Add("LASPlugin::ReadLASFile: getting header pointer from laszip reader failed", "LAS_PLUGIN", FE_LOG_ERROR);
		CloseAndDestroyReader(LaszipReader, true);
		return false;
	}

	laszip_point* CurrentPointPointer;
	if (laszip_get_point_pointer(LaszipReader, &CurrentPointPointer))
	{
		LOG.Add("LASPlugin::ReadLASFile: getting point pointer from laszip reader failed", "LAS_PLUGIN", FE_LOG_ERROR);
		CloseAndDestroyReader(LaszipReader, true);
		return false;
	}

	laszip_U64 PointCount = (FileHeader->number_of_point_records ? FileHeader->number_of_point_records : FileHeader->extended_number_of_point_records);
	if (PointCount == 0)
	{
		LOG.Add("LASPlugin::ReadLASFile: Point count is zero", "LAS_PLUGIN", FE_LOG_ERROR);
		CloseAndDestroyReader(LaszipReader, true);
		return false;
	}

	OutData.FilePath = FilePath;
	OutData.bCompressed = bIsCompressed != 0;
	ConvertHeader(*FileHeader, OutData.Header);

	const laszip_U8 PointFormat = FileHeader->point_data_format;
	const bool bHasColors = PointFormat == 2 || PointFormat == 3 || PointFormat == 5 || PointFormat == 7 || PointFormat == 8 || PointFormat == 10;
	const bool bExtendedPointFormat = PointFormat >= 6;

	if (OutVertices != nullptr)
	{
		OutVertices->resize(PointCount);
	}
	else
	{
		OutData.Positions.resize(PointCount * 3);
		if (bHasColors)
			OutData.Colors.resize(PointCount * 3);
	}

	if (bReadAttributes)
	{
		OutData.Attributes.resize(PointCount);
		OutData.ExtraBytesPerPoint = CurrentPointPointer->num_extra_bytes > 0 ? static_cast<uint32_t>(CurrentPointPointer->num_extra_bytes) : 0;
		OutData.ExtraBytes.resize(PointCount * OutData.ExtraBytesPerPoint);
	}

	laszip_U64 PointIndex = 0;
	while (PointIndex < PointCount)
	{
		if (laszip_read_point(LaszipReader))
		{
			LOG.Add("LASPlugin::ReadLASFile: reading point from laszip reader failed", "LAS_PLUGIN", FE_LOG_ERROR);
			CloseAndDestroyReader(LaszipReader, true);
			if (OutVertices != nullptr)
				OutVertices->clear();

			return false;
		}

		const double X = CurrentPointPointer->X * FileHeader->x_scale_factor + FileHeader->x_offset;
		const double Y = CurrentPointPointer->Y * FileHeader->y_scale_factor + FileHeader->y_offset;
		const double Z = CurrentPointPointer->Z * FileHeader->z_scale_factor + FileHeader->z_offset;

		if (OutVertices != nullptr)
		{
			FEPointCloudVertexDouble& Vertex = (*OutVertices)[PointIndex];
			Vertex.X = X;
			Vertex.Y = Y;
			Vertex.Z = Z;

			Vertex.R = unsigned char(CurrentPointPointer->rgb[0] / float(1 << 16) * 255);
			Vertex.G = unsigned char(CurrentPointPointer->rgb[1] / float(1 << 16) * 255);
			Vertex.B = unsigned char(CurrentPointPointer->rgb[2] / float(1 << 16) * 255);
		}
		else
		{
			OutData.Positions[PointIndex * 3] = X;
			OutData.Positions[PointIndex * 3 + 1] = Y;
			OutData.Positions[PointIndex * 3 + 2] = Z;

			if (bHasColors)
			{
				OutData.Colors[PointIndex * 3] = CurrentPointPointer->rgb[0];
				OutData.Colors[PointIndex * 3 + 1] = CurrentPointPointer->rgb[1];
				OutData.Colors[PointIndex * 3 + 2] = CurrentPointPointer->rgb[2];
			}
		}

		if (bReadAttributes)
		{
			OutData.Attributes[PointIndex] = ConvertPointAttributes(*CurrentPointPointer, bExtendedPointFormat);
			if (OutData.ExtraBytesPerPoint > 0 && CurrentPointPointer->extra_bytes != nullptr)
				std::memcpy(OutData.ExtraBytes.data() + PointIndex * OutData.ExtraBytesPerPoint, CurrentPointPointer->extra_bytes, OutData.ExtraBytesPerPoint);
		}

		PointIndex++;
	}

	CloseAndDestroyReader(LaszipReader, true);
	return true;
}

bool LASPlugin::WriteLASFile(const std::string& FilePath, size_t PointCount, const std::function<void(size_t Index, double* Position, uint16_t* Color)>& GetPoint, bool bIsCompressed, double ScaleFactor, const double* Offset)
{
	if (PointCount == 0)
	{
		LOG.Add("LASPlugin::WriteLASFile: There are no points", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	if (PointCount > UINT32_MAX)
	{
		LOG.Add("LASPlugin::WriteLASFile: Too many points for LAS 1.2 file", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	if (FilePath.empty())
	{
		LOG.Add("LASPlugin::WriteLASFile: FilePath is empty", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	if (!bLASzipLoaded)
	{
		LOG.Add("LASPlugin::WriteLASFile: LASzip DLL is not loaded", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	double Position[3];
	uint16_t Color[3];
	double Min[3] = { DBL_MAX, DBL_MAX, DBL_MAX };
	double Max[3] = { -DBL_MAX, -DBL_MAX, -DBL_MAX };
	for (size_t i = 0; i < PointCount; i++)
	{
		GetPoint(i, Position, Color);
		for (int j = 0; j < 3; j++)
		{
			Min[j] = std::min(Min[j], Position[j]);
			Max[j] = std::max(Max[j], Position[j]);
		}
	}

	// Positions are stored as 32 bit integers relative to offset.
	for (int j = 0; j < 3; j++)
	{
		if (std::max(std::abs(Min[j] - Offset[j]), std::abs(Max[j] - Offset[j])) / ScaleFactor > INT32_MAX)
		{
			LOG.Add("LASPlugin::WriteLASFile: Positions do not fit in LAS file with scale factor " + std::to_string(ScaleFactor), "LAS_PLUGIN", FE_LOG_WARNING);
			return false;
		}
	}

	laszip_POINTER LaszipWriter;
	if (laszip_create(&LaszipWriter))
	{
		LOG.Add("LASPlugin::WriteLASFile: Creating laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);
		return false;
	}

	laszip_header* FileHeader;
	if (laszip_get_header_pointer(LaszipWriter, &FileHeader))
	{
		LOG.Add("LASPlugin::WriteLASFile: Getting header for a file failed", "LAS_PLUGIN", FE_LOG_ERROR);
		if (laszip_destroy(LaszipWriter))
			LOG.Add("LASPlugin::WriteLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);
		return false;
	}

	// Indicate that we have RGB color data in our point format
	FileHeader->point_data_format = 2;  // Format 2 includes RGB values

	// Update the point data record length to account for RGB data
	// Standard record length is 20 bytes, plus 6 bytes for RGB (2 bytes per color channel)
	FileHeader->point_data_record_length = 26;

	// Set appropriate scale factors for data
	// The LAS format stores coordinates as integers internally,
	// but these integers represent real-world coordinates (typically in meters)
	// Scale factors convert between floating-point and integer representations:
	//   integer_value = real_world_value / scale_factor
	//   real_world_value = integer_value * scale_factor
	//
	// For example, with scale_factor = 0.001:
	// - A real-world coordinate of 10.523 meters becomes integer 10523
	// - An integer value of 8421 represents 8.421 meters in real-world space
	//
	FileHeader->x_scale_factor = ScaleFactor;
	FileHeader->y_scale_factor = ScaleFactor;
	FileHeader->z_scale_factor = ScaleFactor;

	FileHeader->x_offset = Offset[0];
	FileHeader->y_offset = Offset[1];
	FileHeader->z_offset = Offset[2];

	FileHeader->min_x = Min[0];
	FileHeader->min_y = Min[1];
	FileHeader->min_z = Min[2];
	FileHeader->max_x = Max[0];
	FileHeader->max_y = Max[1];
	FileHeader->max_z = Max[2];

	FileHeader->number_of_point_records = static_cast<laszip_U32>(PointCount);

	if (laszip_open_writer(LaszipWriter, FilePath.c_str(), bIsCompressed))
	{
		LOG.Add("LASPlugin::WriteLASFile: Opening laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

		laszip_CHAR* Error;
		if (laszip_get_error(LaszipWriter, &Error))
			LOG.Add("LASPlugin::WriteLASFile: Getting laszip error failed", "LAS_PLUGIN", FE_LOG_ERROR);

		LOG.Add("LASPlugin::WriteLASFile: " + std::string(Error), "LAS_PLUGIN", FE_LOG_ERROR);
		if (laszip_destroy(LaszipWriter))
			LOG.Add("LASPlugin::WriteLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);
		return false;
	}

	laszip_point_struct* CurrentPoint = nullptr;
	CurrentPoint = new laszip_point_struct();
	for (size_t i = 0; i < PointCount; i++)
	{
		GetPoint(i, Position, Color);
		CurrentPoint->X = static_cast<laszip_I32>(std::llround((Position[0] - Offset[0]) / FileHeader->x_scale_factor));
		CurrentPoint->Y = static_cast<laszip_I32>(std::llround((Position[1] - Offset[1]) / FileHeader->y_scale_factor));
		CurrentPoint->Z = static_cast<laszip_I32>(std::llround((Position[2] - Offset[2]) / FileHeader->z_scale_factor));

		CurrentPoint->rgb[0] = Color[0];
		CurrentPoint->rgb[1] = Color[1];
		CurrentPoint->rgb[2] = Color[2];

		if (laszip_set_point(LaszipWriter, CurrentPoint))
		{
			LOG.Add("LASPlugin::WriteLASFile: Setting point with index " + std::to_string(i) + " failed", "LAS_PLUGIN", FE_LOG_ERROR);
			delete CurrentPoint;

			if (laszip_close_writer(LaszipWriter))
				LOG.Add("LASPlugin::WriteLASFile: Closing laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

			if (laszip_destroy(LaszipWriter))
				LOG.Add("LASPlugin::WriteLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

			return false;
		}

		if (laszip_write_point(LaszipWriter))
		{
			LOG.Add("LASPlugin::WriteLASFile: Writing point with index " + std::to_string(i) + " failed", "LAS_PLUGIN", FE_LOG_ERROR);
			delete CurrentPoint;

			if (laszip_close_writer(LaszipWriter))
				LOG.Add("LASPlugin::WriteLASFile: Closing laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

			if (laszip_destroy(LaszipWriter))
				LOG.Add("LASPlugin::WriteLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

			return false;
		}
	}

	if (laszip_close_writer(LaszipWriter))
		LOG.Add("LASPlugin::WriteLASFile: Closing laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

	if (laszip_destroy(LaszipWriter))
		LOG.Add("LASPlugin::WriteLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

	delete CurrentPoint;
	return true;
}

std::vector<FEObject*> LASPlugin::ImportPointCloud(const std::string& FilePath)
{
	std::vector<FEObject*> Result;
	LastLoadedData.reset();

	std::unique_ptr<FileData> NewData = std::make_unique<FileData>();
	std::vector<FEPointCloudVertexDouble> Vertices;
	if (!ReadLASFile(FilePath, bKeepAttributesOnImport, *NewData, &Vertices))
		return Result;

	FEPointCloud* PointCloud = RESOURCE_MANAGER.RawDataToFEPointCloud(Vertices, FILE_SYSTEM.GetFileName(FilePath));
	if (PointCloud == nullptr)
		return Result;

	const glm::dvec3 AppliedShift = RESOURCE_MANAGER.GetLastLoadedPointCloudAppliedShift();
	NewData->AppliedShift[0] = AppliedShift.x;
	NewData->AppliedShift[1] = AppliedShift.y;
	NewData->AppliedShift[2] = AppliedShift.z;
	NewData->PointCloudID = UNIQUE_ID.ToString(PointCloud->GetID());
	LastLoadedData = std::move(NewData);

	Result.push_back(PointCloud);
	return Result;
}

bool LASPlugin::ExportPointCloud(FEObject* Object, const std::string& FilePath, bool bIsCompressed)
{
	FEPointCloud* PointCloud = Object == nullptr ? nullptr : RESOURCE_MANAGER.GetPointCloud(Object->GetID());
	if (PointCloud == nullptr)
	{
		LOG.Add("LASPlugin::ExportPointCloud: Object is not a point cloud", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	const std::vector<FEPointCloudVertex> RawData = PointCloud->GetRawData();
	const double NoOffset[3] = { 0.0, 0.0, 0.0 };
	return WriteLASFile(FilePath, RawData.size(), [&](size_t Index, double* Position, uint16_t* Color) {
		Position[0] = static_cast<double>(RawData[Index].X);
		Position[1] = static_cast<double>(RawData[Index].Y);
		Position[2] = static_cast<double>(RawData[Index].Z);

		// Convert 8-bit colors (0-255) to 16-bit colors (0-65535)
		const int CoefficientToConvertTo16Bit = UINT16_MAX / UINT8_MAX;
		Color[0] = static_cast<uint16_t>(RawData[Index].R * CoefficientToConvertTo16Bit);
		Color[1] = static_cast<uint16_t>(RawData[Index].G * CoefficientToConvertTo16Bit);
		Color[2] = static_cast<uint16_t>(RawData[Index].B * CoefficientToConvertTo16Bit);
	}, bIsCompressed, 0.001, NoOffset);
}

const LASPluginInterface::FileData* LASPlugin::ReadFile(const std::string& FilePath, bool bReadAttributes)
{
	LastLoadedData.reset();

	std::unique_ptr<FileData> NewData = std::make_unique<FileData>();
	if (!ReadLASFile(FilePath, bReadAttributes, *NewData, nullptr))
		return nullptr;

	LastLoadedData = std::move(NewData);
	return LastLoadedData.get();
}

const LASPluginInterface::FileData* LASPlugin::GetLastLoadedData() const
{
	return LastLoadedData.get();
}

void LASPlugin::SetKeepAttributesOnImport(bool NewValue)
{
	bKeepAttributesOnImport = NewValue;
}

bool LASPlugin::IsKeepingAttributesOnImport() const
{
	return bKeepAttributesOnImport;
}

bool LASPlugin::SaveFile(const std::string& FilePath, const std::vector<double>& Positions, const std::vector<uint16_t>& Colors, bool bCompressed, double ScaleFactor)
{
	if (Positions.empty() || Positions.size() % 3 != 0 || (!Colors.empty() && Colors.size() != Positions.size()))
	{
		LOG.Add("LASPlugin::SaveFile: Positions should have X, Y and Z of every point, Colors should be empty or have R, G and B of every point", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	if (ScaleFactor <= 0.0)
	{
		LOG.Add("LASPlugin::SaveFile: ScaleFactor should be positive", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	// Minimum as offset keeps stored integers small, also for georeferenced positions.
	double Offset[3] = { Positions[0], Positions[1], Positions[2] };
	for (size_t i = 0; i < Positions.size(); i += 3)
	{
		for (int j = 0; j < 3; j++)
			Offset[j] = std::min(Offset[j], Positions[i + j]);
	}

	return WriteLASFile(FilePath, Positions.size() / 3, [&](size_t Index, double* Position, uint16_t* Color) {
		for (int j = 0; j < 3; j++)
		{
			Position[j] = Positions[Index * 3 + j];
			Color[j] = Colors.empty() ? 0 : Colors[Index * 3 + j];
		}
	}, bCompressed, ScaleFactor, Offset);
}
