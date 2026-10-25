#include "LASPlugin.h"

// laszip_api.h adds "using namespace std;" to the global namespace, after that "byte" in the Windows rpcndr.h is ambiguous with std::byte, so rpcndr.h (through unknwn.h) is included first.
#include <unknwn.h>
#include "ThirdParty/laszip/laszip_api.h"

// DO NOT CHANGE THIS LINE.
// The plugin manager uses it to create and destroy the plugin instance.
REGISTER_PLUGIN(LASPlugin)

void LASPlugin::OnLoad()
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
		LOG.Add("LASPlugin::OnLoad: Loading LASzip DLL failed, LAS and LAZ files are not supported", "LAS_PLUGIN", FE_LOG_ERROR);
		return;
	}
	bLASzipLoaded = true;

	// Point clouds are created on GPU.
	if (APPLICATION.GetGraphicsMode() == FEGraphicsMode::None)
		return;

	FEResourceImporter Importer;
	Importer.Description = "LAS point cloud";
	Importer.Extensions = { "las", "laz" };
	Importer.Import = [this](const std::string& FilePath, FEScene* Scene) {
		std::vector<FEObject*> Result;
		std::vector<FEPointCloudVertexDouble> Vertices;
		if (!ReadLASFile(FilePath, Vertices))
			return Result;

		FEPointCloud* PointCloud = RESOURCE_MANAGER.RawDataToFEPointCloud(Vertices, FILE_SYSTEM.GetFileName(FilePath));
		if (PointCloud != nullptr)
			Result.push_back(PointCloud);

		return Result;
	};
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
}

void LASPlugin::OnUnload()
{
	RESOURCE_MANAGER.UnregisterResourceImporter(ImporterID);
	RESOURCE_MANAGER.UnregisterResourceExporter(LASExporterID);
	RESOURCE_MANAGER.UnregisterResourceExporter(LAZExporterID);

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

bool LASPlugin::ReadLASFile(const std::string& FilePath, std::vector<FEPointCloudVertexDouble>& OutVertices)
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

	OutVertices.resize(PointCount);

	laszip_U64 PointIndex = 0;
	while (PointIndex < PointCount)
	{
		if (laszip_read_point(LaszipReader))
		{
			LOG.Add("LASPlugin::ReadLASFile: reading point from laszip reader failed", "LAS_PLUGIN", FE_LOG_ERROR);
			CloseAndDestroyReader(LaszipReader, true);
			OutVertices.clear();
			return false;
		}

		OutVertices[PointIndex].X = CurrentPointPointer->X * FileHeader->x_scale_factor + FileHeader->x_offset;
		OutVertices[PointIndex].Y = CurrentPointPointer->Y * FileHeader->y_scale_factor + FileHeader->y_offset;
		OutVertices[PointIndex].Z = CurrentPointPointer->Z * FileHeader->z_scale_factor + FileHeader->z_offset;

		OutVertices[PointIndex].R = unsigned char(CurrentPointPointer->rgb[0] / float(1 << 16) * 255);
		OutVertices[PointIndex].G = unsigned char(CurrentPointPointer->rgb[1] / float(1 << 16) * 255);
		OutVertices[PointIndex].B = unsigned char(CurrentPointPointer->rgb[2] / float(1 << 16) * 255);

		PointIndex++;
	}

	CloseAndDestroyReader(LaszipReader, true);
	return true;
}

bool LASPlugin::SaveLASFile(std::vector<FEPointCloudVertex>& RawData, const std::string& FilePath, bool bIsCompressed, double ScaleFactor)
{
	if (RawData.empty())
	{
		LOG.Add("LASPlugin::SaveLASFile: RawData is empty", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	if (FilePath.empty())
	{
		LOG.Add("LASPlugin::SaveLASFile: FilePath is empty", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	if (!bLASzipLoaded)
	{
		LOG.Add("LASPlugin::SaveLASFile: LASzip DLL is not loaded", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	laszip_POINTER LaszipWriter;
	if (laszip_create(&LaszipWriter))
	{
		LOG.Add("LASPlugin::SaveLASFile: Creating laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);
		return false;
	}

	laszip_header* FileHeader;
	if (laszip_get_header_pointer(LaszipWriter, &FileHeader))
	{
		LOG.Add("LASPlugin::SaveLASFile: Getting header for a file failed", "LAS_PLUGIN", FE_LOG_ERROR);
		if (laszip_destroy(LaszipWriter))
			LOG.Add("LASPlugin::SaveLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);
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

	FileHeader->number_of_point_records = static_cast<laszip_U32>(RawData.size());

	if (laszip_open_writer(LaszipWriter, FilePath.c_str(), bIsCompressed))
	{
		LOG.Add("LASPlugin::SaveLASFile: Opening laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

		laszip_CHAR* Error;
		if (laszip_get_error(LaszipWriter, &Error))
			LOG.Add("LASPlugin::SaveLASFile: Getting laszip error failed", "LAS_PLUGIN", FE_LOG_ERROR);

		LOG.Add("LASPlugin::SaveLASFile: " + std::string(Error), "LAS_PLUGIN", FE_LOG_ERROR);
		if (laszip_destroy(LaszipWriter))
			LOG.Add("LASPlugin::SaveLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);
		return false;
	}

	laszip_point_struct* CurrentPoint = nullptr;
	CurrentPoint = new laszip_point_struct();
	for (size_t i = 0; i < RawData.size(); i++)
	{
		CurrentPoint->X = static_cast<laszip_I32>(static_cast<double>(RawData[i].X) / FileHeader->x_scale_factor);
		CurrentPoint->Y = static_cast<laszip_I32>(static_cast<double>(RawData[i].Y) / FileHeader->y_scale_factor);
		CurrentPoint->Z = static_cast<laszip_I32>(static_cast<double>(RawData[i].Z) / FileHeader->z_scale_factor);

		// Convert 8-bit colors (0-255) to 16-bit colors (0-65535)
		int CoefficientToConvertTo16Bit = UINT16_MAX / UINT8_MAX;
		CurrentPoint->rgb[0] = static_cast<laszip_U16>(RawData[i].R * CoefficientToConvertTo16Bit);
		CurrentPoint->rgb[1] = static_cast<laszip_U16>(RawData[i].G * CoefficientToConvertTo16Bit);
		CurrentPoint->rgb[2] = static_cast<laszip_U16>(RawData[i].B * CoefficientToConvertTo16Bit);

		if (laszip_set_point(LaszipWriter, CurrentPoint))
		{
			LOG.Add("LASPlugin::SaveLASFile: Setting point with index " + std::to_string(i) + " failed", "LAS_PLUGIN", FE_LOG_ERROR);
			delete CurrentPoint;

			if (laszip_close_writer(LaszipWriter))
				LOG.Add("LASPlugin::SaveLASFile: Closing laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

			if (laszip_destroy(LaszipWriter))
				LOG.Add("LASPlugin::SaveLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

			return false;
		}

		if (laszip_write_point(LaszipWriter))
		{
			LOG.Add("LASPlugin::SaveLASFile: Writing point with index " + std::to_string(i) + " failed", "LAS_PLUGIN", FE_LOG_ERROR);
			delete CurrentPoint;

			if (laszip_close_writer(LaszipWriter))
				LOG.Add("LASPlugin::SaveLASFile: Closing laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

			if (laszip_destroy(LaszipWriter))
				LOG.Add("LASPlugin::SaveLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

			return false;
		}
	}

	if (laszip_close_writer(LaszipWriter))
		LOG.Add("LASPlugin::SaveLASFile: Closing laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

	if (laszip_destroy(LaszipWriter))
		LOG.Add("LASPlugin::SaveLASFile: Destroying laszip writer failed", "LAS_PLUGIN", FE_LOG_ERROR);

	delete CurrentPoint;
	return true;
}

bool LASPlugin::ExportPointCloud(FEObject* Object, const std::string& FilePath, bool bIsCompressed)
{
	FEPointCloud* PointCloud = Object == nullptr ? nullptr : RESOURCE_MANAGER.GetPointCloud(Object->GetID());
	if (PointCloud == nullptr)
	{
		LOG.Add("LASPlugin::ExportPointCloud: Object is not a point cloud", "LAS_PLUGIN", FE_LOG_WARNING);
		return false;
	}

	std::vector<FEPointCloudVertex> RawData = PointCloud->GetRawData();
	return SaveLASFile(RawData, FilePath, bIsCompressed);
}