#include "SubSystems/FocalEngine/SubSystems/PluginSystem/FEPluginConnector.h"
#include "Public/LASPluginInterface.h"
using namespace FocalEngine;

class LASPlugin : public FEPluginCore, public LASPluginInterface
{
	HMODULE LASzipDLL = nullptr;
	bool bLASzipLoaded = false;

	FEUUID ImporterID;
	FEUUID LASExporterID;
	FEUUID LAZExporterID;

	std::unique_ptr<FileData> LastLoadedData;
	bool bKeepAttributesOnImport = false;

	// With OutVertices positions and colors are saved there instead of OutData.
	bool ReadLASFile(const std::string& FilePath, bool bReadAttributes, FileData& OutData, std::vector<FEPointCloudVertexDouble>* OutVertices);
	bool WriteLASFile(const std::string& FilePath, size_t PointCount, const std::function<void(size_t Index, double* Position, uint16_t* Color)>& GetPoint, bool bIsCompressed, double ScaleFactor, const double* Offset);
	std::vector<FEObject*> ImportPointCloud(const std::string& FilePath);
	bool ExportPointCloud(FEObject* Object, const std::string& FilePath, bool bIsCompressed);
public:
	bool OnLoad() override;
	void OnUnload() override;
	void OnUpdate(double DeltaTime) override;

	const FileData* ReadFile(const std::string& FilePath, bool bReadAttributes) override;
	const FileData* GetLastLoadedData() const override;
	void SetKeepAttributesOnImport(bool NewValue) override;
	bool IsKeepingAttributesOnImport() const override;
	bool SaveFile(const std::string& FilePath, const std::vector<double>& Positions, const std::vector<uint16_t>& Colors, bool bCompressed, double ScaleFactor) override;
};