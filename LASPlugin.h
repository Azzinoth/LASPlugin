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

	bool ReadLASFile(const std::string& FilePath, std::vector<FEPointCloudVertexDouble>& OutVertices);
	bool SaveLASFile(std::vector<FEPointCloudVertex>& RawData, const std::string& FilePath, bool bIsCompressed, double ScaleFactor = 0.001);
	bool ExportPointCloud(FEObject* Object, const std::string& FilePath, bool bIsCompressed);
public:
	void OnLoad() override;
	void OnUnload() override;
	void OnUpdate(double DeltaTime) override;
};