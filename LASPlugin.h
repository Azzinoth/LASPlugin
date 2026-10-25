#include "SubSystems/FocalEngine/SubSystems/PluginSystem/FEPluginConnector.h"
#include "Public/LASPluginInterface.h"
using namespace FocalEngine;

class LASPlugin : public FEPluginCore, public LASPluginInterface
{
public:
	void OnLoad() override;
	void OnUnload() override;
	void OnUpdate(double DeltaTime) override;
};