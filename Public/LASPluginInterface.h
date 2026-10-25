#pragma once

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

	// Order of virtual functions is part of the interface, new functions should be added only at the end.
	virtual ~LASPluginInterface() = default;
	virtual int GetInterfaceVersion() const { return InterfaceVersion; }
};