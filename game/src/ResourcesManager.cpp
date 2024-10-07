#include "ResourcesManager.h"

void ResourceManager::AddTexture(const std::string& assetId, const std::string& path)
{
	auto texture = LoadTexture(path.c_str());
	SetTextureFilter(texture, TEXTURE_FILTER_BILINEAR);
	m_textures.emplace(assetId, texture);
}

Texture2D ResourceManager::GetTexture(const std::string& assetId) const
{
	return m_textures.at(assetId);
}

ResourceManager::~ResourceManager()
{
	for (const auto& [id, texture] : m_textures) 
		UnloadTexture(texture);

	m_textures.clear();
}