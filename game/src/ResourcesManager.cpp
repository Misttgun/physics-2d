#include "ResourcesManager.h"

void ResourceManager::AddTexture(const std::string& assetId, const std::string& path)
{
	auto texture = LoadTexture(path.c_str());
	SetTextureFilter(texture, TEXTURE_FILTER_BILINEAR);
	m_textures.emplace(assetId, texture);
}

Texture2D ResourceManager::GetTexture(const std::string& assetId) const
{
	if(const auto found = m_textures.find(assetId); found != m_textures.end())
		return found->second;

	return Texture2D{};
}

ResourceManager::~ResourceManager()
{
	for (const auto& [id, texture] : m_textures) 
		UnloadTexture(texture);

	m_textures.clear();
}