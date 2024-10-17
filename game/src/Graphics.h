#pragma once

#include <string>
#include <vector>
#include "raylib.h"

struct Vec2;

class Graphics
{
public:
	static int m_windowWidth;
	static int m_windowHeight;

	static int Width();
	static int Height();
	static void OpenWindow();
	static void CloseWindow();
	static void ClearScreen(const Color& color);
	static void DrawLine(const Vec2& startPos, const Vec2& endPos, const Color& color);
	static void DrawCircle(const Vec2& pos, float radius, float angle, const Color& color);
	static void DrawFillCircle(const Vec2& pos, float radius, const Color& color);
	static void DrawRect(const Vec2& pos, int width, int height, const Color& color);
	static void DrawFillRect(const Vec2& pos, int width, int height, const Color& color);
	static void DrawPolygon(const Vec2& pos, const std::vector<Vec2>& vertices, const Color& color);
	static void DrawFillPolygon(const Vec2& center, const std::vector<Vec2>& vertices, const Color& color);
	static void DrawTexture(const Vec2& pos, float width, float height, float rotation, const Texture2D& texture);
	static void DrawText(const std::string& text, const Color& color);
};